# SPDX-License-Identifier: LGPL-2.1-or-later

# ***************************************************************************
# *   Copyright (c) 2026 FreeCAD contributors                              *
# *                                                                         *
# *   This program is free software; you can redistribute it and/or modify  *
# *   it under the terms of the GNU Lesser General Public License (LGPL)    *
# *   as published by the Free Software Foundation; either version 2 of     *
# *   the License, or (at your option) any later version.                   *
# *   for detail see the LICENCE text file.                                 *
# *                                                                         *
# *   This program is distributed in the hope that it will be useful,       *
# *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
# *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
# *   GNU Library General Public License for more details.                  *
# *                                                                         *
# *   You should have received a copy of the GNU Library General Public     *
# *   License along with this program; if not, write to the Free Software   *
# *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  *
# *   USA                                                                   *
# *                                                                         *
# ***************************************************************************

"""CAM Nesting — arranges model objects within stock bounds for machining.

Uses a **no-fit polygon** (NFP) approach for packing.  Each 3D model is
projected to a 2D convex-hull outline.  The NFP between every pair of
(possibly rotated) parts — and between each part and the stock boundary —
determines the exact set of collision-free placement positions.  A
bottom-left-fill strategy picks the best position from those candidates,
scored by the active bias / gravity mode.

The solver is isolated in ``NFPPacker`` so it can be tested and improved
independently of the FreeCAD object model.
"""

import math

import FreeCAD
import Part
import Path
import Path.Main.Stock as PathStock

from PySide.QtCore import QT_TRANSLATE_NOOP

translate = FreeCAD.Qt.translate

if False:
    Path.Log.setLevel(Path.Log.Level.DEBUG, Path.Log.thisModule())
    Path.Log.trackModule(Path.Log.thisModule())
else:
    Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())

# Geometric tolerance used throughout (mm).
_EPS = 1e-9


# ---------------------------------------------------------------------------
#   Packing bias modes
# ---------------------------------------------------------------------------


class PackingBias:
    """Supported directional bias modes for the nesting packer.

    Direction modes (``Front``, ``Back``, ``Left``, ``Right``) correspond
    to the FreeCAD navigation cube when viewed from above (top-down XY):

      * **Front** — cluster toward −Y (toward the viewer)
      * **Back**  — cluster toward +Y (away from the viewer)
      * **Left**  — cluster toward −X
      * **Right** — cluster toward +X

    ``Center`` attracts parts towards the centre of the stock.

    ``CustomPoint`` attracts parts towards an arbitrary (x, y) coordinate
    supplied by the caller.
    """

    Front = "Front"
    Back = "Back"
    Left = "Left"
    Right = "Right"
    Center = "Center"
    CustomPoint = "CustomPoint"

    ALL = [
        Front,
        Back,
        Left,
        Right,
        Center,
        CustomPoint,
    ]

    # Labels shown in the GUI combo-box (same order as ALL).
    LABELS = [
        QT_TRANSLATE_NOOP("CAM_Nesting", "Front"),
        QT_TRANSLATE_NOOP("CAM_Nesting", "Back"),
        QT_TRANSLATE_NOOP("CAM_Nesting", "Left"),
        QT_TRANSLATE_NOOP("CAM_Nesting", "Right"),
        QT_TRANSLATE_NOOP("CAM_Nesting", "Center"),
        QT_TRANSLATE_NOOP("CAM_Nesting", "Custom Point"),
    ]

    @classmethod
    def isDirection(cls, bias):
        return bias in (cls.Front, cls.Back, cls.Left, cls.Right)


# ---------------------------------------------------------------------------
#   2-D polygon helpers
# ---------------------------------------------------------------------------

# A "polygon" throughout this module is a list of (x, y) tuples describing
# a **counter-clockwise** simple polygon.  The last point is NOT a duplicate
# of the first (the closing edge is implicit).


def _polygon_area_signed(poly):
    """Signed area of *poly* (positive ↔ counter-clockwise)."""
    n = len(poly)
    if n < 3:
        return 0.0
    a = 0.0
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        a += x0 * y1 - x1 * y0
    return a / 2.0


def _ensure_ccw(poly):
    """Return *poly* in counter-clockwise order."""
    if _polygon_area_signed(poly) < 0:
        return list(reversed(poly))
    return list(poly)


def _ensure_cw(poly):
    """Return *poly* in clockwise order."""
    if _polygon_area_signed(poly) >= 0:
        return list(reversed(poly))
    return list(poly)


def _polygon_bounds(poly):
    """Return (xmin, ymin, xmax, ymax) of *poly*."""
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    return min(xs), min(ys), max(xs), max(ys)


def _translate_polygon(poly, dx, dy):
    return [(x + dx, y + dy) for x, y in poly]


def _rotate_polygon(poly, angle_deg):
    """Rotate *poly* about the origin by *angle_deg* degrees."""
    if abs(angle_deg) < _EPS:
        return list(poly)
    rad = math.radians(angle_deg)
    cos_a = math.cos(rad)
    sin_a = math.sin(rad)
    return [(x * cos_a - y * sin_a, x * sin_a + y * cos_a) for x, y in poly]


def _mirror_polygon(poly, mirror_x, mirror_y, sx, sy):
    """Mirror *poly* about stock centre (sx/2, sy/2) on the requested axes."""
    pts = list(poly)
    if mirror_x:
        pts = [(sx - x, y) for x, y in pts]
    if mirror_y:
        pts = [(x, sy - y) for x, y in pts]
    # Mirroring reverses winding — flip back.
    if mirror_x != mirror_y:  # exactly one mirror ↔ winding flipped
        pts.reverse()
    return pts


def _convex_hull(points):
    """Andrew's monotone-chain convex hull.  Returns CCW polygon."""
    pts = sorted(set(points))
    if len(pts) <= 1:
        return list(pts)
    lower = []
    for p in pts:
        while len(lower) >= 2 and _cross2(lower[-2], lower[-1], p) <= 0:
            lower.pop()
        lower.append(p)
    upper = []
    for p in reversed(pts):
        while len(upper) >= 2 and _cross2(upper[-2], upper[-1], p) <= 0:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def _cross2(o, a, b):
    return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])


# ---------------------------------------------------------------------------
#   No-fit polygon computation
# ---------------------------------------------------------------------------


def _minkowski_sum_convex(A, B):
    """Minkowski sum of two *convex* CCW polygons → convex CCW polygon.

    Uses the rotating-calipers / edge-merge approach (O(n+m)).
    """
    # Find the bottom-most-then-leftmost vertex of each polygon.
    def _start(poly):
        best = 0
        for i in range(1, len(poly)):
            if (poly[i][1], poly[i][0]) < (poly[best][1], poly[best][0]):
                best = i
        return best

    na, nb = len(A), len(B)
    ia, ib = _start(A), _start(B)

    result = []
    ca = cb = 0
    while ca < na or cb < nb:
        result.append((A[ia % na][0] + B[ib % nb][0],
                        A[ia % na][1] + B[ib % nb][1]))
        # Edge vectors.
        ea = ((A[(ia + 1) % na][0] - A[ia % na][0]),
              (A[(ia + 1) % na][1] - A[ia % na][1]))
        eb = ((B[(ib + 1) % nb][0] - B[ib % nb][0]),
              (B[(ib + 1) % nb][1] - B[ib % nb][1]))
        cross = ea[0] * eb[1] - ea[1] * eb[0]
        if cross > _EPS:
            ia += 1; ca += 1
        elif cross < -_EPS:
            ib += 1; cb += 1
        else:
            ia += 1; ca += 1
            ib += 1; cb += 1

    return _convex_hull(result)  # hull cleans up any collinear edges


def _negate_polygon(poly):
    """Return −P (reflect through origin)."""
    return [(-x, -y) for x, y in poly]


def _compute_nfp(static_poly, orbiting_poly):
    """No-fit polygon of *orbiting_poly* around *static_poly*.

    Both must be convex CCW polygons.  The NFP is the Minkowski sum of
    *static_poly* and the reflection of *orbiting_poly*, which gives the
    locus of positions for the **reference point** (origin) of the
    orbiting polygon such that the two polygons just touch or overlap.

    The *boundary* of this NFP is the set of touching positions; the
    *interior* is the set of overlapping positions.  A valid placement
    therefore lies **outside** the NFP.
    """
    neg_orb = _negate_polygon(orbiting_poly)
    neg_orb = _ensure_ccw(neg_orb)
    return _minkowski_sum_convex(static_poly, neg_orb)


def _compute_ifp(stock_w, stock_h, part_poly):
    """Inner-fit polygon: valid reference-point positions inside stock.

    The stock is the rectangle [0, stock_w] × [0, stock_h].
    Returns a CCW polygon (which will be a rectangle for a convex part).
    """
    xmin, ymin, xmax, ymax = _polygon_bounds(part_poly)
    # The reference point can range such that the whole part fits inside.
    left = -xmin
    bottom = -ymin
    right = stock_w - xmax
    top = stock_h - ymax
    if right < left - _EPS or top < bottom - _EPS:
        return []  # part doesn't fit in stock at all
    return [(left, bottom), (right, bottom), (right, top), (left, top)]


def _point_in_convex_polygon(px, py, poly):
    """Test if (px, py) is inside or on the boundary of convex CCW *poly*."""
    n = len(poly)
    if n < 3:
        return False
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        cross = (x1 - x0) * (py - y0) - (y1 - y0) * (px - x0)
        if cross < -_EPS:
            return False
    return True


def _segments_intersect_or_touch(a0, a1, b0, b1):
    """Return True if segment a0-a1 intersects/touches segment b0-b1."""
    d1 = _cross2(a0, a1, b0)
    d2 = _cross2(a0, a1, b1)
    d3 = _cross2(b0, b1, a0)
    d4 = _cross2(b0, b1, a1)
    if ((d1 > 0 and d2 < 0) or (d1 < 0 and d2 > 0)) and \
       ((d3 > 0 and d4 < 0) or (d3 < 0 and d4 > 0)):
        return True
    if abs(d1) < _EPS and _on_segment(a0, b0, a1):
        return True
    if abs(d2) < _EPS and _on_segment(a0, b1, a1):
        return True
    if abs(d3) < _EPS and _on_segment(b0, a0, b1):
        return True
    if abs(d4) < _EPS and _on_segment(b0, a1, b1):
        return True
    return False


def _on_segment(p, q, r):
    return (min(p[0], r[0]) - _EPS <= q[0] <= max(p[0], r[0]) + _EPS and
            min(p[1], r[1]) - _EPS <= q[1] <= max(p[1], r[1]) + _EPS)


# ---------------------------------------------------------------------------
#   IFP clipping against NFPs
# ---------------------------------------------------------------------------


def _clip_ifp_edges_against_nfps(ifp, nfps):
    """Return candidate positions along the IFP boundary that are outside
    all NFPs.

    Instead of full polygon clipping we sample the IFP edges and also
    collect all intersection points between IFP edges and NFP edges.
    This gives us the "transition" points where a sliding placement enters
    or exits a forbidden zone, plus a regular sampling for robustness.
    """
    candidates = []
    n_ifp = len(ifp)
    if n_ifp < 2:
        return candidates

    for i in range(n_ifp):
        a0 = ifp[i]
        a1 = ifp[(i + 1) % n_ifp]
        # Collect parameter values along the edge where intersections occur.
        params = [0.0, 1.0]

        ex = a1[0] - a0[0]
        ey = a1[1] - a0[1]
        edge_len_sq = ex * ex + ey * ey
        if edge_len_sq < _EPS * _EPS:
            # Degenerate edge.
            pt = a0
            if _position_valid(pt[0], pt[1], nfps):
                candidates.append(pt)
            continue

        for nfp in nfps:
            n_nfp = len(nfp)
            for j in range(n_nfp):
                b0 = nfp[j]
                b1 = nfp[(j + 1) % n_nfp]
                t = _edge_intersection_param(a0, a1, b0, b1)
                if t is not None:
                    params.append(t)

        params.sort()
        # Deduplicate.
        deduped = [params[0]]
        for p in params[1:]:
            if p - deduped[-1] > _EPS:
                deduped.append(p)
        params = deduped

        # Also add midpoints between consecutive params so we don't miss
        # valid segments.
        all_t = []
        for p in params:
            all_t.append(p)
        for k in range(len(params) - 1):
            all_t.append((params[k] + params[k + 1]) / 2.0)
        all_t.sort()

        for t in all_t:
            if t < -_EPS or t > 1.0 + _EPS:
                continue
            t = max(0.0, min(1.0, t))
            px = a0[0] + t * ex
            py = a0[1] + t * ey
            if _position_valid(px, py, nfps):
                candidates.append((px, py))

    # Also test IFP vertices.
    for pt in ifp:
        if _position_valid(pt[0], pt[1], nfps):
            candidates.append(pt)

    return candidates


def _edge_intersection_param(a0, a1, b0, b1):
    """Return parameter *t* ∈ [0, 1] for the intersection of segments
    a0→a1 and b0→b1, or None."""
    dx = a1[0] - a0[0]
    dy = a1[1] - a0[1]
    bx = b1[0] - b0[0]
    by = b1[1] - b0[1]
    denom = dx * by - dy * bx
    if abs(denom) < _EPS:
        return None
    t = ((b0[0] - a0[0]) * by - (b0[1] - a0[1]) * bx) / denom
    u = ((b0[0] - a0[0]) * dy - (b0[1] - a0[1]) * dx) / denom
    if -_EPS <= t <= 1.0 + _EPS and -_EPS <= u <= 1.0 + _EPS:
        return max(0.0, min(1.0, t))
    return None


def _position_valid(px, py, nfps):
    """Return True if (px, py) is outside all NFPs (on boundary is OK)."""
    for nfp in nfps:
        if _strictly_inside_convex(px, py, nfp):
            return False
    return True


def _strictly_inside_convex(px, py, poly):
    """True if (px,py) is strictly inside convex CCW *poly* (not on edge)."""
    n = len(poly)
    if n < 3:
        return False
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        cross = (x1 - x0) * (py - y0) - (y1 - y0) * (px - x0)
        if cross <= _EPS:   # on or outside this edge → not strictly inside
            return False
    return True


# ---------------------------------------------------------------------------
#   2-D outline extraction from FreeCAD shapes
# ---------------------------------------------------------------------------


def _extract_outline_2d(model):
    """Return a convex-hull polygon [(x,y), ...] of *model*'s XY projection.

    The shape is reset to identity placement so the outline is in the
    model's local frame.  For 2.5-D nesting (sheet cutting) this gives a
    tight convex boundary of the part when viewed from above.
    """
    shape = model.Shape.copy()
    shape.Placement = FreeCAD.Placement()

    points_2d = []

    # Collect vertices from all edges/faces projected onto XY.
    for v in shape.Vertexes:
        points_2d.append((v.X, v.Y))

    # Also sample edges to capture curved outlines.
    for edge in shape.Edges:
        try:
            fp = edge.FirstParameter
            lp = edge.LastParameter
            n_samples = max(8, int((lp - fp) / 0.5))
            for i in range(n_samples + 1):
                t = fp + (lp - fp) * i / n_samples
                pt = edge.valueAt(t)
                points_2d.append((pt.x, pt.y))
        except Exception:
            pass

    if len(points_2d) < 3:
        # Fallback to bounding box (normalised to origin).
        bb = shape.BoundBox
        return [
            (0.0, 0.0),
            (bb.XLength, 0.0),
            (bb.XLength, bb.YLength),
            (0.0, bb.YLength),
        ]

    hull = _convex_hull(points_2d)
    if len(hull) < 3:
        bb = shape.BoundBox
        hull = [
            (bb.XMin, bb.YMin),
            (bb.XMax, bb.YMin),
            (bb.XMax, bb.YMax),
            (bb.XMin, bb.YMax),
        ]
    hull = _ensure_ccw(hull)

    # Normalise so the bounding-box minimum sits at (0, 0).
    # This makes the packer's reference point == the outline's lower-left
    # corner, which maps directly to (lbb.XMin, lbb.YMin) in local space.
    xmin, ymin, _xmax, _ymax = _polygon_bounds(hull)
    if abs(xmin) > _EPS or abs(ymin) > _EPS:
        hull = [(x - xmin, y - ymin) for x, y in hull]
    return hull


def _offset_polygon(poly, distance):
    """Outward-offset a convex CCW polygon by *distance* (Minkowski sum
    with a circle, approximated by expanding along edge normals then
    re-hulling).

    For spacing/kerf: each part is expanded by ``spacing / 2`` so that
    when two offset parts touch, the gap between the originals is
    ``spacing``.
    """
    if distance <= _EPS:
        return list(poly)

    n = len(poly)
    if n < 3:
        return list(poly)

    # Compute outward normals for each edge.
    expanded = []
    for i in range(n):
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        ex, ey = x1 - x0, y1 - y0
        length = math.hypot(ex, ey)
        if length < _EPS:
            continue
        # Outward normal for CCW polygon (edge goes from i to i+1,
        # outward is to the right → rotate edge vector CW by 90°).
        nx, ny = ey / length, -ex / length
        # Shift both endpoints of this edge outward.
        expanded.append((x0 + nx * distance, y0 + ny * distance))
        expanded.append((x1 + nx * distance, y1 + ny * distance))

    # Also push each vertex outward along the bisector.
    for i in range(n):
        xp, yp = poly[(i - 1) % n]
        x0, y0 = poly[i]
        x1, y1 = poly[(i + 1) % n]
        # Bisector of the two adjacent edges.
        e_in = (x0 - xp, y0 - yp)
        e_out = (x1 - x0, y1 - y0)
        l_in = math.hypot(*e_in)
        l_out = math.hypot(*e_out)
        if l_in < _EPS or l_out < _EPS:
            continue
        n_in = (e_in[1] / l_in, -e_in[0] / l_in)
        n_out = (e_out[1] / l_out, -e_out[0] / l_out)
        bx = n_in[0] + n_out[0]
        by = n_in[1] + n_out[1]
        bl = math.hypot(bx, by)
        if bl < _EPS:
            continue
        # The bisector offset should be distance / cos(half_angle).
        cos_half = bl / 2.0
        d = distance / max(cos_half, 0.25)  # clamp to avoid huge spikes
        expanded.append((x0 + bx / bl * d, y0 + by / bl * d))

    return _convex_hull(expanded)


# ---------------------------------------------------------------------------
#   NFP-based nesting packer
# ---------------------------------------------------------------------------


class NFPPacker:
    """No-Fit-Polygon packer for convex 2-D outlines.

    Places parts one by one using a bottom-left-fill (BLF) strategy scored
    by the active bias mode.  Rotation search and spacing are handled
    identically to the previous ``RectPacker`` interface.

    Parameters
    ----------
    stock_width, stock_height : float
        Usable stock dimensions.
    spacing : float
        Minimum gap between parts.
    allow_rotation : bool
        Whether to try rotated orientations.
    rotation_step : float
        Rotation increment in degrees (must be ≥ 1).
    bias : str
        One of the ``PackingBias`` constants.
    gravity_point : tuple[float, float] | None
        Target ``(x, y)`` for ``CustomPoint`` bias.
    """

    def __init__(
        self,
        stock_width,
        stock_height,
        spacing=0.0,
        allow_rotation=True,
        rotation_step=90.0,
        bias=PackingBias.Front,
        gravity_point=None,
    ):
        self.stock_width = stock_width
        self.stock_height = stock_height
        self.spacing = spacing
        self.allow_rotation = allow_rotation
        self.rotation_step = max(rotation_step, 1.0)
        self.bias = bias
        self._placed_polys = []  # list of translated+rotated offset polys

        # Pre-compute gravity target.
        if bias == PackingBias.Center:
            self._gravity = (stock_width / 2.0, stock_height / 2.0)
        elif bias == PackingBias.CustomPoint and gravity_point is not None:
            self._gravity = (float(gravity_point[0]), float(gravity_point[1]))
        else:
            self._gravity = None

    # -- public API ---------------------------------------------------------

    def pack(self, outlines):
        """Pack convex *outlines* into the stock.

        Parameters
        ----------
        outlines : list of polygon
            Each polygon is ``[(x, y), ...]`` CCW, in the model's local
            frame (origin = model's local origin, already offset by
            ``spacing / 2`` if desired).

        Returns
        -------
        list[tuple[float, float, float] | None]
            ``(x, y, rotation_deg)`` for each input outline, or ``None``
            for outlines that could not be placed.  ``(x, y)`` is the
            translation applied to the outline's reference point (its
            local origin) so that the part sits inside the stock.
        """
        # Sort by descending area for better packing — place big parts first.
        areas = []
        for poly in outlines:
            areas.append(abs(_polygon_area_signed(poly)))
        indexed = sorted(range(len(outlines)), key=lambda i: -areas[i])

        results = [None] * len(outlines)

        for orig_idx in indexed:
            base_poly = outlines[orig_idx]

            best = self._find_best_placement(base_poly, orig_idx)
            if best is None:
                continue

            px, py, angle, placed_poly = best

            self._placed_polys.append(placed_poly)
            results[orig_idx] = (px, py, angle)

        # --- Center / CustomPoint: shift entire cluster -------------------
        if self._gravity is not None:
            self._shift_to_gravity(results, outlines)

        return results

    def _find_best_placement(self, base_poly, orig_idx):
        """Try all candidate rotations, return the best placement or None."""
        best = None  # (score, px, py, angle, placed_poly)

        for angle in self._candidate_angles():
            rotated = _rotate_polygon(base_poly, angle)
            rotated = _ensure_ccw(rotated)

            # Inner-fit polygon: where the reference point can go.
            ifp = _compute_ifp(self.stock_width, self.stock_height, rotated)
            if not ifp:
                continue  # part doesn't fit at this rotation

            # Compute NFP with every already-placed polygon.
            nfps = []
            for pp in self._placed_polys:
                nfp = _compute_nfp(pp, rotated)
                if nfp and len(nfp) >= 3:
                    nfps.append(nfp)

            # Find candidate positions on the IFP boundary that are
            # outside all NFPs.
            candidates = _clip_ifp_edges_against_nfps(ifp, nfps)

            # If no parts placed yet, all IFP vertices are valid.
            if not self._placed_polys and not candidates:
                candidates = list(ifp)

            # Add NFP vertices as candidates — these represent "touching"
            # positions which often give tight packing.
            for nfp in nfps:
                for pt in nfp:
                    if _point_in_convex_polygon(pt[0], pt[1], ifp) and \
                       _position_valid(pt[0], pt[1], nfps):
                        candidates.append(pt)

            # For gravity-based modes, also try the gravity target and
            # IFP centre directly — boundary-only sampling misses them.
            if self._gravity is not None:
                ifp_bb = _polygon_bounds(ifp)
                extra_pts = [
                    self._gravity,
                    ((ifp_bb[0] + ifp_bb[2]) / 2.0,
                     (ifp_bb[1] + ifp_bb[3]) / 2.0),
                ]
                for pt in extra_pts:
                    if _point_in_convex_polygon(pt[0], pt[1], ifp) and \
                       _position_valid(pt[0], pt[1], nfps):
                        candidates.append(pt)

            for cx, cy in candidates:
                score = self._placement_score(cx, cy, rotated)
                if best is None or score < best[0]:
                    placed = _translate_polygon(rotated, cx, cy)
                    best = (score, cx, cy, angle, placed)

        if best is None:
            return None
        _score, px, py, angle, placed_poly = best
        return (px, py, angle, placed_poly)

    def _shift_to_gravity(self, results, outlines):
        """Translate all placed parts so the cluster centre aligns with
        the gravity target, clamped to stock bounds."""
        placed = [
            (i, results[i]) for i in range(len(results))
            if results[i] is not None
        ]
        if not placed:
            return

        cxmin = float("inf")
        cymin = float("inf")
        cxmax = float("-inf")
        cymax = float("-inf")

        for i, (px, py, angle) in placed:
            rotated = _rotate_polygon(outlines[i], angle)
            translated = _translate_polygon(rotated, px, py)
            bx0, by0, bx1, by1 = _polygon_bounds(translated)
            cxmin = min(cxmin, bx0)
            cymin = min(cymin, by0)
            cxmax = max(cxmax, bx1)
            cymax = max(cymax, by1)

        cluster_cx = (cxmin + cxmax) / 2.0
        cluster_cy = (cymin + cymax) / 2.0
        cluster_w = cxmax - cxmin
        cluster_h = cymax - cymin

        gx, gy = self._gravity
        dx = gx - cluster_cx
        dy = gy - cluster_cy

        if cxmin + dx < 0:
            dx = -cxmin
        if cymin + dy < 0:
            dy = -cymin
        if cxmin + dx + cluster_w > self.stock_width:
            dx = self.stock_width - cluster_w - cxmin
        if cymin + dy + cluster_h > self.stock_height:
            dy = self.stock_height - cluster_h - cymin

        if abs(dx) < _EPS and abs(dy) < _EPS:
            return

        for i, (px, py, angle) in placed:
            results[i] = (px + dx, py + dy, angle)

    # -- internals ----------------------------------------------------------

    def _candidate_angles(self):
        yield 0.0
        if self.allow_rotation:
            a = self.rotation_step
            while a < 180.0:
                yield a
                a += self.rotation_step

    def _placement_score(self, px, py, rotated_poly):
        """Score a candidate position — lower is better.

        Direction biases (navigation-cube convention, top-down XY):
          Front → minimise Y (cluster toward −Y), secondary X
          Back  → maximise Y (cluster toward +Y), secondary −X
          Left  → minimise X (cluster toward −X), secondary Y
          Right → maximise X (cluster toward +X), secondary −Y

        Center / CustomPoint → minimise distance to gravity target,
        with Y as a light tie-breaker.
        """
        if self._gravity is not None:
            gx, gy = self._gravity
            dist = math.hypot(px - gx, py - gy)
            return (dist, py, px)

        # Direction biases — primary axis determines packing wall,
        # secondary axis spreads parts along that wall.
        if self.bias == PackingBias.Front:
            return (py, px)         # low Y first, then low X
        if self.bias == PackingBias.Back:
            return (-py, -px)       # high Y first, then high X
        if self.bias == PackingBias.Left:
            return (px, py)         # low X first, then low Y
        if self.bias == PackingBias.Right:
            return (-px, -py)       # high X first, then high Y
        # Fallback (shouldn't happen).
        return (py, px)


# ---------------------------------------------------------------------------
#   Public API — called directly from the GUI command
# ---------------------------------------------------------------------------


def _localBoundBox(model):
    """Return the bounding box of *model*'s shape in its local (identity-
    placement) coordinate frame."""
    shape = model.Shape.copy()
    shape.Placement = FreeCAD.Placement()
    return shape.BoundBox


def _resolveSource(clone):
    """Return the original body behind a Draft clone, or the object itself."""
    if hasattr(clone, "Objects") and clone.Objects:
        return clone.Objects[0]
    return clone


def _isStockModel(model, stock):
    """Return True if *model* represents the stock body rather than a
    real workpiece."""
    if stock is None:
        return False
    src = _resolveSource(model)
    if model.Name == stock.Name or src.Name == stock.Name:
        return True
    if hasattr(stock, "Base") and stock.Base is not None:
        if src.Name == stock.Base.Name:
            return True
    return False


def _stockType(stock):
    """Return a string describing the stock type."""
    stype = PathStock.StockType.FromStock(stock)
    if stype and stype != PathStock.StockType.Unknown:
        return stype

    if hasattr(stock, "Proxy") and stock.Proxy is not None:
        cls = type(stock.Proxy).__name__
        if "FromBase" in cls:
            return "FromBase"
        if "CreateBox" in cls or "Box" in cls:
            return "CreateBox"
        if "CreateCylinder" in cls or "Cylinder" in cls:
            return "CreateCylinder"

    if hasattr(stock, "ExtXneg") and hasattr(stock, "ExtZpos"):
        return "FromBase"
    if hasattr(stock, "Length") and hasattr(stock, "Width"):
        return "CreateBox"
    if hasattr(stock, "Radius") and hasattr(stock, "Height"):
        return "CreateCylinder"

    return "ExistingSolid"


def _estimateRequiredStock(outlines, spacing, allow_rotation, rotation_step,
                           bias=PackingBias.Front, gravity_point=None,
                           edge_margin=None):
    """Estimate minimum stock dimensions needed to pack *outlines*."""
    if not outlines:
        return (0, 0)

    if edge_margin is None:
        edge_margin = spacing / 2.0
    half_gap = spacing / 2.0
    edge_inset = max(0.0, edge_margin - half_gap)

    total_area = 0
    max_dim = 0
    for poly in outlines:
        total_area += abs(_polygon_area_signed(poly))
        xmin, ymin, xmax, ymax = _polygon_bounds(poly)
        max_dim = max(max_dim, xmax - xmin, ymax - ymin)

    side = max(math.sqrt(total_area * 1.3), max_dim + spacing)
    stock_w = side
    stock_h = side

    for _ in range(20):
        pw = stock_w - 2.0 * edge_inset
        ph = stock_h - 2.0 * edge_inset
        if pw < _EPS or ph < _EPS:
            stock_w *= 1.3
            stock_h *= 1.3
            continue
        packer = NFPPacker(
            pw, ph,
            spacing=0.0,  # outlines are already offset
            allow_rotation=allow_rotation,
            rotation_step=rotation_step,
            bias=bias,
            gravity_point=gravity_point,
        )
        results = packer.pack(outlines)
        if all(r is not None for r in results):
            # Compute actual used extent.
            used_w = 0.0
            used_h = 0.0
            for i, res in enumerate(results):
                if res is None:
                    continue
                px, py, angle = res
                rotated = _rotate_polygon(outlines[i], angle)
                translated = _translate_polygon(rotated, px, py)
                bx0, by0, bx1, by1 = _polygon_bounds(translated)
                used_w = max(used_w, bx1)
                used_h = max(used_h, by1)
            return (used_w + 2.0 * edge_inset + spacing,
                    used_h + 2.0 * edge_inset + spacing)
        stock_w *= 1.3
        stock_h *= 1.3

    return (stock_w, stock_h)


def nestModels(job, spacing=2.0, allow_rotation=True,
               rotation_step=90.0,
               bias=PackingBias.Front, gravity_point=None,
               edge_margin=None):
    """Arrange the models of *job* within its stock bounds.

    Parameters
    ----------
    job : CAM Job document object
    spacing : float
        Minimum gap between parts in mm.
    allow_rotation : bool
        Whether the packer may try rotations.
    rotation_step : float
        Rotation increment in degrees.
    bias : str
        A ``PackingBias`` constant.
    gravity_point : tuple[float, float] | None
        Target ``(x, y)`` for ``PackingBias.CustomPoint``.
    edge_margin : float | None
        Minimum gap between parts and the stock boundary in mm.
        Defaults to ``spacing / 2`` when *None*.

    Returns
    -------
    str
        A human-readable report of what was placed.
    """
    models = job.Model.Group if hasattr(job, "Model") and job.Model else []
    if not models:
        return translate("CAM_Nesting", "Job has no models")

    stock = job.Stock
    if stock is None:
        return translate("CAM_Nesting", "Job has no stock")

    models = [m for m in models if not _isStockModel(m, stock)]
    if not models:
        return translate("CAM_Nesting", "Job has no models (only stock)")

    stype = _stockType(stock)
    gap = float(spacing)
    half_gap = gap / 2.0
    if edge_margin is None:
        edge_margin = half_gap
    else:
        edge_margin = float(edge_margin)
    edge_inset = max(0.0, edge_margin - half_gap)
    rot_step = max(float(rotation_step), 1.0)

    # ------------------------------------------------------------------
    # Extract 2-D outlines and apply spacing offset
    # ------------------------------------------------------------------
    raw_outlines = []
    outlines = []      # offset by half_gap
    local_bbs = []
    for m in models:
        outline = _extract_outline_2d(m)
        raw_outlines.append(outline)
        outlines.append(_offset_polygon(outline, half_gap))
        local_bbs.append(_localBoundBox(m))

    Path.Log.debug("Nesting: %d model(s), stock type '%s'" % (len(models), stype))
    for m, poly in zip(models, raw_outlines):
        bb = _polygon_bounds(poly)
        Path.Log.debug(
            "  '%s'  %.2f x %.2f mm  (%d-gon hull)"
            % (m.Label, bb[2] - bb[0], bb[3] - bb[1], len(poly))
        )

    # ------------------------------------------------------------------
    # Determine packing area
    # ------------------------------------------------------------------
    if stype == "FromBase":
        est_w, est_h = _estimateRequiredStock(
            outlines, gap, allow_rotation, rot_step,
            bias=bias, gravity_point=gravity_point,
            edge_margin=edge_margin,
        )
        stock_w = est_w
        stock_h = est_h
        Path.Log.debug(
            "Nesting: auto-sizing stock to %.2f x %.2f mm"
            % (stock_w, stock_h)
        )
    else:
        sbb = stock.Shape.BoundBox
        stock_w = sbb.XLength
        stock_h = sbb.YLength
        Path.Log.debug(
            "Nesting: using existing stock %.2f x %.2f mm"
            % (stock_w, stock_h)
        )

    if stock_w < _EPS or stock_h < _EPS:
        return translate("CAM_Nesting", "Stock has zero area")

    # ------------------------------------------------------------------
    # Pack (using NFP)
    # ------------------------------------------------------------------
    packer_w = stock_w - 2.0 * edge_inset
    packer_h = stock_h - 2.0 * edge_inset
    if packer_w < _EPS or packer_h < _EPS:
        return translate("CAM_Nesting", "Edge margin too large for stock")

    packer = NFPPacker(
        packer_w,
        packer_h,
        spacing=0.0,  # spacing already baked into outline offsets
        allow_rotation=bool(allow_rotation),
        rotation_step=rot_step,
        bias=bias,
        gravity_point=gravity_point,
    )
    results = packer.pack(outlines)

    # ------------------------------------------------------------------
    # Apply placements
    # ------------------------------------------------------------------
    if stype == "FromBase":
        stock_x0 = edge_inset
        stock_y0 = edge_inset
        stock_z0 = 0.0
    else:
        sbb = stock.Shape.BoundBox
        stock_x0 = sbb.XMin + edge_inset
        stock_y0 = sbb.YMin + edge_inset
        stock_z0 = sbb.ZMin

    placed = 0
    failed = 0
    overflow_x = stock_x0 + stock_w + 10.0
    overflow_y = stock_y0

    for idx, (model, lbb) in enumerate(zip(models, local_bbs)):
        if results[idx] is None:
            bb = _polygon_bounds(raw_outlines[idx])
            FreeCAD.Console.PrintWarning(
                "Nesting: could not fit '%s' (%.2f x %.2f) into stock\n"
                % (model.Label, bb[2] - bb[0], bb[3] - bb[1])
            )
            overflow_translation = FreeCAD.Vector(
                overflow_x - lbb.XMin,
                overflow_y - lbb.YMin,
                stock_z0 - lbb.ZMin,
            )
            model.Placement = FreeCAD.Placement(
                overflow_translation, FreeCAD.Rotation()
            )
            overflow_y += lbb.YLength + 5.0
            failed += 1
            continue

        nest_x, nest_y, angle = results[idx]
        rot = FreeCAD.Rotation(FreeCAD.Vector(0, 0, 1), angle)

        # The packer placed the *normalised* outline at (nest_x, nest_y).
        # The normalised outline's (0,0) corresponds to
        # (lbb.XMin, lbb.YMin, lbb.ZMin) in the model's local frame.
        #
        # When FreeCAD applies Placement = (translation, rotation), it
        # first rotates the shape (in local coords) about the origin,
        # then translates.  So the local point (lbb.XMin, lbb.YMin,
        # lbb.ZMin) ends up at:
        #     rot * (XMin, YMin, ZMin) + translation
        #
        # We want that to equal (stock_x0 + nest_x, stock_y0 + nest_y,
        # stock_z0).  Therefore:
        #     translation = target - rot * local_min
        local_min = FreeCAD.Vector(lbb.XMin, lbb.YMin, lbb.ZMin)
        rotated_min = rot.multVec(local_min)

        target = FreeCAD.Vector(
            stock_x0 + nest_x,
            stock_y0 + nest_y,
            stock_z0,
        )
        translation = target - rotated_min

        model.Placement = FreeCAD.Placement(translation, rot)
        placed += 1

        Path.Log.debug(
            "  placed '%s' at (%.2f, %.2f) rot %.1f°"
            % (model.Label, target.x, target.y, angle)
        )

    # ------------------------------------------------------------------
    # FromBase stock: update placement and extensions
    # ------------------------------------------------------------------
    if stype == "FromBase" and placed > 0:
        # Compute the global bounding box of the newly-placed models so
        # the stock wraps tightly around them.
        models_bb = FreeCAD.BoundBox()
        for idx, model in enumerate(models):
            if results[idx] is not None:
                models_bb.add(model.Shape.BoundBox)

        # The stock's Placement.Base must match the models' new BB
        # origin so that StockFromBase.execute positions the box
        # correctly around the moved parts.
        stock.Placement = FreeCAD.Placement(
            FreeCAD.Vector(models_bb.XMin, models_bb.YMin, models_bb.ZMin),
            FreeCAD.Rotation(),
        )

        stock.ExtXneg = edge_margin
        stock.ExtXpos = edge_margin
        stock.ExtYneg = edge_margin
        stock.ExtYpos = edge_margin

    report = translate("CAM_Nesting", "Placed %d of %d models") % (
        placed, placed + failed,
    )
    if failed:
        report += "  —  " + (
            translate("CAM_Nesting", "%d could not fit") % failed
        )
    Path.Log.info("Nesting: " + report)
    return report

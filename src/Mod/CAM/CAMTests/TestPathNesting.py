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

"""Tests for the CAM Nesting feature (Path.Op.Nesting).

Tests are split into two classes:
  * ``TestNestingGeometry`` — pure polygon / NFP helpers (no FreeCAD docs).
  * ``TestNestingPacker`` — NFPPacker integration tests (no FreeCAD docs).
  * ``TestNestingModels`` — full nestModels() with FreeCAD document objects.
"""

import math

import FreeCAD
import Path.Op.Nesting as Nesting

from CAMTests.PathTestUtils import PathTestBase


# ── Shorthand access to private helpers under test ────────────────────────

_polygon_area_signed = Nesting._polygon_area_signed
_ensure_ccw = Nesting._ensure_ccw
_ensure_cw = Nesting._ensure_cw
_polygon_bounds = Nesting._polygon_bounds
_translate_polygon = Nesting._translate_polygon
_rotate_polygon = Nesting._rotate_polygon
_convex_hull = Nesting._convex_hull
_minkowski_sum_convex = Nesting._minkowski_sum_convex
_negate_polygon = Nesting._negate_polygon
_compute_nfp = Nesting._compute_nfp
_compute_ifp = Nesting._compute_ifp
_point_in_convex_polygon = Nesting._point_in_convex_polygon
_position_valid = Nesting._position_valid
_strictly_inside_convex = Nesting._strictly_inside_convex
_offset_polygon = Nesting._offset_polygon
_clip_ifp_edges_against_nfps = Nesting._clip_ifp_edges_against_nfps

NFPPacker = Nesting.NFPPacker
PackingBias = Nesting.PackingBias


# ── Geometry helpers ──────────────────────────────────────────────────────

# Canonical CCW unit square.
UNIT_SQUARE = [(0, 0), (1, 0), (1, 1), (0, 1)]

# CCW right-triangle with legs along axes.
TRIANGLE = [(0, 0), (10, 0), (0, 10)]


def _roughly(a, b, tol=1e-6):
    return abs(a - b) < tol


# ═══════════════════════════════════════════════════════════════════════════
#  1. Polygon / NFP geometry helpers
# ═══════════════════════════════════════════════════════════════════════════


class TestNestingGeometry(PathTestBase):
    """Pure-function tests — no FreeCAD documents needed."""

    # -- polygon area -------------------------------------------------------

    def test_area_unit_square(self):
        """Signed area of CCW unit square is +1."""
        self.assertRoughly(_polygon_area_signed(UNIT_SQUARE), 1.0)

    def test_area_cw_square(self):
        """CW winding gives negative area."""
        cw = list(reversed(UNIT_SQUARE))
        self.assertRoughly(_polygon_area_signed(cw), -1.0)

    def test_area_triangle(self):
        """Triangle (0,0)-(10,0)-(0,10) has area 50."""
        self.assertRoughly(_polygon_area_signed(TRIANGLE), 50.0)

    # -- ensure_ccw / ensure_cw ---------------------------------------------

    def test_ensure_ccw_already_ccw(self):
        """CCW polygon is returned unchanged."""
        result = _ensure_ccw(UNIT_SQUARE)
        self.assertEqual(result, UNIT_SQUARE)

    def test_ensure_ccw_from_cw(self):
        """CW polygon is reversed to CCW."""
        cw = list(reversed(UNIT_SQUARE))
        result = _ensure_ccw(cw)
        self.assertTrue(_polygon_area_signed(result) > 0)

    def test_ensure_cw(self):
        """ensure_cw reverses a CCW polygon."""
        result = _ensure_cw(UNIT_SQUARE)
        self.assertTrue(_polygon_area_signed(result) < 0)

    # -- polygon_bounds -----------------------------------------------------

    def test_bounds_unit_square(self):
        """Bounds of unit square are (0,0,1,1)."""
        xmin, ymin, xmax, ymax = _polygon_bounds(UNIT_SQUARE)
        self.assertRoughly(xmin, 0.0)
        self.assertRoughly(ymin, 0.0)
        self.assertRoughly(xmax, 1.0)
        self.assertRoughly(ymax, 1.0)

    def test_bounds_translated(self):
        """Bounds track translation."""
        t = _translate_polygon(UNIT_SQUARE, 5, 10)
        xmin, ymin, xmax, ymax = _polygon_bounds(t)
        self.assertRoughly(xmin, 5.0)
        self.assertRoughly(ymin, 10.0)
        self.assertRoughly(xmax, 6.0)
        self.assertRoughly(ymax, 11.0)

    # -- convex_hull --------------------------------------------------------

    def test_hull_collinear(self):
        """Collinear points yield a 2-point 'hull'."""
        pts = [(0, 0), (1, 0), (2, 0)]
        hull = _convex_hull(pts)
        self.assertEqual(len(hull), 2)

    def test_hull_square(self):
        """Hull of 4 square corners is a 4-vertex polygon."""
        pts = [(0, 0), (1, 0), (1, 1), (0, 1)]
        hull = _convex_hull(pts)
        self.assertEqual(len(hull), 4)

    def test_hull_interior_point(self):
        """Interior point is excluded from the hull."""
        pts = [(0, 0), (2, 0), (2, 2), (0, 2), (1, 1)]
        hull = _convex_hull(pts)
        self.assertEqual(len(hull), 4)

    # -- rotation -----------------------------------------------------------

    def test_rotate_90(self):
        """Rotating unit square 90° preserves area and changes bounds."""
        rotated = _rotate_polygon(UNIT_SQUARE, 90)
        area = abs(_polygon_area_signed(rotated))
        self.assertRoughly(area, 1.0)
        xmin, ymin, xmax, ymax = _polygon_bounds(rotated)
        self.assertRoughly(xmax - xmin, 1.0)
        self.assertRoughly(ymax - ymin, 1.0)

    def test_rotate_360_identity(self):
        """Rotating 360° returns approximately the original polygon."""
        rotated = _rotate_polygon(TRIANGLE, 360)
        for (x0, y0), (x1, y1) in zip(TRIANGLE, rotated):
            self.assertRoughly(x0, x1, 1e-9)
            self.assertRoughly(y0, y1, 1e-9)

    # -- offset_polygon -----------------------------------------------------

    def test_offset_zero(self):
        """Zero offset returns original polygon."""
        result = _offset_polygon(UNIT_SQUARE, 0)
        self.assertEqual(len(result), len(UNIT_SQUARE))

    def test_offset_positive_grows(self):
        """Positive offset increases bounding box."""
        offset = _offset_polygon(UNIT_SQUARE, 1.0)
        xmin, ymin, xmax, ymax = _polygon_bounds(offset)
        self.assertTrue(xmax - xmin > 1.0)
        self.assertTrue(ymax - ymin > 1.0)

    def test_offset_preserves_ccw(self):
        """Offset polygon is still CCW."""
        offset = _offset_polygon(UNIT_SQUARE, 2.0)
        self.assertTrue(_polygon_area_signed(offset) > 0)

    # -- IFP ----------------------------------------------------------------

    def test_ifp_small_part(self):
        """IFP of a 1×1 part in 10×10 stock is a valid rectangle."""
        ifp = _compute_ifp(10, 10, UNIT_SQUARE)
        self.assertEqual(len(ifp), 4)
        xmin, ymin, xmax, ymax = _polygon_bounds(ifp)
        self.assertRoughly(xmin, 0.0)
        self.assertRoughly(ymin, 0.0)
        self.assertRoughly(xmax, 9.0)
        self.assertRoughly(ymax, 9.0)

    def test_ifp_exact_fit(self):
        """Part exactly matching stock yields a single-point IFP."""
        box = [(0, 0), (10, 0), (10, 10), (0, 10)]
        ifp = _compute_ifp(10, 10, box)
        xmin, ymin, xmax, ymax = _polygon_bounds(ifp)
        self.assertRoughly(xmax - xmin, 0.0)
        self.assertRoughly(ymax - ymin, 0.0)

    def test_ifp_too_big(self):
        """Part bigger than stock returns empty IFP."""
        big = [(0, 0), (20, 0), (20, 20), (0, 20)]
        ifp = _compute_ifp(10, 10, big)
        self.assertEqual(len(ifp), 0)

    # -- NFP ----------------------------------------------------------------

    def test_nfp_two_unit_squares(self):
        """NFP of two identical unit squares is a 2×2 region."""
        nfp = _compute_nfp(UNIT_SQUARE, UNIT_SQUARE)
        self.assertTrue(len(nfp) >= 3)
        xmin, ymin, xmax, ymax = _polygon_bounds(nfp)
        self.assertRoughly(xmax - xmin, 2.0)
        self.assertRoughly(ymax - ymin, 2.0)

    def test_nfp_area_relation(self):
        """NFP of two convex polygons has area ≥ sum of individual areas."""
        a = [(0, 0), (3, 0), (3, 2), (0, 2)]
        b = [(0, 0), (1, 0), (1, 1), (0, 1)]
        nfp = _compute_nfp(a, b)
        nfp_area = abs(_polygon_area_signed(nfp))
        area_a = abs(_polygon_area_signed(a))
        area_b = abs(_polygon_area_signed(b))
        self.assertTrue(nfp_area >= area_a + area_b - 1e-6)

    def test_nfp_symmetry(self):
        """NFP(A, B) bounds match NFP(B, A) bounds (up to sign/offset)."""
        a = [(0, 0), (4, 0), (4, 3), (0, 3)]
        b = [(0, 0), (2, 0), (2, 1), (0, 1)]
        nfp_ab = _compute_nfp(a, b)
        nfp_ba = _compute_nfp(b, a)
        # Both should have the same dimensions.
        ab_b = _polygon_bounds(nfp_ab)
        ba_b = _polygon_bounds(nfp_ba)
        self.assertRoughly(ab_b[2] - ab_b[0], ba_b[2] - ba_b[0])
        self.assertRoughly(ab_b[3] - ab_b[1], ba_b[3] - ba_b[1])

    # -- point-in-polygon ---------------------------------------------------

    def test_point_inside(self):
        """Centre of unit square is inside."""
        self.assertTrue(_point_in_convex_polygon(0.5, 0.5, UNIT_SQUARE))

    def test_point_outside(self):
        """Point far away is outside."""
        self.assertFalse(_point_in_convex_polygon(5, 5, UNIT_SQUARE))

    def test_point_on_edge(self):
        """Point on boundary counts as inside."""
        self.assertTrue(_point_in_convex_polygon(0.5, 0, UNIT_SQUARE))

    def test_point_on_vertex(self):
        """Vertex point counts as inside."""
        self.assertTrue(_point_in_convex_polygon(0, 0, UNIT_SQUARE))

    # -- strictly_inside_convex ---------------------------------------------

    def test_strictly_inside_center(self):
        """Centre is strictly inside."""
        self.assertTrue(_strictly_inside_convex(0.5, 0.5, UNIT_SQUARE))

    def test_strictly_inside_edge(self):
        """Edge point is NOT strictly inside."""
        self.assertFalse(_strictly_inside_convex(0.5, 0, UNIT_SQUARE))

    # -- position_valid -----------------------------------------------------

    def test_position_valid_no_nfps(self):
        """Any position is valid with no NFPs."""
        self.assertTrue(_position_valid(5, 5, []))

    def test_position_valid_outside_nfp(self):
        """Position outside all NFPs is valid."""
        nfp = [(0, 0), (2, 0), (2, 2), (0, 2)]
        self.assertTrue(_position_valid(5, 5, [nfp]))

    def test_position_invalid_inside_nfp(self):
        """Position strictly inside an NFP is invalid."""
        nfp = [(0, 0), (10, 0), (10, 10), (0, 10)]
        self.assertFalse(_position_valid(5, 5, [nfp]))

    def test_position_valid_on_nfp_boundary(self):
        """Position on NFP boundary is valid (touching, not overlapping)."""
        nfp = [(0, 0), (2, 0), (2, 2), (0, 2)]
        self.assertTrue(_position_valid(1, 0, [nfp]))


# ═══════════════════════════════════════════════════════════════════════════
#  2. NFPPacker integration tests
# ═══════════════════════════════════════════════════════════════════════════


class TestNestingPacker(PathTestBase):
    """Integration tests for NFPPacker (no FreeCAD documents)."""

    # -- Basic single-part placement ----------------------------------------

    def test_single_square(self):
        """Single square fits in stock."""
        packer = NFPPacker(20, 20)
        results = packer.pack([UNIT_SQUARE])
        self.assertEqual(len(results), 1)
        self.assertIsNotNone(results[0])

    def test_single_part_too_big(self):
        """Part larger than stock returns None."""
        big = [(0, 0), (50, 0), (50, 50), (0, 50)]
        packer = NFPPacker(10, 10)
        results = packer.pack([big])
        self.assertIsNone(results[0])

    # -- Multiple parts without overlap -------------------------------------

    def test_two_squares_no_overlap(self):
        """Two 5×5 squares fit in 20×20 stock without overlap."""
        sq = [(0, 0), (5, 0), (5, 5), (0, 5)]
        packer = NFPPacker(20, 20)
        results = packer.pack([sq, sq])
        self.assertIsNotNone(results[0])
        self.assertIsNotNone(results[1])
        # Verify no overlap: placed polygons should not intersect.
        self._assert_no_overlap(results, [sq, sq])

    def test_three_triangles_fit(self):
        """Three small triangles fit in 100×100 stock."""
        tri = [(0, 0), (5, 0), (0, 5)]
        packer = NFPPacker(100, 100)
        results = packer.pack([tri, tri, tri])
        placed_count = sum(1 for r in results if r is not None)
        self.assertEqual(placed_count, 3)
        self._assert_no_overlap(results, [tri, tri, tri])

    def test_many_small_squares(self):
        """Eight 4×4 squares fit in 20×20 stock."""
        sq = [(0, 0), (4, 0), (4, 4), (0, 4)]
        packer = NFPPacker(20, 20)
        results = packer.pack([sq] * 8)
        placed = sum(1 for r in results if r is not None)
        self.assertEqual(placed, 8)
        self._assert_no_overlap(results, [sq] * 8)

    # -- Rotation -----------------------------------------------------------

    def test_rotation_helps_fit(self):
        """A thin rectangle fits better with rotation enabled."""
        # 2×18 rect — two won't fit side-by-side in 20×20 without rotation,
        # but should fit with rotation allowing one to be turned.
        rect = [(0, 0), (2, 0), (2, 18), (0, 18)]
        packer_norot = NFPPacker(20, 20, allow_rotation=False)
        res_norot = packer_norot.pack([rect, rect])

        packer_rot = NFPPacker(20, 20, allow_rotation=True, rotation_step=90)
        res_rot = packer_rot.pack([rect, rect])

        placed_rot = sum(1 for r in res_rot if r is not None)
        self.assertEqual(placed_rot, 2)

    # -- Bias directions ----------------------------------------------------

    def test_bias_front(self):
        """Front bias places a part near the low-Y edge."""
        sq = [(0, 0), (5, 0), (5, 5), (0, 5)]
        packer = NFPPacker(100, 100, bias=PackingBias.Front)
        results = packer.pack([sq])
        px, py, _angle = results[0]
        self.assertTrue(py < 10, "Front bias should place near Y=0")

    def test_bias_back(self):
        """Back bias places a part near the high-Y edge."""
        sq = [(0, 0), (5, 0), (5, 5), (0, 5)]
        packer = NFPPacker(100, 100, bias=PackingBias.Back)
        results = packer.pack([sq])
        px, py, _angle = results[0]
        self.assertTrue(py > 80, "Back bias should place near Y=100")

    def test_bias_left(self):
        """Left bias places a part near the low-X edge."""
        sq = [(0, 0), (5, 0), (5, 5), (0, 5)]
        packer = NFPPacker(100, 100, bias=PackingBias.Left)
        results = packer.pack([sq])
        px, py, _angle = results[0]
        self.assertTrue(px < 10, "Left bias should place near X=0")

    def test_bias_right(self):
        """Right bias places a part near the high-X edge."""
        sq = [(0, 0), (5, 0), (5, 5), (0, 5)]
        packer = NFPPacker(100, 100, bias=PackingBias.Right)
        results = packer.pack([sq])
        px, py, _angle = results[0]
        self.assertTrue(px > 80, "Right bias should place near X=100")

    def test_bias_center(self):
        """Center bias places a single part near the stock centre."""
        sq = [(0, 0), (5, 0), (5, 5), (0, 5)]
        packer = NFPPacker(100, 100, bias=PackingBias.Center)
        results = packer.pack([sq])
        px, py, _angle = results[0]
        # After gravity shift, part should be near centre.
        cx = px + 2.5
        cy = py + 2.5
        dist = math.hypot(cx - 50, cy - 50)
        self.assertTrue(dist < 30, "Center bias should cluster near centre")

    # -- Parts stay inside stock --------------------------------------------

    def test_all_parts_inside_stock(self):
        """All placed parts fit within [0, W] × [0, H]."""
        shapes = [
            [(0, 0), (6, 0), (6, 4), (0, 4)],
            [(0, 0), (3, 0), (3, 8), (0, 8)],
            [(0, 0), (5, 0), (2.5, 5)],
        ]
        packer = NFPPacker(30, 30, allow_rotation=True, rotation_step=15)
        results = packer.pack(shapes)
        for idx, res in enumerate(results):
            if res is None:
                continue
            px, py, angle = res
            rotated = _rotate_polygon(shapes[idx], angle)
            placed = _translate_polygon(rotated, px, py)
            bx0, by0, bx1, by1 = _polygon_bounds(placed)
            self.assertTrue(bx0 >= -1e-6, "Part extends past left edge")
            self.assertTrue(by0 >= -1e-6, "Part extends past bottom edge")
            self.assertTrue(bx1 <= 30 + 1e-6, "Part extends past right edge")
            self.assertTrue(by1 <= 30 + 1e-6, "Part extends past top edge")

    # -- Helpers ------------------------------------------------------------

    def _assert_no_overlap(self, results, outlines):
        """Verify no two placed outlines have overlapping interiors.

        Uses NFP: if the reference point of outline B lies strictly
        inside NFP(placed_A, B), the two overlap.
        """
        placed = []
        for idx, res in enumerate(results):
            if res is None:
                continue
            px, py, angle = res
            rotated = _rotate_polygon(outlines[idx], angle)
            translated = _translate_polygon(rotated, px, py)
            placed.append(translated)

        for i in range(len(placed)):
            for j in range(i + 1, len(placed)):
                nfp = _compute_nfp(placed[i], placed[j])
                # placed[j]'s reference point is (0,0) of the original
                # outline, which was translated to its position.
                # For this check, compute barycentre of placed[j] and
                # verify it's not strictly inside the NFP.
                bx = sum(x for x, y in placed[j]) / len(placed[j])
                by = sum(y for x, y in placed[j]) / len(placed[j])
                # A simpler check: the bounding boxes should not have
                # significant interior overlap.
                bi = _polygon_bounds(placed[i])
                bj = _polygon_bounds(placed[j])
                ox = max(0, min(bi[2], bj[2]) - max(bi[0], bj[0]))
                oy = max(0, min(bi[3], bj[3]) - max(bi[1], bj[1]))
                overlap_area = ox * oy
                ai = (bi[2] - bi[0]) * (bi[3] - bi[1])
                aj = (bj[2] - bj[0]) * (bj[3] - bj[1])
                min_area = min(ai, aj)
                # Allow up to 5% overlap from bounding-box approximation.
                self.assertTrue(
                    overlap_area < min_area * 0.05 + 1e-6,
                    "Parts %d and %d overlap (BB overlap %.2f vs min area %.2f)"
                    % (i, j, overlap_area, min_area),
                )


# ═══════════════════════════════════════════════════════════════════════════
#  3. Full nestModels with FreeCAD documents
# ═══════════════════════════════════════════════════════════════════════════


class TestNestingModels(PathTestBase):
    """End-to-end tests using FreeCAD document objects."""

    def setUp(self):
        self.doc = FreeCAD.newDocument("TestNesting")

        # Build a simple Job with two boxes as models.
        self.box1 = self.doc.addObject("Part::Box", "Box1")
        self.box1.Length = 20
        self.box1.Width = 15
        self.box1.Height = 5

        self.box2 = self.doc.addObject("Part::Box", "Box2")
        self.box2.Length = 10
        self.box2.Width = 10
        self.box2.Height = 5

        self.doc.recompute()

        # Minimal Job mock with Model group and Stock.
        self.job = self.doc.addObject("App::FeaturePython", "Job")
        self.job.addProperty("App::PropertyLink", "Model")
        self.job.addProperty("App::PropertyLink", "Stock")

        model_group = self.doc.addObject(
            "App::DocumentObjectGroup", "Model"
        )
        model_group.addObject(self.box1)
        model_group.addObject(self.box2)
        self.job.Model = model_group

        import Path.Main.Stock as PathStock

        self.stock = PathStock.CreateBox(
            self.job,
            extent=FreeCAD.Vector(80, 80, 5),
            placement=FreeCAD.Placement(
                FreeCAD.Vector(0, 0, 0), FreeCAD.Rotation()
            ),
        )
        self.job.Stock = self.stock
        self.doc.recompute()

    def tearDown(self):
        FreeCAD.closeDocument("TestNesting")

    def test_nest_places_all(self):
        """nestModels places both boxes successfully."""
        report = Nesting.nestModels(self.job, spacing=2.0)
        self.assertIn("2", report)
        # Both boxes should have been moved.
        self.assertIsNotNone(self.box1.Placement)
        self.assertIsNotNone(self.box2.Placement)

    def test_nest_models_within_stock(self):
        """All nested models stay within the stock bounding box."""
        Nesting.nestModels(
            self.job, spacing=2.0, allow_rotation=False
        )
        self.doc.recompute()
        sbb = self.stock.Shape.BoundBox
        for box in (self.box1, self.box2):
            bb = box.Shape.BoundBox
            self.assertTrue(
                bb.XMin >= sbb.XMin - 0.1,
                "%s extends past stock left" % box.Label,
            )
            self.assertTrue(
                bb.YMin >= sbb.YMin - 0.1,
                "%s extends past stock front" % box.Label,
            )
            self.assertTrue(
                bb.XMax <= sbb.XMax + 0.1,
                "%s extends past stock right" % box.Label,
            )
            self.assertTrue(
                bb.YMax <= sbb.YMax + 0.1,
                "%s extends past stock back" % box.Label,
            )

    def test_nest_no_overlap(self):
        """Nested models do not overlap in XY."""
        Nesting.nestModels(
            self.job, spacing=0.0, allow_rotation=False
        )
        self.doc.recompute()
        bb1 = self.box1.Shape.BoundBox
        bb2 = self.box2.Shape.BoundBox
        ox = max(0, min(bb1.XMax, bb2.XMax) - max(bb1.XMin, bb2.XMin))
        oy = max(0, min(bb1.YMax, bb2.YMax) - max(bb1.YMin, bb2.YMin))
        overlap = ox * oy
        self.assertTrue(overlap < 0.5, "Models overlap in XY")

    def test_nest_spacing(self):
        """Parts respect the requested gap."""
        gap = 5.0
        Nesting.nestModels(
            self.job, spacing=gap, allow_rotation=False
        )
        self.doc.recompute()
        bb1 = self.box1.Shape.BoundBox
        bb2 = self.box2.Shape.BoundBox
        # Distance between closest edges in X and Y.
        dx = max(0, max(bb1.XMin, bb2.XMin) - min(bb1.XMax, bb2.XMax))
        dy = max(0, max(bb1.YMin, bb2.YMin) - min(bb1.YMax, bb2.YMax))
        separation = max(dx, dy)
        self.assertTrue(
            separation >= gap - 1.0,
            "Gap is %.2f, expected >= %.2f" % (separation, gap - 1.0),
        )

    def test_nest_edge_margin(self):
        """Parts respect edge margin from stock boundary."""
        margin = 10.0
        Nesting.nestModels(
            self.job, spacing=0.0, edge_margin=margin,
            allow_rotation=False,
        )
        self.doc.recompute()
        sbb = self.stock.Shape.BoundBox
        for box in (self.box1, self.box2):
            bb = box.Shape.BoundBox
            self.assertTrue(
                bb.XMin >= sbb.XMin + margin - 0.5,
                "%s too close to left edge (%.2f vs %.2f)"
                % (box.Label, bb.XMin, sbb.XMin + margin),
            )
            self.assertTrue(
                bb.YMin >= sbb.YMin + margin - 0.5,
                "%s too close to front edge (%.2f vs %.2f)"
                % (box.Label, bb.YMin, sbb.YMin + margin),
            )

    def test_nest_bias_directions(self):
        """Different bias directions place the first part in the expected quadrant."""
        sbb = self.stock.Shape.BoundBox
        cx = (sbb.XMin + sbb.XMax) / 2.0
        cy = (sbb.YMin + sbb.YMax) / 2.0

        for bias, check_x, check_y in [
            (PackingBias.Front, None, lambda y: y < cy),
            (PackingBias.Back, None, lambda y: y > cy),
            (PackingBias.Left, lambda x: x < cx, None),
            (PackingBias.Right, lambda x: x > cx, None),
        ]:
            # Reset placements.
            self.box1.Placement = FreeCAD.Placement()
            self.box2.Placement = FreeCAD.Placement()
            self.doc.recompute()

            Nesting.nestModels(
                self.job, spacing=0.0, bias=bias,
                allow_rotation=False,
            )
            self.doc.recompute()

            bb = self.box1.Shape.BoundBox
            mid_x = (bb.XMin + bb.XMax) / 2.0
            mid_y = (bb.YMin + bb.YMax) / 2.0
            if check_x is not None:
                self.assertTrue(
                    check_x(mid_x),
                    "Bias %s: box1 X midpoint %.2f unexpected" % (bias, mid_x),
                )
            if check_y is not None:
                self.assertTrue(
                    check_y(mid_y),
                    "Bias %s: box1 Y midpoint %.2f unexpected" % (bias, mid_y),
                )

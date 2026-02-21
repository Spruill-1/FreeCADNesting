# SPDX-License-Identifier: LGPL-2.1-or-later

# ***************************************************************************
# *   Copyright (c) 2017 sliptonic <shopinthewoods@gmail.com>               *
# *   Copyright (c) 2026 davidgilkaufman (David Kaufman)                    *
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
"""Reusable holding-tab (tag) utilities for CAM operations.

This module extracts the core tag geometry (``Tag``), edge-splitting
engine (``MapWireToTag``), and auto-distribution helpers from the original
``Path.Dressup.Tags`` module so they can be reused directly from profile
(and potentially other) operations without requiring a dressup object.

Key improvements over the original dressup-only code:

* **Pass-Z awareness** – ``processTags`` segments the edge list by depth
  pass and supplies the correct climb-over Z to ``MapWireToTag`` for each
  pass, so intermediate passes in the tab zone are properly modified.
* **Standalone API** – ``applyTagsToPath`` accepts a ``Path.Path``, tag
  parameters, and depth information and returns a modified ``Path.Path``.
  No FreeCAD document object or dressup proxy is required.
"""

import copy
import math

import FreeCAD
import Path

# lazily loaded modules
from lazy_loader.lazy_loader import LazyLoader

Part = LazyLoader("Part", globals(), "Part")

__title__ = "CAM Tag Utilities"
__author__ = "sliptonic (Brad Collette), davidgilkaufman (David Kaufman)"
__url__ = "https://www.freecad.org"
__doc__ = "Reusable holding-tab (tag) geometry and path-modification helpers."

if False:
    Path.Log.setLevel(Path.Log.Level.DEBUG, Path.Log.thisModule())
    Path.Log.trackModule(Path.Log.thisModule())
else:
    Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())


# ---------------------------------------------------------------------------
# Tag solid geometry
# ---------------------------------------------------------------------------

class Tag:
    """A single holding-tab solid, positioned at (x, y) on the bottom wire.

    The solid is a cylinder (angle == 90), truncated cone / trapezoid
    (0 < angle < 90), or triangle (when height exceeds half-width at the
    given angle).  The ``toolRadius`` pad is added around the base width
    so that the tag shape accounts for the cutter.
    """

    def __init__(self, nr, x, y, width, height, angle, radius, enabled=True):
        self.nr = nr
        self.x = x
        self.y = y
        self.width = math.fabs(width)
        self.height = math.fabs(height)
        self.actualHeight = self.height
        self.angle = math.fabs(angle)
        self.radius = getattr(
            radius, "Value", FreeCAD.Units.Quantity(radius, FreeCAD.Units.Length).Value
        )
        self.enabled = enabled
        self.isSquare = False

        # Populated by createSolidsAt()
        self.toolRadius = None
        self.realRadius = None
        self.r1 = None
        self.r2 = None
        self.solid = None
        self.z = None

    def fullWidth(self):
        return 2 * self.toolRadius + self.width

    def originAt(self, z):
        return FreeCAD.Vector(self.x, self.y, z)

    def bottom(self):
        return self.z

    def top(self):
        return self.z + self.actualHeight

    def createSolidsAt(self, z, R):
        """Build the 3-D tag solid at the given *z* floor with tool radius *R*."""
        self.z = z
        self.toolRadius = R
        r1 = self.fullWidth() / 2
        self.r1 = r1
        self.r2 = r1
        height = self.height * 1.01
        radius = 0

        if Path.Geom.isRoughly(90, self.angle) and height > 0:
            # Rectangular – cylinder
            self.isSquare = True
            self.solid = Part.makeCylinder(r1, height)
            radius = min(min(self.radius, r1), self.height)
            Path.Log.debug("Part.makeCylinder(%f, %f)" % (r1, height))
        elif self.angle > 0.0 and height > 0.0:
            # Trapezoidal – truncated cone
            rad = math.radians(self.angle)
            tangens = math.tan(rad)
            dr = height / tangens
            if dr < r1:
                r2 = r1 - dr
                s = height / math.sin(rad)
                radius = min(r2, s) * math.tan((math.pi - rad) / 2) * 0.95
            else:
                # Degenerates to triangle
                r2 = 0
                height = r1 * tangens * 1.01
                self.actualHeight = height
            self.r2 = r2
            Path.Log.debug("Part.makeCone(%f, %f, %f)" % (r1, r2, height))
            self.solid = Part.makeCone(r1, r2, height)
        else:
            # Degenerate – effectively no tag
            Path.Log.debug("Part.makeSphere(%f / 10000)" % r1)
            self.solid = Part.makeSphere(r1 / 10000)

        if not Path.Geom.isRoughly(0, R):
            angle = -Path.Geom.getAngle(self.originAt(0)) * 180 / math.pi
            Path.Log.debug("solid.rotate(%f)" % angle)
            self.solid.rotate(FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(0, 0, 1), angle)

        orig = self.originAt(z - 0.01 * self.actualHeight)
        Path.Log.debug("solid.translate(%s)" % orig)
        self.solid.translate(orig)

        radius = min(self.radius, radius)
        self.realRadius = radius
        if not Path.Geom.isRoughly(0, radius):
            Path.Log.debug("makeFillet(%.4f)" % radius)
            self.solid = self.solid.makeFillet(radius, [self.solid.Edges[0]])

    def intersects(self, edge, param):
        """Return the nearest intersection point if *edge* enters this tag, else ``None``."""
        def isDefinitelySmaller(z, zRef):
            return z < zRef and not Path.Geom.isRoughly(z, zRef, 0.01)

        if self.enabled:
            zFirst = edge.valueAt(edge.FirstParameter).z
            zLast = edge.valueAt(edge.LastParameter).z
            zMax = self.top()
            if isDefinitelySmaller(zFirst, zMax) or isDefinitelySmaller(zLast, zMax):
                return self._nextIntersectionClosestTo(edge, self.solid, edge.valueAt(param))
        return None

    def _nextIntersectionClosestTo(self, edge, solid, refPt):
        vertexes = edge.common(solid).Vertexes
        if vertexes:
            return sorted(vertexes, key=lambda v: (v.Point - refPt).Length)[0].Point
        return None


# ---------------------------------------------------------------------------
# Edge ↔ tag mapping (wire splitting / climb-over)
# ---------------------------------------------------------------------------

class MapWireToTag:
    """Maps a sequence of path edges through a single tag solid.

    This is the core engine that splits edges at the tag boundary, builds
    a boolean intersection (shell ∩ tag-solid), cleans up the resulting
    edges and emits the replacement G-code commands that climb over or
    around the tag.

    Parameters
    ----------
    edge : Part.Edge
        The first edge that enters the tag.
    tag : Tag
        The tag solid to route over.
    i : FreeCAD.Vector
        The intersection point where the edge meets the tag.
    maxZ : float
        The climb-over ceiling for this pass.  For the bottom pass this is
        typically the global ``maxZ``; for intermediate passes it should be
        the pass's own Z level.
    hSpeed, vSpeed : float
        Horizontal / vertical feed rates for command generation.
    tolerance : float
        Geometry tolerance (from Job.GeometryTolerance).
    """

    def __init__(self, edge, tag, i, maxZ, hSpeed, vSpeed, tolerance):
        self.tag = tag
        self.maxZ = maxZ
        self.hSpeed = hSpeed
        self.vSpeed = vSpeed
        self.tolerance = tolerance

        if Path.Geom.pointsCoincide(edge.valueAt(edge.FirstParameter), i):
            tail = edge
            self.commands = []
        elif Path.Geom.pointsCoincide(edge.valueAt(edge.LastParameter), i):
            self.commands = Path.Geom.cmdsForEdge(
                edge, hSpeed=self.hSpeed, vSpeed=self.vSpeed, tol=tolerance,
            )
            tail = None
        else:
            e, tail = Path.Geom.splitEdgeAt(edge, i)
            self.commands = Path.Geom.cmdsForEdge(
                e, hSpeed=self.hSpeed, vSpeed=self.vSpeed, tol=tolerance,
            )
            self.initialEdge = edge

        self.tail = tail
        self.edges = []
        self.entry = i
        self.complete = False
        self.haveProblem = False

        # Populated later
        self.exit = None
        self.finalEdge = None
        self.offendingEdge = None
        self.realEntry = None
        self.realExit = None
        self.entryEdges = None
        self.exitEdges = None
        self.edgePoints = None
        self.edgesCleanup = None
        self.edgesOrder = None

    # -- edge collection ---------------------------------------------------

    def addEdge(self, edge):
        self.edges.append(edge)

    def add(self, edge):
        self.tail = None
        self.finalEdge = edge
        if self.tag.solid.isInside(edge.valueAt(edge.LastParameter), Path.Geom.Tolerance, True):
            self.addEdge(edge)
        else:
            i = self.tag.intersects(edge, edge.LastParameter)
            if not i:
                self.offendingEdge = edge
                i = edge.valueAt(edge.FirstParameter)
            if Path.Geom.pointsCoincide(i, edge.valueAt(edge.FirstParameter)):
                self.tail = edge
            else:
                e, tail = Path.Geom.splitEdgeAt(edge, i)
                self.addEdge(e)
                self.tail = tail
            self.exit = i
            self.complete = True
            self.commands.extend(self.commandsForEdges())

    def mappingComplete(self):
        return self.complete

    # -- shell / boolean intersection and command generation ----------------

    def shell(self):
        if len(self.edges) > 1:
            wire = Part.Wire(self.initialEdge)
        else:
            edge = self.edges[0]
            if Path.Geom.pointsCoincide(
                edge.valueAt(edge.FirstParameter),
                self.finalEdge.valueAt(self.finalEdge.FirstParameter),
            ):
                wire = Part.Wire(self.finalEdge)
            elif hasattr(self, "initialEdge") and Path.Geom.pointsCoincide(
                edge.valueAt(edge.FirstParameter),
                self.initialEdge.valueAt(self.initialEdge.FirstParameter),
            ):
                wire = Part.Wire(self.initialEdge)
            else:
                wire = Part.Wire(edge)

        for edge in self.edges[1:]:
            if Path.Geom.pointsCoincide(
                edge.valueAt(edge.FirstParameter),
                self.finalEdge.valueAt(self.finalEdge.FirstParameter),
            ):
                wire.add(self.finalEdge)
            else:
                wire.add(edge)

        shell = wire.extrude(FreeCAD.Vector(0, 0, self.tag.height + 1))
        nullFaces = [f for f in shell.Faces if Path.Geom.isRoughly(f.Area, 0)]
        if nullFaces:
            return shell.removeShape(nullFaces)
        return shell

    def commandsForEdges(self):
        if not self.edges:
            return []
        try:
            shape = self.shell().common(self.tag.solid)
            commands = []
            rapid = None
            for e, flip in self.orderAndFlipEdges(self.cleanupEdges(shape.Edges)):
                p1 = e.valueAt(e.FirstParameter)
                p2 = e.valueAt(e.LastParameter)
                if (
                    self.tag.isSquare
                    and (Path.Geom.isRoughly(p1.z, self.maxZ) or p1.z > self.maxZ)
                    and (Path.Geom.isRoughly(p2.z, self.maxZ) or p2.z > self.maxZ)
                ):
                    rapid = p1 if flip else p2
                else:
                    if rapid:
                        commands.append(
                            Path.Command("G0", {"X": rapid.x, "Y": rapid.y, "Z": rapid.z})
                        )
                        rapid = None
                    commands.extend(
                        Path.Geom.cmdsForEdge(
                            e, hSpeed=self.hSpeed, vSpeed=self.vSpeed, tol=self.tolerance,
                        )
                    )
            if rapid:
                commands.append(
                    Path.Command("G0", {"X": rapid.x, "Y": rapid.y, "Z": rapid.z})
                )
            return commands
        except Exception as exc:
            Path.Log.error(
                "Exception during processing tag @(%.2f, %.2f) (%s) - disabling"
                % (self.tag.x, self.tag.y, exc.args[0])
            )
            self.tag.enabled = False
            commands = []
            for e in self.edges:
                commands.extend(
                    Path.Geom.cmdsForEdge(
                        e, hSpeed=self.hSpeed, vSpeed=self.vSpeed, tol=self.tolerance,
                    )
                )
            return commands

    # -- edge cleanup and ordering -----------------------------------------

    def cleanupEdges(self, edges):
        if not edges:
            return edges
        self.edgesCleanup = [copy.copy(edges)]
        self.entryEdges = []
        self.exitEdges = []
        self.edgePoints = []

        for e in copy.copy(edges):
            p1 = e.valueAt(e.FirstParameter)
            p2 = e.valueAt(e.LastParameter)
            self.edgePoints.append(p1)
            self.edgePoints.append(p2)
            if self.tag.solid.isInside(p1, Path.Geom.Tolerance, False) or \
               self.tag.solid.isInside(p2, Path.Geom.Tolerance, False):
                edges.remove(e)
            else:
                if Path.Geom.pointsCoincide(p1, self.entry) or \
                   Path.Geom.pointsCoincide(p2, self.entry):
                    self.entryEdges.append(e)
                if Path.Geom.pointsCoincide(p1, self.exit) or \
                   Path.Geom.pointsCoincide(p2, self.exit):
                    self.exitEdges.append(e)
        self.edgesCleanup.append(copy.copy(edges))

        if not self.entryEdges:
            self.realEntry = sorted(self.edgePoints, key=lambda p: (p - self.entry).Length)[0]
            self.entryEdges = [e for e in edges if Path.Geom.edgeConnectsTo(e, self.realEntry)]
            if not Path.Geom.pointsCoincide(self.entry, self.realEntry):
                edges.append(Part.Edge(Part.LineSegment(self.entry, self.realEntry)))
        else:
            self.realEntry = None

        if not self.exitEdges:
            self.realExit = sorted(self.edgePoints, key=lambda p: (p - self.exit).Length)[0]
            self.exitEdges = [e for e in edges if Path.Geom.edgeConnectsTo(e, self.realExit)]
            if not Path.Geom.pointsCoincide(self.realExit, self.exit):
                edges.append(Part.Edge(Part.LineSegment(self.realExit, self.exit)))
        else:
            self.realExit = None
        self.edgesCleanup.append(copy.copy(edges))

        if len(self.entryEdges) > 1:
            if self.entryEdges[0].BoundBox.ZMax < self.entryEdges[1].BoundBox.ZMax:
                edges.remove(self.entryEdges[0])
            else:
                edges.remove(self.entryEdges[1])
        if len(self.exitEdges) > 1:
            if self.exitEdges[0].BoundBox.ZMax < self.exitEdges[1].BoundBox.ZMax:
                if self.exitEdges[0] in edges:
                    edges.remove(self.exitEdges[0])
            else:
                if self.exitEdges[1] in edges:
                    edges.remove(self.exitEdges[1])

        self.edgesCleanup.append(copy.copy(edges))
        return edges

    def orderAndFlipEdges(self, inputEdges):
        self.edgesOrder = []
        outputEdges = []
        p0 = self.entry
        lastP = p0
        edges = copy.copy(inputEdges)

        while edges:
            for e in copy.copy(edges):
                p1 = e.valueAt(e.FirstParameter)
                p2 = e.valueAt(e.LastParameter)
                if Path.Geom.pointsCoincide(p1, p0):
                    outputEdges.append((e, False))
                    edges.remove(e)
                    lastP = None
                    p0 = p2
                    break
                elif Path.Geom.pointsCoincide(p2, p0):
                    flipped = Path.Geom.flipEdge(e)
                    if flipped is not None:
                        outputEdges.append((flipped, True))
                    else:
                        p0tmp = None
                        cnt = 0
                        for p in reversed(e.discretize(Deflection=0.01)):
                            if p0tmp is not None:
                                outputEdges.append((Part.Edge(Part.LineSegment(p0tmp, p)), True))
                                cnt += 1
                            p0tmp = p
                        Path.Log.info("replaced edge with %d straight segments" % cnt)
                    edges.remove(e)
                    lastP = None
                    p0 = p1
                    break

            if lastP == p0:
                self.edgesOrder.append(outputEdges)
                self.edgesOrder.append(edges)
                raise ValueError("No connection to %s" % p0)
            lastP = p0

        return outputEdges


# ---------------------------------------------------------------------------
# Rapid-edge detection helper
# ---------------------------------------------------------------------------

class _RapidEdges:
    """Identifies rapid (G0) edges in a converted wire."""

    def __init__(self, rapid):
        self.rapid = rapid
        # Build a set of (x0,y0,z0,x1,y1,z1) keys rounded to 6 decimals
        # so isRapid() is O(1) instead of O(len(rapid)).
        self._keys = frozenset(
            self._key(r) for r in rapid
        )

    @staticmethod
    def _key(edge):
        v0 = edge.Vertexes[0]
        v1 = edge.Vertexes[1]
        return (
            round(v0.X, 6), round(v0.Y, 6), round(v0.Z, 6),
            round(v1.X, 6), round(v1.Y, 6), round(v1.Z, 6),
        )

    def isRapid(self, edge):
        if type(edge.Curve) in [Part.Line, Part.LineSegment]:
            if not edge.Vertexes:
                return False
            return self._key(edge) in self._keys
        return False


# ---------------------------------------------------------------------------
# Auto-distribution along a wire
# ---------------------------------------------------------------------------

def distributeTags(wire, count, width, height, angle, toolRadius):
    """Evenly distribute *count* tags along a closed *wire*.

    Returns a list of ``Tag`` objects with solids already built at the
    wire's minimum Z.
    """
    if count < 1 or wire is None:
        return []

    # Find Z floor of the wire
    minZ = min(v.Point.z for e in wire.Edges for v in e.Vertexes)

    tagDistance = wire.Length / count

    # Sort edges by length to start on the longest one
    edgeLengths = [(i, e.Length) for i, e in enumerate(wire.Edges)]
    longestIdx = max(edgeLengths, key=lambda t: t[1])[0]
    shortestLen = min(edgeLengths, key=lambda t: t[1])[1]
    longestLen = edgeLengths[longestIdx][1]

    startEdge = wire.Edges[longestIdx]
    startCount = int(startEdge.Length / tagDistance)
    if (longestLen - shortestLen) > shortestLen:
        startCount = int(startEdge.Length / tagDistance) + 1

    lastTagLength = (startEdge.Length + (startCount - 1) * tagDistance) / 2
    currentLength = startEdge.Length
    minLength = min(2.0 * width, longestLen)

    edgeDict = {longestIdx: startCount}

    def _processEdge(index, edge):
        nonlocal currentLength, lastTagLength
        tagCount = 0
        currentLength += edge.Length
        if edge.Length >= minLength:
            while lastTagLength + tagDistance < currentLength:
                tagCount += 1
                lastTagLength += tagDistance
            if tagCount > 0:
                edgeDict[index] = tagCount

    for i in range(longestIdx + 1, len(wire.Edges)):
        _processEdge(i, wire.Edges[i])
    for i in range(0, longestIdx):
        _processEdge(i, wire.Edges[i])

    tags = []
    nr = 0
    for i, cnt in edgeDict.items():
        edge = wire.Edges[i]
        if cnt > 0:
            distance = (edge.LastParameter - edge.FirstParameter) / cnt
            for j in range(cnt):
                pt = edge.Curve.value((j + 0.5) * distance)
                tag = Tag(nr, pt.x, pt.y, width, height, angle, 0, True)
                tag.createSolidsAt(minZ, toolRadius)
                tags.append(tag)
                nr += 1

    return tags


def _pointToSegDist2D(px, py, ax, ay, bx, by):
    """Squared 2-D distance from point (px,py) to segment (a→b)."""
    dx, dy = bx - ax, by - ay
    lenSq = dx * dx + dy * dy
    if lenSq < 1e-14:
        ex, ey = px - ax, py - ay
        return math.sqrt(ex * ex + ey * ey)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / lenSq))
    cx, cy = ax + t * dx - px, ay + t * dy - py
    return math.sqrt(cx * cx + cy * cy)


def _wireToSegments2D(wire):
    """Discretise a wire into a list of 2-D segment endpoints.

    Each entry is (x1, y1, x2, y2).  Curves are approximated with a
    0.1 mm deflection, which is more than adequate for the proximity
    check we need.
    """
    segs = []
    for edge in wire.Edges:
        pts = edge.discretize(Deflection=0.1)
        for i in range(len(pts) - 1):
            segs.append((pts[i].x, pts[i].y, pts[i + 1].x, pts[i + 1].y))
    return segs


def _minDistToSegments(px, py, segs):
    """Minimum 2-D distance from a point to a list of segments."""
    best = float("inf")
    for ax, ay, bx, by in segs:
        d = _pointToSegDist2D(px, py, ax, ay, bx, by)
        if d < best:
            best = d
    return best


def pruneConflictingTags(tagsByWire, bottomWires, toolRadius):
    """Disable tags that would be destroyed by an adjacent contour's cut.

    When multiple profiles are close together, a tab on the inner-facing
    edge of contour A may sit in the path swept by contour B's cutter.
    This function detects those conflicts and disables the affected tags
    so material isn't wasted on tabs that won't survive.

    Parameters
    ----------
    tagsByWire : list of list of Tag
        Tags grouped by their source contour, one inner list per wire.
        Must correspond 1:1 with *bottomWires*.
    bottomWires : list of Part.Wire
        The closed bottom wires (one per contour).
    toolRadius : float
        Active tool radius.

    Returns
    -------
    list of Tag
        Flat list of all tags (enabled survivors + disabled conflicts).
        The input tag objects are modified in-place.
    """
    if len(bottomWires) < 2:
        # Single contour – nothing to conflict with.
        result = []
        for group in tagsByWire:
            result.extend(group)
        return result

    # Pre-discretise every wire into 2-D segments once, rather than
    # calling the expensive OCC distToShape for each tag.
    wireSegs = [_wireToSegments2D(w) for w in bottomWires]

    pruned = 0
    result = []

    for wireIdx, wireTags in enumerate(tagsByWire):
        otherSegs = [s for i, s in enumerate(wireSegs) if i != wireIdx]

        for tag in wireTags:
            # The tab material extends tag.width/2 from the tag centre.
            # The neighbouring profile cut removes material within
            # toolRadius of the contour (on the cut side).  If the
            # distance from the tag centre to any other contour is less
            # than this threshold, the other contour's cut will destroy
            # (or significantly damage) the tab.
            threshold = tag.width / 2.0 + toolRadius

            conflict = False
            for segs in otherSegs:
                dist = _minDistToSegments(tag.x, tag.y, segs)
                if dist < threshold:
                    conflict = True
                    break

            if conflict:
                tag.enabled = False
                pruned += 1

            result.append(tag)

    if pruned:
        Path.Log.info(
            "Tab pruning: disabled %d tab(s) that conflict with adjacent contours" % pruned
        )

    return result


# ---------------------------------------------------------------------------
# Path utilities
# ---------------------------------------------------------------------------

def _findZLimits(edges, rapidEdges):
    """Return (minZ, maxZ) of non-rapid cutting edges."""
    minZ = float("inf")
    maxZ = float("-inf")
    for e in edges:
        if rapidEdges.isRapid(e):
            continue
        for v in e.Vertexes:
            if v.Point.z < minZ:
                minZ = v.Point.z
            if v.Point.z > maxZ:
                maxZ = v.Point.z
    return (minZ, maxZ)


def _findBottomWires(edges, minZ):
    """Extract all closed wires from edges that lie entirely at *minZ*.

    When a profile operation covers multiple objects, the bottom edges
    form several disjoint closed contours.  ``Part.sortEdges`` groups
    connected edges, and each group that forms a closed wire is returned.

    Returns
    -------
    list of Part.Wire
        Closed wires at *minZ*.  May be empty.
    """
    bottom = [
        e for e in edges
        if Path.Geom.isRoughly(e.Vertexes[0].Point.z, minZ)
        and Path.Geom.isRoughly(e.Vertexes[1].Point.z, minZ)
    ]
    if not bottom:
        return []

    wires = []
    try:
        groups = Part.sortEdges(bottom)
    except Exception:
        groups = [bottom]

    for group in groups:
        try:
            wire = Part.Wire(group)
            if wire.isClosed():
                wires.append(wire)
        except Exception:
            continue
    return wires


def _findBottomWire(edges, minZ):
    """Extract a single closed wire from edges at *minZ*.

    Convenience wrapper around :func:`_findBottomWires` that returns the
    first closed wire found, or ``None``.  Kept for backward
    compatibility and simple single-contour cases.
    """
    wires = _findBottomWires(edges, minZ)
    return wires[0] if wires else None


def _isValidTagStartIntersection(edge, i):
    """Filter out false-positive intersection starts."""
    if Path.Geom.pointsCoincide(i, edge.valueAt(edge.LastParameter)):
        return False
    p1 = edge.valueAt(edge.FirstParameter)
    p2 = edge.valueAt(edge.LastParameter)
    if Path.Geom.pointsCoincide(Path.Geom.xy(p1), Path.Geom.xy(p2)):
        if p1.z < p2.z:
            return False
    return True


# ---------------------------------------------------------------------------
# Pass-Z–aware tag processing
# ---------------------------------------------------------------------------

def _createPath(edges, tags, maxZ, rapidEdges, horizFeed, vertFeed, horizRapid, vertRapid, tol):
    """Walk *edges*, applying *tags* – identical logic to the dressup's
    ``createPath`` but taking explicit parameters instead of an obj proxy.

    *maxZ* is the climb-over ceiling for the current pass.
    """
    commands = []
    lastEdge = 0
    lastTag = 0
    t = 0
    edge = None
    mapper = None

    while edge or lastEdge < len(edges):
        if not edge:
            edge = edges[lastEdge]
            lastEdge += 1

        if mapper:
            mapper.add(edge)
            if mapper.mappingComplete():
                commands.extend(mapper.commands)
                edge = mapper.tail
                mapper = None
            else:
                edge = None

        if edge:
            tIndex = (t + lastTag) % len(tags)
            t += 1
            i = tags[tIndex].intersects(edge, edge.FirstParameter)
            if i and _isValidTagStartIntersection(edge, i):
                mapper = MapWireToTag(
                    edge, tags[tIndex], i, maxZ,
                    hSpeed=horizFeed, vSpeed=vertFeed, tolerance=tol,
                )
                edge = mapper.tail

        if not mapper and t >= len(tags):
            if edge:
                if rapidEdges.isRapid(edge):
                    v = edge.Vertexes[1]
                    if (
                        not commands
                        and Path.Geom.isRoughly(0, v.X)
                        and Path.Geom.isRoughly(0, v.Y)
                        and not Path.Geom.isRoughly(0, v.Z)
                    ):
                        commands.append(Path.Command("G0", {"Z": v.Z, "F": horizRapid}))
                    else:
                        commands.append(
                            Path.Command("G0", {"X": v.X, "Y": v.Y, "Z": v.Z, "F": vertRapid})
                        )
                else:
                    commands.extend(
                        Path.Geom.cmdsForEdge(edge, hSpeed=horizFeed, vSpeed=vertFeed, tol=tol)
                    )
            edge = None
            t = 0

    return commands


def _segmentEdgesByPass(edges, rapidEdges):
    """Split *edges* into per-depth-pass groups.

    A new pass begins whenever we see a rapid (G0) edge that moves
    upward in Z (retract).  Each group is a list of edges plus the
    cutting Z of that pass.

    Returns
    -------
    list of (passZ, list_of_edges)
        *passZ* is the minimum cutting-edge Z in that group.
    """
    segments = []
    current = []
    lastRetract = False

    for e in edges:
        isRapid = rapidEdges.isRapid(e)

        if isRapid:
            p1 = e.Vertexes[0].Point
            p2 = e.Vertexes[1].Point
            goingUp = p2.z > p1.z + 0.01

            if goingUp and current:
                # Retract detected – close current segment
                passZ = _passZForEdges(current, rapidEdges)
                segments.append((passZ, current))
                current = [e]
                lastRetract = True
            else:
                current.append(e)
                lastRetract = False
        else:
            current.append(e)
            lastRetract = False

    if current:
        passZ = _passZForEdges(current, rapidEdges)
        segments.append((passZ, current))

    return segments


def _passZForEdges(edges, rapidEdges):
    """Return the minimum Z of the cutting edges in *edges*."""
    minZ = float("inf")
    for e in edges:
        if rapidEdges.isRapid(e):
            continue
        for v in e.Vertexes:
            if v.Point.z < minZ:
                minZ = v.Point.z
    return minZ


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def applyTagsToPath(path, tags, toolRadius, horizFeed, vertFeed,
                    horizRapid, vertRapid, tol,
                    _wire=None, _rapid=None):
    """Apply holding-tab *tags* to an existing ``Path.Path``.

    This is the primary entry point for the Profile operation.  It
    converts the path to edges, segments by depth pass, and applies
    tag modifications only to passes whose Z falls within the tag
    height zone.  Passes above all tags are left untouched.

    Parameters
    ----------
    path : Path.Path
        The original toolpath (all depth passes).
    tags : list of Tag
        Tags with solids already built (``createSolidsAt`` called).
    toolRadius : float
        Active tool radius (mm).
    horizFeed, vertFeed : float
        Cutting feed rates.
    horizRapid, vertRapid : float
        Rapid feed rates.
    tol : float
        Geometry tolerance.
    _wire, _rapid : Part.Wire, list of Part.Edge, optional
        Pre-computed results from ``Path.Geom.wireForPath(path)``.
        When supplied, the expensive wire conversion is skipped.

    Returns
    -------
    Path.Path
        Modified path with tag climb-overs on affected passes.
    """
    if not tags or not path.Commands:
        return path

    # Filter out disabled tags before any geometry work.
    tags = [t for t in tags if t.enabled]
    if not tags:
        return path

    if _wire is not None and _rapid is not None:
        wire, rapid = _wire, _rapid
    else:
        wire, rapid, _rapid_indexes = Path.Geom.wireForPath(path)
    if not wire:
        return path

    rapidEdges = _RapidEdges(rapid)
    edges = wire.Edges

    (minZ, maxZ) = _findZLimits(edges, rapidEdges)

    # Find the highest tag top
    tagCeiling = max(t.top() for t in tags if t.enabled)

    # Segment the path by depth pass
    segments = _segmentEdgesByPass(edges, rapidEdges)

    allCommands = []

    for passZ, passEdges in segments:
        if passZ >= tagCeiling:
            # This pass is entirely above the tags – emit edges unchanged
            for e in passEdges:
                if rapidEdges.isRapid(e):
                    v = e.Vertexes[1]
                    allCommands.append(
                        Path.Command("G0", {"X": v.X, "Y": v.Y, "Z": v.Z, "F": vertRapid})
                    )
                else:
                    allCommands.extend(
                        Path.Geom.cmdsForEdge(e, hSpeed=horizFeed, vSpeed=vertFeed, tol=tol)
                    )
        else:
            # This pass intersects the tag zone – apply tag processing.
            # The climb-over Z for this pass = maxZ of the full path
            # (so the tool retracts to safe height when going over a tag).
            climbZ = maxZ
            cmds = _createPath(
                passEdges, tags, climbZ, rapidEdges,
                horizFeed, vertFeed, horizRapid, vertRapid, tol,
            )
            allCommands.extend(cmds)

    return Path.Path(allCommands)

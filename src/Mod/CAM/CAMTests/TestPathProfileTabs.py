# SPDX-License-Identifier: LGPL-2.1-or-later

# ***************************************************************************
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

"""Tests for Path.Op.TagUtils – tag geometry, distribution, and path modification."""

import math

import FreeCAD
import Part
import Path
import Path.Op.TagUtils as TagUtils
import CAMTests.PathTestUtils as PathTestUtils

Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())


class TestTagGeometry(PathTestUtils.PathTestBase):
    """Test Tag solid creation at various angles."""

    def test_rectangular_tag_90_degrees(self):
        """90° angle produces a cylinder (rectangular cross-section)."""
        tag = TagUtils.Tag(0, 10, 10, 4.0, 2.0, 90, 0)
        tag.createSolidsAt(0, 0)
        self.assertTrue(tag.isSquare)
        self.assertIsNotNone(tag.solid)
        # Bounding box height should be close to 2.0 * 1.01
        self.assertAlmostEqual(tag.solid.BoundBox.ZLength, 2.0 * 1.01, places=1)

    def test_trapezoidal_tag_45_degrees(self):
        """45° angle produces a truncated cone (trapezoidal cross-section)."""
        tag = TagUtils.Tag(0, 10, 10, 6.0, 2.0, 45, 0)
        tag.createSolidsAt(0, 0)
        self.assertFalse(tag.isSquare)
        self.assertIsNotNone(tag.solid)
        # At 45°, dr = height, so r2 = r1 - height. r1 = width/2 = 3, dr = 2, r2 = 1
        self.assertAlmostEqual(tag.r2, 1.0, places=1)

    def test_trapezoidal_tag_60_degrees(self):
        """60° angle produces a gentle trapezoid."""
        tag = TagUtils.Tag(0, 10, 10, 6.0, 2.0, 60, 0)
        tag.createSolidsAt(0, 0)
        self.assertFalse(tag.isSquare)
        # At 60°, dr = height / tan(60) ≈ 2 / 1.732 ≈ 1.155
        # r1 = 3, r2 = 3 - 1.155 ≈ 1.845
        self.assertGreater(tag.r2, 1.0)

    def test_triangular_tag_small_width(self):
        """When height is too tall for the width, tag degenerates to triangle."""
        tag = TagUtils.Tag(0, 10, 10, 2.0, 10.0, 45, 0)
        tag.createSolidsAt(0, 0)
        # r1 = 1.0, dr = height(10) > r1(1), so r2 should be 0
        self.assertAlmostEqual(tag.r2, 0, places=5)

    def test_degenerate_tag_zero_angle(self):
        """0° angle produces a degenerate (negligible) solid."""
        tag = TagUtils.Tag(0, 10, 10, 4.0, 2.0, 0, 0)
        tag.createSolidsAt(0, 0)
        # Should be a tiny sphere
        self.assertLess(tag.solid.BoundBox.DiagonalLength, 0.1)

    def test_tag_bottom_and_top(self):
        """bottom() and top() reflect z floor and actual height."""
        tag = TagUtils.Tag(0, 5, 5, 4.0, 3.0, 45, 0)
        tag.createSolidsAt(-2.0, 0)
        self.assertAlmostEqual(tag.bottom(), -2.0)
        self.assertAlmostEqual(tag.top(), -2.0 + tag.actualHeight, places=2)

    def test_tag_with_tool_radius(self):
        """Tool radius pads the base width."""
        tag = TagUtils.Tag(0, 0, 10, 4.0, 2.0, 90, 0)
        tag.createSolidsAt(0, 1.5)
        # fullWidth = 2*1.5 + 4 = 7, r1 = 3.5
        self.assertAlmostEqual(tag.r1, 3.5, places=2)

    def test_tag_intersects_edge_inside(self):
        """An edge cutting through the tag zone returns an intersection."""
        tag = TagUtils.Tag(0, 10, 0, 4.0, 2.0, 90, 0)
        tag.createSolidsAt(0, 0)
        # Create an edge that passes right through the tag at Z=0
        edge = Part.makeLine(FreeCAD.Vector(5, 0, 0), FreeCAD.Vector(15, 0, 0))
        result = tag.intersects(edge, edge.FirstParameter)
        self.assertIsNotNone(result)

    def test_tag_intersects_edge_above(self):
        """An edge entirely above the tag returns None."""
        tag = TagUtils.Tag(0, 10, 0, 4.0, 2.0, 90, 0)
        tag.createSolidsAt(0, 0)
        # Edge at Z=5 is well above the 2mm tag
        edge = Part.makeLine(FreeCAD.Vector(5, 0, 5), FreeCAD.Vector(15, 0, 5))
        result = tag.intersects(edge, edge.FirstParameter)
        self.assertIsNone(result)

    def test_disabled_tag_no_intersection(self):
        """A disabled tag never reports intersection."""
        tag = TagUtils.Tag(0, 10, 0, 4.0, 2.0, 90, 0, enabled=False)
        tag.createSolidsAt(0, 0)
        edge = Part.makeLine(FreeCAD.Vector(5, 0, 0), FreeCAD.Vector(15, 0, 0))
        result = tag.intersects(edge, edge.FirstParameter)
        self.assertIsNone(result)


class TestTagDistribution(PathTestUtils.PathTestBase):
    """Test distributeTags along closed wires."""

    def _makeSquareWire(self, size=20.0, z=0.0):
        """Create a simple closed square wire at the given Z."""
        p1 = FreeCAD.Vector(0, 0, z)
        p2 = FreeCAD.Vector(size, 0, z)
        p3 = FreeCAD.Vector(size, size, z)
        p4 = FreeCAD.Vector(0, size, z)
        e1 = Part.makeLine(p1, p2)
        e2 = Part.makeLine(p2, p3)
        e3 = Part.makeLine(p3, p4)
        e4 = Part.makeLine(p4, p1)
        return Part.Wire([e1, e2, e3, e4])

    def test_distribute_4_tags(self):
        """Distributing 4 tags on a square yields 4 tags."""
        wire = self._makeSquareWire()
        tags = TagUtils.distributeTags(wire, 4, 3.0, 1.5, 45, 0)
        self.assertEqual(len(tags), 4)

    def test_distribute_zero_count(self):
        """Zero count returns empty list."""
        wire = self._makeSquareWire()
        tags = TagUtils.distributeTags(wire, 0, 3.0, 1.5, 45, 0)
        self.assertEqual(len(tags), 0)

    def test_tags_on_wire(self):
        """All distributed tags lie on or very close to the wire."""
        wire = self._makeSquareWire(20.0, z=-5.0)
        tags = TagUtils.distributeTags(wire, 4, 3.0, 1.5, 45, 0)
        for tag in tags:
            pt = FreeCAD.Vector(tag.x, tag.y, -5.0)
            dist = wire.distToShape(Part.Vertex(pt))[0]
            self.assertLess(dist, 0.5, f"Tag at ({tag.x:.1f},{tag.y:.1f}) too far from wire: {dist}")

    def test_tags_have_solids(self):
        """All tags returned have their solids already built."""
        wire = self._makeSquareWire()
        tags = TagUtils.distributeTags(wire, 4, 3.0, 1.5, 45, 0)
        for tag in tags:
            self.assertIsNotNone(tag.solid)

    def test_tag_dimensions(self):
        """Tags have the requested width, height, angle."""
        wire = self._makeSquareWire()
        tags = TagUtils.distributeTags(wire, 2, 5.0, 2.0, 60, 0)
        for tag in tags:
            self.assertAlmostEqual(tag.width, 5.0)
            self.assertAlmostEqual(tag.height, 2.0)
            self.assertAlmostEqual(tag.angle, 60.0)


class TestRapidEdges(PathTestUtils.PathTestBase):
    """Test _RapidEdges detection helper."""

    def test_identifies_rapid(self):
        """A rapid edge in the list is recognized."""
        e = Part.makeLine(FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(0, 0, 10))
        rapid = TagUtils._RapidEdges([e])
        self.assertTrue(rapid.isRapid(e))

    def test_non_rapid_not_matched(self):
        """An edge not in the rapid list is not flagged."""
        e1 = Part.makeLine(FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(0, 0, 10))
        e2 = Part.makeLine(FreeCAD.Vector(1, 1, 0), FreeCAD.Vector(5, 5, 0))
        rapid = TagUtils._RapidEdges([e1])
        self.assertFalse(rapid.isRapid(e2))


class TestPathModification(PathTestUtils.PathTestBase):
    """Test applyTagsToPath end-to-end on synthetic paths."""

    def _makeSimpleProfilePath(self, z=-5.0):
        """Build a simple square profile path at the given Z.

        The path includes:
        1. G0 Z10 (rapid to clearance)
        2. G0 X0 Y0 Z10 (rapid to start)
        3. G1 Z{z} (plunge)
        4. G1 X20 Y0 (cut edge 1)
        5. G1 X20 Y20 (cut edge 2)
        6. G1 X0 Y20 (cut edge 3)
        7. G1 X0 Y0 (cut edge 4)
        8. G0 Z10 (retract)
        """
        commands = [
            Path.Command("G0", {"Z": 10}),
            Path.Command("G0", {"X": 0, "Y": 0, "Z": 10}),
            Path.Command("G1", {"X": 0, "Y": 0, "Z": z, "F": 100}),
            Path.Command("G1", {"X": 20, "Y": 0, "Z": z, "F": 300}),
            Path.Command("G1", {"X": 20, "Y": 20, "Z": z, "F": 300}),
            Path.Command("G1", {"X": 0, "Y": 20, "Z": z, "F": 300}),
            Path.Command("G1", {"X": 0, "Y": 0, "Z": z, "F": 300}),
            Path.Command("G0", {"Z": 10}),
        ]
        return Path.Path(commands)

    def test_no_tags_returns_same_path(self):
        """With no tags, applyTagsToPath returns the original path."""
        path = self._makeSimpleProfilePath()
        result = TagUtils.applyTagsToPath(path, [], 1.0, 300, 100, 1000, 500, 0.001)
        self.assertEqual(len(result.Commands), len(path.Commands))

    def test_tags_modify_path(self):
        """With tags, the result has more commands than the original (climb-overs)."""
        path = self._makeSimpleProfilePath()
        # Place a tag on the bottom edge
        tag = TagUtils.Tag(0, 10, 0, 3.0, 2.0, 45, 0)
        tag.createSolidsAt(-5.0, 1.0)

        result = TagUtils.applyTagsToPath(path, [tag], 1.0, 300, 100, 1000, 500, 0.001)
        # The modified path should have more commands due to climb-over
        self.assertGreater(len(result.Commands), len(path.Commands))

    def test_empty_path_unchanged(self):
        """An empty path is returned unchanged."""
        path = Path.Path()
        result = TagUtils.applyTagsToPath(path, [], 1.0, 300, 100, 1000, 500, 0.001)
        self.assertEqual(len(result.Commands), 0)


class TestPassSegmentation(PathTestUtils.PathTestBase):
    """Test _segmentEdgesByPass correctly groups edges."""

    def test_single_pass(self):
        """A single-depth path produces one segment."""
        commands = [
            Path.Command("G0", {"Z": 10}),
            Path.Command("G0", {"X": 0, "Y": 0, "Z": 10}),
            Path.Command("G1", {"X": 0, "Y": 0, "Z": -5, "F": 100}),
            Path.Command("G1", {"X": 20, "Y": 0, "Z": -5, "F": 300}),
            Path.Command("G1", {"X": 20, "Y": 20, "Z": -5, "F": 300}),
            Path.Command("G0", {"Z": 10}),
        ]
        path = Path.Path(commands)
        wire, rapid, _ = Path.Geom.wireForPath(path)
        rapidEdges = TagUtils._RapidEdges(rapid)
        segments = TagUtils._segmentEdgesByPass(wire.Edges, rapidEdges)
        # Should have at least 1 segment
        self.assertGreaterEqual(len(segments), 1)

    def test_multi_pass_segmentation(self):
        """A two-depth path produces two segments with correct Z values."""
        commands = [
            Path.Command("G0", {"Z": 10}),
            Path.Command("G0", {"X": 0, "Y": 0, "Z": 10}),
            # Pass 1 at Z=-2
            Path.Command("G1", {"X": 0, "Y": 0, "Z": -2, "F": 100}),
            Path.Command("G1", {"X": 20, "Y": 0, "Z": -2, "F": 300}),
            Path.Command("G1", {"X": 20, "Y": 20, "Z": -2, "F": 300}),
            Path.Command("G1", {"X": 0, "Y": 20, "Z": -2, "F": 300}),
            Path.Command("G1", {"X": 0, "Y": 0, "Z": -2, "F": 300}),
            # Retract
            Path.Command("G0", {"Z": 10}),
            Path.Command("G0", {"X": 0, "Y": 0, "Z": 10}),
            # Pass 2 at Z=-5
            Path.Command("G1", {"X": 0, "Y": 0, "Z": -5, "F": 100}),
            Path.Command("G1", {"X": 20, "Y": 0, "Z": -5, "F": 300}),
            Path.Command("G1", {"X": 20, "Y": 20, "Z": -5, "F": 300}),
            Path.Command("G1", {"X": 0, "Y": 20, "Z": -5, "F": 300}),
            Path.Command("G1", {"X": 0, "Y": 0, "Z": -5, "F": 300}),
            Path.Command("G0", {"Z": 10}),
        ]
        path = Path.Path(commands)
        wire, rapid, _ = Path.Geom.wireForPath(path)
        rapidEdges = TagUtils._RapidEdges(rapid)
        segments = TagUtils._segmentEdgesByPass(wire.Edges, rapidEdges)
        # Should have 2+ segments (initial rapid + pass 1 + pass 2)
        self.assertGreaterEqual(len(segments), 2)
        # Check that the last segment is the deepest
        passZValues = [s[0] for s in segments]
        self.assertAlmostEqual(min(passZValues), -5.0, places=1)


class TestBottomWire(PathTestUtils.PathTestBase):
    """Test _findBottomWire and _findBottomWires extraction."""

    def test_extracts_closed_wire(self):
        """A closed contour at minZ is extracted as a wire."""
        # Build edges: a square at Z=-5 plus a plunge/retract
        p1 = FreeCAD.Vector(0, 0, -5)
        p2 = FreeCAD.Vector(20, 0, -5)
        p3 = FreeCAD.Vector(20, 20, -5)
        p4 = FreeCAD.Vector(0, 20, -5)
        e1 = Part.makeLine(p1, p2)
        e2 = Part.makeLine(p2, p3)
        e3 = Part.makeLine(p3, p4)
        e4 = Part.makeLine(p4, p1)
        # Add a plunge edge at different Z
        plunge = Part.makeLine(FreeCAD.Vector(0, 0, 10), FreeCAD.Vector(0, 0, -5))
        edges = [plunge, e1, e2, e3, e4]
        wire = TagUtils._findBottomWire(edges, -5.0)
        self.assertIsNotNone(wire)
        self.assertTrue(wire.isClosed())

    def test_no_closed_wire_returns_none(self):
        """When edges don't form a closed loop at minZ, returns None."""
        p1 = FreeCAD.Vector(0, 0, -5)
        p2 = FreeCAD.Vector(20, 0, -5)
        e1 = Part.makeLine(p1, p2)
        wire = TagUtils._findBottomWire([e1], -5.0)
        self.assertIsNone(wire)

    def test_multiple_disjoint_contours(self):
        """Two separate closed squares at minZ are both found."""
        # Square 1: (0,0) to (10,10) at Z=-5
        s1 = [
            Part.makeLine(FreeCAD.Vector(0, 0, -5), FreeCAD.Vector(10, 0, -5)),
            Part.makeLine(FreeCAD.Vector(10, 0, -5), FreeCAD.Vector(10, 10, -5)),
            Part.makeLine(FreeCAD.Vector(10, 10, -5), FreeCAD.Vector(0, 10, -5)),
            Part.makeLine(FreeCAD.Vector(0, 10, -5), FreeCAD.Vector(0, 0, -5)),
        ]
        # Square 2: (30,0) to (40,10) at Z=-5
        s2 = [
            Part.makeLine(FreeCAD.Vector(30, 0, -5), FreeCAD.Vector(40, 0, -5)),
            Part.makeLine(FreeCAD.Vector(40, 0, -5), FreeCAD.Vector(40, 10, -5)),
            Part.makeLine(FreeCAD.Vector(40, 10, -5), FreeCAD.Vector(30, 10, -5)),
            Part.makeLine(FreeCAD.Vector(30, 10, -5), FreeCAD.Vector(30, 0, -5)),
        ]
        # Add a plunge edge at different Z
        plunge = Part.makeLine(FreeCAD.Vector(0, 0, 10), FreeCAD.Vector(0, 0, -5))
        edges = [plunge] + s1 + s2
        wires = TagUtils._findBottomWires(edges, -5.0)
        self.assertEqual(len(wires), 2)
        for w in wires:
            self.assertTrue(w.isClosed())

    def test_findBottomWire_returns_first_of_many(self):
        """_findBottomWire still returns a wire even with multiple contours."""
        s1 = [
            Part.makeLine(FreeCAD.Vector(0, 0, -5), FreeCAD.Vector(10, 0, -5)),
            Part.makeLine(FreeCAD.Vector(10, 0, -5), FreeCAD.Vector(10, 10, -5)),
            Part.makeLine(FreeCAD.Vector(10, 10, -5), FreeCAD.Vector(0, 10, -5)),
            Part.makeLine(FreeCAD.Vector(0, 10, -5), FreeCAD.Vector(0, 0, -5)),
        ]
        s2 = [
            Part.makeLine(FreeCAD.Vector(30, 0, -5), FreeCAD.Vector(40, 0, -5)),
            Part.makeLine(FreeCAD.Vector(40, 0, -5), FreeCAD.Vector(40, 10, -5)),
            Part.makeLine(FreeCAD.Vector(40, 10, -5), FreeCAD.Vector(30, 10, -5)),
            Part.makeLine(FreeCAD.Vector(30, 10, -5), FreeCAD.Vector(30, 0, -5)),
        ]
        edges = s1 + s2
        wire = TagUtils._findBottomWire(edges, -5.0)
        self.assertIsNotNone(wire)
        self.assertTrue(wire.isClosed())

    def test_empty_when_no_bottom(self):
        """_findBottomWires returns empty list when no edges at minZ."""
        e = Part.makeLine(FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(10, 0, 0))
        wires = TagUtils._findBottomWires([e], -5.0)
        self.assertEqual(len(wires), 0)


class TestMultiContourTabs(PathTestUtils.PathTestBase):
    """Test tag distribution and path modification with multiple objects."""

    def _makeMultiObjectPath(self, z=-5.0):
        """Build a path with two separate square profiles at the same Z.

        Object 1: (0,0)→(10,0)→(10,10)→(0,10)→(0,0) at z
        Object 2: (30,0)→(40,0)→(40,10)→(30,10)→(30,0) at z
        """
        commands = [
            # Rapid to clearance
            Path.Command("G0", {"Z": 10}),
            # Object 1
            Path.Command("G0", {"X": 0, "Y": 0, "Z": 10}),
            Path.Command("G1", {"X": 0, "Y": 0, "Z": z, "F": 100}),
            Path.Command("G1", {"X": 10, "Y": 0, "Z": z, "F": 300}),
            Path.Command("G1", {"X": 10, "Y": 10, "Z": z, "F": 300}),
            Path.Command("G1", {"X": 0, "Y": 10, "Z": z, "F": 300}),
            Path.Command("G1", {"X": 0, "Y": 0, "Z": z, "F": 300}),
            # Retract
            Path.Command("G0", {"Z": 10}),
            # Object 2
            Path.Command("G0", {"X": 30, "Y": 0, "Z": 10}),
            Path.Command("G1", {"X": 30, "Y": 0, "Z": z, "F": 100}),
            Path.Command("G1", {"X": 40, "Y": 0, "Z": z, "F": 300}),
            Path.Command("G1", {"X": 40, "Y": 10, "Z": z, "F": 300}),
            Path.Command("G1", {"X": 30, "Y": 10, "Z": z, "F": 300}),
            Path.Command("G1", {"X": 30, "Y": 0, "Z": z, "F": 300}),
            # Final retract
            Path.Command("G0", {"Z": 10}),
        ]
        return Path.Path(commands)

    def test_bottom_wires_from_multi_object_path(self):
        """Multi-object path yields multiple closed bottom wires."""
        path = self._makeMultiObjectPath()
        wire, rapid, _ = Path.Geom.wireForPath(path)
        rapidEdges = TagUtils._RapidEdges(rapid)
        edges = wire.Edges
        (minZ, _) = TagUtils._findZLimits(edges, rapidEdges)
        wires = TagUtils._findBottomWires(edges, minZ)
        self.assertEqual(len(wires), 2)

    def test_distribute_tags_across_contours(self):
        """Tags can be distributed across each contour independently."""
        path = self._makeMultiObjectPath()
        wire, rapid, _ = Path.Geom.wireForPath(path)
        rapidEdges = TagUtils._RapidEdges(rapid)
        edges = wire.Edges
        (minZ, _) = TagUtils._findZLimits(edges, rapidEdges)
        bottomWires = TagUtils._findBottomWires(edges, minZ)

        allTags = []
        for bw in bottomWires:
            tags = TagUtils.distributeTags(bw, 2, 3.0, 1.5, 45, 0)
            allTags.extend(tags)
        # 2 contours × 2 tags each = 4 tags
        self.assertEqual(len(allTags), 4)

    def test_apply_tags_to_multi_object_path(self):
        """applyTagsToPath works on paths with multiple objects."""
        path = self._makeMultiObjectPath()
        wire, rapid, _ = Path.Geom.wireForPath(path)
        rapidEdges = TagUtils._RapidEdges(rapid)
        edges = wire.Edges
        (minZ, _) = TagUtils._findZLimits(edges, rapidEdges)
        bottomWires = TagUtils._findBottomWires(edges, minZ)

        allTags = []
        for bw in bottomWires:
            tags = TagUtils.distributeTags(bw, 2, 3.0, 1.5, 45, 0)
            allTags.extend(tags)

        result = TagUtils.applyTagsToPath(
            path, allTags, 1.0, 300, 100, 1000, 500, 0.001
        )
        # The modified path should have more commands due to climb-overs
        self.assertGreater(len(result.Commands), len(path.Commands))


class TestTabPruning(PathTestUtils.PathTestBase):
    """Tests for pruneConflictingTags – proximity-aware tab filtering."""

    @staticmethod
    def _makeSquareWire(cx, cy, size, z=0):
        """Return a closed square wire centred at (cx, cy) on plane z."""
        hs = size / 2.0
        pts = [
            FreeCAD.Vector(cx - hs, cy - hs, z),
            FreeCAD.Vector(cx + hs, cy - hs, z),
            FreeCAD.Vector(cx + hs, cy + hs, z),
            FreeCAD.Vector(cx - hs, cy + hs, z),
        ]
        edges = [Part.makeLine(pts[i], pts[(i + 1) % 4]) for i in range(4)]
        return Part.Wire(edges)

    def test_single_wire_no_pruning(self):
        """A single contour has nothing to conflict with – all tabs stay."""
        wire = self._makeSquareWire(0, 0, 20)
        tags = TagUtils.distributeTags(wire, 4, 3.0, 1.5, 45, 0)
        result = TagUtils.pruneConflictingTags([tags], [wire], 1.0)
        enabled = [t for t in result if t.enabled]
        self.assertEqual(len(enabled), 4)

    def test_distant_wires_no_pruning(self):
        """Two contours far apart – no tabs are pruned."""
        wireA = self._makeSquareWire(0, 0, 20)
        wireB = self._makeSquareWire(100, 0, 20)
        tagsA = TagUtils.distributeTags(wireA, 4, 3.0, 1.5, 45, 0)
        tagsB = TagUtils.distributeTags(wireB, 4, 3.0, 1.5, 45, 0)
        result = TagUtils.pruneConflictingTags(
            [tagsA, tagsB], [wireA, wireB], 1.0
        )
        enabled = [t for t in result if t.enabled]
        self.assertEqual(len(enabled), 8)

    def test_adjacent_wires_prune_facing_tabs(self):
        """Two adjacent contours – tabs on facing edges are disabled."""
        # 20×20 squares with only 2 mm gap between them (edge gap < threshold).
        wireA = self._makeSquareWire(0, 0, 20)
        wireB = self._makeSquareWire(22, 0, 20)
        # Place many tabs so some inevitably fall on the facing edges.
        tagsA = TagUtils.distributeTags(wireA, 8, 3.0, 1.5, 45, 0)
        tagsB = TagUtils.distributeTags(wireB, 8, 3.0, 1.5, 45, 0)
        result = TagUtils.pruneConflictingTags(
            [tagsA, tagsB], [wireA, wireB], 1.5
        )
        disabled = [t for t in result if not t.enabled]
        # At least some tabs on the facing edges should be pruned.
        self.assertGreater(len(disabled), 0)
        # But tabs on the outer edges should survive.
        enabled = [t for t in result if t.enabled]
        self.assertGreater(len(enabled), 0)

    def test_touching_wires_prune_all_facing(self):
        """Two touching squares (gap = 0) – facing-edge tabs are disabled."""
        wireA = self._makeSquareWire(0, 0, 20)
        wireB = self._makeSquareWire(20, 0, 20)  # touching at x = 10
        tagsA = TagUtils.distributeTags(wireA, 4, 3.0, 1.5, 45, 0)
        tagsB = TagUtils.distributeTags(wireB, 4, 3.0, 1.5, 45, 0)
        result = TagUtils.pruneConflictingTags(
            [tagsA, tagsB], [wireA, wireB], 1.0
        )
        disabled = [t for t in result if not t.enabled]
        self.assertGreater(len(disabled), 0)

    def test_pruned_tags_are_still_returned(self):
        """Pruned tags are returned in the result list (just disabled)."""
        wireA = self._makeSquareWire(0, 0, 20)
        wireB = self._makeSquareWire(22, 0, 20)
        tagsA = TagUtils.distributeTags(wireA, 4, 3.0, 1.5, 45, 0)
        tagsB = TagUtils.distributeTags(wireB, 4, 3.0, 1.5, 45, 0)
        result = TagUtils.pruneConflictingTags(
            [tagsA, tagsB], [wireA, wireB], 1.5
        )
        # All tags should be in the result regardless of enabled state.
        self.assertEqual(len(result), 8)

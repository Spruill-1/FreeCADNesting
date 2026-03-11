# SPDX-License-Identifier: LGPL-2.1-or-later

# ***************************************************************************
# *   Copyright (c) 2026                                                    *
# *                                                                         *
# *   This file is part of the FreeCAD CAx development system.              *
# *                                                                         *
# *   This program is free software; you can redistribute it and/or modify  *
# *   it under the terms of the GNU Lesser General Public License (LGPL)    *
# *   as published by the Free Software Foundation; either version 2 of     *
# *   the License, or (at your option) any later version.                   *
# *   for detail see the LICENCE text file.                                 *
# *                                                                         *
# ***************************************************************************

import types

import FreeCAD
import Part
import Path
import Path.Main.Job as PathJob
import Path.Op.PocketShape as PathPocketShape

from CAMTests.PathTestUtils import PathTestBase


def _makeFace(xmin, xmax, ymin, ymax, z):
    p1 = FreeCAD.Vector(xmin, ymin, z)
    p2 = FreeCAD.Vector(xmax, ymin, z)
    p3 = FreeCAD.Vector(xmax, ymax, z)
    p4 = FreeCAD.Vector(xmin, ymax, z)
    return Part.Face(
        Part.Wire(
            [
                Part.makeLine(p1, p2),
                Part.makeLine(p2, p3),
                Part.makeLine(p3, p4),
                Part.makeLine(p4, p1),
            ]
        )
    )


def _topFaceName(shape):
    for index, face in enumerate(shape.Faces, start=1):
        if (
            abs(face.BoundBox.ZMin - shape.BoundBox.ZMax) < 1e-7
            and abs(face.BoundBox.ZMax - shape.BoundBox.ZMax) < 1e-7
            and face.normalAt(0.5, 0.5).z > 0.99
        ):
            return f"Face{index}"
    raise AssertionError("No top face found")


class _FakeExtension:
    def __init__(self, obj, feature, wire, fallbackFace, length=4.0):
        self.obj = obj
        self.feature = feature
        self.sub = "Edge1"
        self.avoid = False
        self.length = types.SimpleNamespace(Value=length)
        self._wire = wire
        self._fallbackFace = fallbackFace

    def getWire(self):
        return self._wire

    def getExtensionFaces(self, extensionWire):
        return [self._fallbackFace]


class TestPathPocketShape(PathTestBase):
    """Unit tests for Pocket Shape specific behavior."""

    def setUp(self):
        self.doc = FreeCAD.newDocument()
        self.body = self.doc.addObject("Part::Feature", "Body")
        self.body.Shape = Part.makeBox(10, 10, 2)
        self.job = PathJob.Create("Job", [self.body], None)

    def tearDown(self):
        FreeCAD.closeDocument(self.doc.Name)

    def test_outline_extensions_use_outline_specific_face(self):
        pocket = PathPocketShape.Create("Pocket")
        top_face = _topFaceName(self.body.Shape)
        pocket.Base = [(self.body, [top_face])]
        pocket.UseOutline = True

        preview_face = _makeFace(10, 11, 0, 10, 2)
        outline_face = _makeFace(10, 14, 0, 10, 2)
        fake_ext = _FakeExtension(self.body, top_face, preview_face.OuterWire, preview_face)

        original_getExtensions = PathPocketShape.FeatureExtensions.getExtensions
        original_getExtendOutlineFace = PathPocketShape.FeatureExtensions.getExtendOutlineFace
        original_combineHorizontalFaces = Path.Geom.combineHorizontalFaces
        original_removeHoles = pocket.Proxy.removeHoles
        try:
            PathPocketShape.FeatureExtensions.getExtensions = lambda obj: [fake_ext]
            PathPocketShape.FeatureExtensions.getExtendOutlineFace = (
                lambda base_shape, face, extension, remHoles=False, offset_tolerance=1e-4: outline_face
            )
            Path.Geom.combineHorizontalFaces = lambda faces: list(faces)
            pocket.Proxy.removeHoles = lambda base, face: face

            removal_shapes = pocket.Proxy.areaOpShapes(pocket)
        finally:
            PathPocketShape.FeatureExtensions.getExtensions = original_getExtensions
            PathPocketShape.FeatureExtensions.getExtendOutlineFace = original_getExtendOutlineFace
            Path.Geom.combineHorizontalFaces = original_combineHorizontalFaces
            pocket.Proxy.removeHoles = original_removeHoles

        self.assertTrue(removal_shapes)
        self.assertTrue(hasattr(pocket, "removalshape"))
        self.assertGreater(pocket.removalshape.BoundBox.XMax, 13.5)
        self.assertLess(pocket.removalshape.BoundBox.XMax, 14.5)
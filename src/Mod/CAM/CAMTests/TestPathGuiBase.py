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
import FreeCADGui
import Part

from CAMTests.PathTestUtils import PathTestBase

try:
    if not hasattr(FreeCADGui, "addCommand"):
        FreeCADGui.addCommand = lambda *args, **kwargs: None
    import Path.Op.Gui.Base as PathOpGuiBase
except Exception:
    PathOpGuiBase = None


def _box(name, x, y, z, dx, dy, dz):
    return types.SimpleNamespace(
        Name=name,
        Label=name,
        Shape=Part.makeBox(dx, dy, dz, FreeCAD.Vector(x, y, z), FreeCAD.Vector(0, 0, 1)),
    )


def _faceAtZ(obj, zValue):
    for index, face in enumerate(obj.Shape.Faces, start=1):
        if abs(face.BoundBox.ZMax - zValue) < 1e-7 and face.normalAt(0.5, 0.5).z > 0.99:
            return f"Face{index}"
    raise AssertionError(f"No upward face found at Z={zValue}")


class TestPathGuiBase(PathTestBase):
    """Regression tests for base-geometry GUI helpers."""

    def test_coplanar_face_expansion_respects_stock(self):
        if PathOpGuiBase is None:
            self.skipTest("GUI base module is not available in this test environment")

        stock = types.SimpleNamespace(Shape=Part.makeBox(20, 20, 10))
        inside_a = _box("InsideA", 0, 0, 0, 4, 4, 5)
        inside_b = _box("InsideB", 8, 0, 0, 4, 4, 5)
        outside = _box("Outside", 30, 0, 0, 4, 4, 5)
        different_plane = _box("Higher", 0, 8, 0, 4, 4, 7)

        subname = _faceAtZ(inside_a, 5.0)
        selected_face = inside_a.Shape.getElement(subname)
        selection = [
            types.SimpleNamespace(
                Object=inside_a,
                HasSubObjects=True,
                SubElementNames=[subname],
                SubObjects=[selected_face],
            )
        ]

        job = types.SimpleNamespace(
            Stock=stock,
            Model=types.SimpleNamespace(Group=[inside_a, inside_b, outside, different_plane]),
        )

        entries = PathOpGuiBase._jobCoplanarFaceCandidates(job, selection, 0.001)
        names = {(obj.Name, sub) for obj, sub in entries}

        self.assertIn((inside_a.Name, subname), names)
        self.assertIn((inside_b.Name, _faceAtZ(inside_b, 5.0)), names)
        self.assertNotIn((outside.Name, _faceAtZ(outside, 5.0)), names)
        self.assertNotIn((different_plane.Name, _faceAtZ(different_plane, 7.0)), names)

    def test_non_planar_faces_are_ignored(self):
        if PathOpGuiBase is None:
            self.skipTest("GUI base module is not available in this test environment")

        cylinder = types.SimpleNamespace(
            Name="Cylinder",
            Label="Cylinder",
            Shape=Part.makeCylinder(2, 5),
        )
        stock = types.SimpleNamespace(Shape=Part.makeBox(10, 10, 10, FreeCAD.Vector(-5, -5, -1)))
        curved_index = next(
            index
            for index, face in enumerate(cylinder.Shape.Faces, start=1)
            if not isinstance(face.Surface, Part.Plane)
        )
        curved_name = f"Face{curved_index}"
        selection = [
            types.SimpleNamespace(
                Object=cylinder,
                HasSubObjects=True,
                SubElementNames=[curved_name],
                SubObjects=[cylinder.Shape.getElement(curved_name)],
            )
        ]
        job = types.SimpleNamespace(Stock=stock, Model=types.SimpleNamespace(Group=[cylinder]))

        entries = PathOpGuiBase._jobCoplanarFaceCandidates(job, selection, 0.001)
        self.assertEqual([], entries)
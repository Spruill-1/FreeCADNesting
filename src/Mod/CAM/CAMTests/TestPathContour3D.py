# SPDX-License-Identifier: LGPL-2.1-or-later

import unittest

import FreeCAD
import Part
import Path
import Path.Main.Job as PathJob
import Path.Op.Contour3D as PathContour3D
from CAMTests.PathTestUtils import PathTestBase

if FreeCAD.GuiUp:
    import Path.Main.Gui.Job as PathJobGui
    import Path.Op.Gui.Contour3D as PathContour3DGui


class TestPathContour3D(PathTestBase):
    """Initial tests for the 3D Contour operation scaffold."""

    def test_property_setup_metadata_includes_contour_options(self):
        properties = set(PathContour3D.SetupProperties())
        self.assertIn("PassMode", properties)
        self.assertIn("FinishPass", properties)
        self.assertIn("BoundaryCleanupPass", properties)
        self.assertIn("MinBevelTilt", properties)

    def test_horizontal_face_is_not_bevel_candidate(self):
        poly = Part.makePolygon(
            [
                FreeCAD.Vector(0, 0, 0),
                FreeCAD.Vector(10, 0, 0),
                FreeCAD.Vector(10, 10, 0),
                FreeCAD.Vector(0, 10, 0),
                FreeCAD.Vector(0, 0, 0),
            ]
        )
        face = Part.Face(poly)
        self.assertFalse(PathContour3D._is_bevel_candidate_face(face))

    def test_sloped_face_is_bevel_candidate(self):
        poly = Part.makePolygon(
            [
                FreeCAD.Vector(0, 0, 0),
                FreeCAD.Vector(10, 0, 0),
                FreeCAD.Vector(10, 10, 10),
                FreeCAD.Vector(0, 10, 10),
                FreeCAD.Vector(0, 0, 0),
            ]
        )
        face = Part.Face(poly)
        self.assertTrue(PathContour3D._is_bevel_candidate_face(face))

    def test_face_only_rejection(self):
        op = PathContour3D.ObjectContour3D.__new__(PathContour3D.ObjectContour3D)
        self.assertTrue(op.opRejectAddBase(None, None, "Edge1"))
        self.assertFalse(op.opRejectAddBase(None, None, "Face1"))

    def test_contour_defaults_are_bevel_focused(self):
        self.assertTrue(hasattr(self.prototype, "PassMode"))
        self.assertEqual(self.prototype.PassMode, "SlicesOnly")
        self.assertFalse(self.prototype.FinishPass)
        self.assertFalse(self.prototype.SpringPass)
        self.assertTrue(self.prototype.BoundaryCleanupPass)
        self.assertFalse(self.prototype.KeepToolDown)
        self.assertRoughly(self.prototype.MinBevelTilt, 5.0)

        if hasattr(self.prototype, "MinStayDownDistance"):
            self.assertRoughly(self.prototype.MinStayDownDistance.Value, 0.0)

        if PathContour3D.HAS_OCL:
            self.assertEqual(self.prototype.Algorithm, "OCL Adaptive")
            self.assertEqual(self.prototype.LayerMode, "Multi-pass")
            self.assertTrue(self.prototype.BoundaryEnforcement)
            self.assertEqual(self.prototype.CutPattern, "None")
            self.assertRoughly(self.prototype.StepOver, 25.0)

    def test_algorithm_is_constrained_to_face_aware_mode(self):
        # Regression for the Waterline face-selection gap: Contour3D must not
        # expose algorithm choices that ignore selected faces.
        contour = PathContour3D.Create("Contour3D_Algorithm", parentJob=self.job)
        contour.Base = [(self.doc.Fusion, [self.sloped_face_name or "Face3"])]
        contour.Label = "Contour3D_Algorithm"

        if hasattr(contour, "Algorithm"):
            enumerations = dict(contour.Proxy.propertyEnumerations())
            self.assertEqual(enumerations.get("Algorithm"), ["OCL Adaptive"])
            self.assertEqual(contour.Algorithm, "OCL Adaptive")

    def test_pass_mode_syncs_finish_and_spring(self):
        contour = PathContour3D.Create("Contour3D_ModeSync", parentJob=self.job)
        contour.Base = [(self.doc.Fusion, [self.sloped_face_name or "Face3"])]
        contour.Label = "Contour3D_ModeSync"

        contour.PassMode = "SlicesPlusFinish"
        contour.Proxy.opOnChanged(contour, "PassMode")
        self.assertTrue(contour.FinishPass)

        contour.SpringPass = True
        contour.FinishPass = False
        contour.Proxy.opOnChanged(contour, "FinishPass")
        self.assertEqual(contour.PassMode, "SlicesOnly")
        self.assertFalse(contour.SpringPass)

    def test_extract_cut_sections_identifies_deepest_loop(self):
        commands = _sample_contour_commands()
        sections = PathContour3D._extract_cut_sections(commands)
        self.assertEqual(len(sections), 2)
        self.assertRoughly(sections[0]["min_z"], -1.0)
        self.assertRoughly(sections[1]["min_z"], -2.0)
        deepest = PathContour3D._deepest_sections(sections)
        self.assertEqual(len(deepest), 1)
        self.assertRoughly(deepest[0]["min_z"], -2.0)

    def test_boundary_cleanup_commands_follow_selected_face_depth_section(self):
        # Cleanup should follow an exact face/plane intersection, not a generic
        # bounding loop, so the generated points should lie on the known sample
        # face section.
        face = _sample_sloped_face()
        obj = _mock_op_object()
        proxy = _mock_proxy()
        commands = PathContour3D._build_boundary_cleanup_commands([face], 0.0, obj, proxy)

        self.assertTrue(commands)
        rendered = [cmd.toGCode() if hasattr(cmd, "toGCode") else str(cmd.Name) for cmd in commands]
        self.assertTrue(any("X 10.000" in text or "X10.000" in text for text in rendered))
        self.assertTrue(any("Y 0.000" in text or "Y0.000" in text for text in rendered))

    def test_post_process_prefers_exact_boundary_cleanup_commands(self):
        commands = _sample_contour_commands()
        exact_cleanup = [
            Path.Command("G0", {"Z": 5.0}),
            Path.Command("G0", {"X": 42.0, "Y": 24.0}),
            Path.Command("G1", {"Z": -2.0}),
        ]
        processed = PathContour3D._post_process_contour_commands(
            commands,
            add_finish_pass=True,
            add_boundary_cleanup=True,
            add_spring_pass=True,
            boundary_cleanup_commands=exact_cleanup,
        )
        rendered = [cmd.toGCode() if hasattr(cmd, "toGCode") else str(cmd.Name) for cmd in processed]
        self.assertTrue(any("X 42.000" in text or "X42.000" in text for text in rendered))
        self.assertTrue(any("3D Contour boundary cleanup" in text for text in rendered))

    def test_keep_tool_down_replaces_short_retract_transition(self):
        # Short retract/reposition/plunge triplets should collapse into a
        # direct feed link when the motion returns to the same depth.
        commands = _sample_contour_commands()
        exact_cleanup = [
            Path.Command("G0", {"Z": 5.0}),
            Path.Command("G0", {"X": 1.5, "Y": 1.5}),
            Path.Command("G1", {"Z": -2.0, "F": 120.0}),
            Path.Command("G1", {"X": 3.0, "Y": 1.5, "F": 180.0}),
        ]
        processed = PathContour3D._post_process_contour_commands(
            commands,
            add_finish_pass=False,
            add_boundary_cleanup=True,
            add_spring_pass=False,
            boundary_cleanup_commands=exact_cleanup,
            keep_tool_down=True,
            min_stay_down_distance=2.0,
        )

        rendered = [cmd.toGCode() if hasattr(cmd, "toGCode") else str(cmd.Name) for cmd in processed]
        self.assertTrue(any("3D Contour stay-down link" in text for text in rendered))
        self.assertTrue(any(("X 1.500" in text or "X1.500" in text) and ("G1" in text) for text in rendered))

    def test_post_process_adds_finish_cleanup_and_spring_passes(self):
        commands = _sample_contour_commands()
        processed = PathContour3D._post_process_contour_commands(
            commands,
            add_finish_pass=True,
            add_boundary_cleanup=True,
            add_spring_pass=True,
        )
        rendered = [cmd.toGCode() if hasattr(cmd, "toGCode") else str(cmd.Name) for cmd in processed]
        self.assertTrue(any("3D Contour finish pass" in text for text in rendered))
        self.assertTrue(any("3D Contour boundary cleanup" in text for text in rendered))
        self.assertTrue(any("3D Contour spring pass" in text for text in rendered))
        self.assertTrue(len(processed) > len(commands))

    @classmethod
    def setUpClass(cls):
        cls.needsInit = True

    @classmethod
    def initClass(cls):
        cls.needsInit = False
        FreeCAD.ConfigSet("SuppressRecomputeRequiredDialog", "True")
        cls.doc = FreeCAD.open(FreeCAD.getHomePath() + "Mod/CAM/CAMTests/test_adaptive.fcstd")
        FreeCAD.ConfigSet("SuppressRecomputeRequiredDialog", "")

        cls.job = PathJob.Create("Job", [cls.doc.Fusion], None)
        cls.job.GeometryTolerance.Value = 0.001
        if FreeCAD.GuiUp:
            cls.job.ViewObject.Proxy = PathJobGui.ViewProvider(cls.job.ViewObject)

        # Pick a real sloped face from the shipped adaptive test model so the
        # runtime test is resilient if face numbering changes in the fixture.
        cls.sloped_face_name = _first_sloped_face_name(cls.doc.Fusion.Shape)

        cls.prototype = PathContour3D.Create("Contour3D", parentJob=cls.job)
        if cls.sloped_face_name:
            cls.prototype.Base = [(cls.doc.Fusion, [cls.sloped_face_name])]
        else:
            cls.prototype.Base = [(cls.doc.Fusion, ["Face3"])]
        cls.prototype.Label = "Prototype"
        _addViewProvider(cls.prototype)
        cls.doc.recompute()

    @classmethod
    def tearDownClass(cls):
        if not cls.needsInit:
            FreeCAD.closeDocument(cls.doc.Name)

    def setUp(self):
        if self.needsInit:
            self.initClass()

    @unittest.skipUnless(PathContour3D.HAS_OCL, "OpenCamLib not available")
    def test_face_generates_path(self):
        if not self.sloped_face_name:
            self.skipTest("No sloped face found in test_adaptive model")
        # End-to-end regression: the operation must produce actual cutting
        # motion, not just a placeholder Path object.
        contour = PathContour3D.Create("Contour3D_Test", parentJob=self.job)
        contour.Base = [(self.doc.Fusion, [self.sloped_face_name])]
        contour.Label = "Contour3D_Test"
        contour.Comment = "Verify 3D Contour builds a non-empty path on a sloped face."
        _addViewProvider(contour)

        self.doc.recompute()

        self.assertTrue(contour.Path is not None, "Operation did not create a Path object")
        self.assertTrue(
            len(contour.Path.Commands) > 0,
            "3D Contour operation generated no commands",
        )
        self.assertTrue(
            any(cmd.Name in ("G1", "G01", "G2", "G02", "G3", "G03") for cmd in contour.Path.Commands),
            "3D Contour operation generated no cutting moves",
        )


def _addViewProvider(contour_op):
    if FreeCAD.GuiUp:
        PathOpGui = PathContour3DGui.PathOpGui
        cmdRes = PathContour3DGui.Command.res
        contour_op.ViewObject.Proxy = PathOpGui.ViewProvider(contour_op.ViewObject, cmdRes)


def _sample_contour_commands():
    # Two simple nested loops with different depths.  These commands are used
    # to unit-test post-processing logic without needing a full FreeCAD model.
    return [
        Path.Command("G0", {"Z": 5.0}),
        Path.Command("G0", {"X": 0.0, "Y": 0.0}),
        Path.Command("G1", {"Z": -1.0}),
        Path.Command("G1", {"X": 5.0, "Y": 0.0}),
        Path.Command("G1", {"X": 5.0, "Y": 5.0}),
        Path.Command("G1", {"X": 0.0, "Y": 5.0}),
        Path.Command("G1", {"X": 0.0, "Y": 0.0}),
        Path.Command("G0", {"Z": 3.0}),
        Path.Command("G0", {"X": 1.0, "Y": 1.0}),
        Path.Command("G1", {"Z": -2.0}),
        Path.Command("G1", {"X": 4.0, "Y": 1.0}),
        Path.Command("G1", {"X": 4.0, "Y": 4.0}),
        Path.Command("G1", {"X": 1.0, "Y": 4.0}),
        Path.Command("G1", {"X": 1.0, "Y": 1.0}),
    ]


def _sample_sloped_face():
    poly = Part.makePolygon(
        [
            FreeCAD.Vector(0, 0, 0),
            FreeCAD.Vector(10, 0, 0),
            FreeCAD.Vector(10, 10, 10),
            FreeCAD.Vector(0, 10, 10),
            FreeCAD.Vector(0, 0, 0),
        ]
    )
    return Part.Face(poly)


def _first_sloped_face_name(shape):
    # Scan the fixture shape for any face accepted by the bevel candidate test
    # so the runtime suite does not depend on a hard-coded face index.
    for index, face in enumerate(shape.Faces, start=1):
        if PathContour3D._is_bevel_candidate_face(face):
            return f"Face{index}"
    return None


class _DistanceValue:
    def __init__(self, value):
        self.Value = value


class _MockOpObject:
    def __init__(self):
        self.ClearanceHeight = _DistanceValue(5.0)
        self.SafeHeight = _DistanceValue(3.0)


class _MockProxy:
    def __init__(self):
        self.vertRapid = 300.0
        self.horizRapid = 500.0
        self.vertFeed = 120.0
        self.horizFeed = 180.0


def _mock_op_object():
    return _MockOpObject()


def _mock_proxy():
    return _MockProxy()
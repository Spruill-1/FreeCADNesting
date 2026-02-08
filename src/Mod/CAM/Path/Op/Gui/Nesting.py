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

"""GUI command for the CAM Nesting operation.

Opens a task panel where the user can configure nesting parameters
(gap, rotation, and packing direction bias) before executing the
arrangement.
"""

import FreeCAD
import FreeCADGui
import Path
import Path.Main.Job as PathJob
import Path.Op.Nesting as PathNesting
import PathScripts.PathUtils as PathUtils

from PySide import QtGui
from PySide.QtCore import QT_TRANSLATE_NOOP

__doc__ = """CAM Nesting command"""

translate = FreeCAD.Qt.translate


# ---------------------------------------------------------------------------
#   Helpers
# ---------------------------------------------------------------------------


def _existingBaseObjects(job):
    """Return a set of the original base object Names in the Job's Model."""
    bases = set()
    if not hasattr(job, "Model") or job.Model is None:
        return bases
    for clone in job.Model.Group:
        if hasattr(clone, "Objects") and clone.Objects:
            for src in clone.Objects:
                bases.add(src.Name)
        else:
            bases.add(clone.Name)
    return bases


def _isSolidObject(obj):
    """Return True if *obj* has a non-null Shape with at least one solid."""
    if not hasattr(obj, "Shape"):
        return False
    if obj.Shape.isNull():
        return False
    return len(obj.Shape.Solids) > 0


def _classifySelection(sel, job):
    """Split *sel* into objects already in the Job and new bodies."""
    existing = _existingBaseObjects(job)
    job_objects = []
    new_bodies = []
    for obj in sel:
        if obj.Name in existing:
            job_objects.append(obj)
        elif _isSolidObject(obj):
            new_bodies.append(obj)
    return job_objects, new_bodies


# ---------------------------------------------------------------------------
#   Task Panel
# ---------------------------------------------------------------------------


class NestingTaskPanel:
    """Task panel for configuring and executing nesting.

    On open the panel immediately adds any Mode-B bodies to the Job and
    runs a preview nest so the user sees the arrangement right away.
    The entire session lives inside a single undo transaction so that
    Cancel reverts everything (including added clones).
    """

    def __init__(self, job, new_bodies=None):
        self.job = job
        self.new_bodies = new_bodies or []
        self._new_clone_map = {}  # original Name -> clone Name
        self._orig_placements = {}  # Name -> FreeCAD.Placement
        self.form = self._buildUI()

    # ----- UI construction -------------------------------------------------

    def _buildUI(self):
        widget = QtGui.QWidget()
        layout = QtGui.QVBoxLayout(widget)

        # --- Packing parameters -------------------------------------------
        grp_packing = QtGui.QGroupBox(translate("CAM_Nesting", "Packing"))
        form_pack = QtGui.QFormLayout(grp_packing)

        self.spinGap = QtGui.QDoubleSpinBox()
        self.spinGap.setRange(0.0, 1000.0)
        self.spinGap.setValue(2.0)
        self.spinGap.setSuffix(" mm")
        self.spinGap.setDecimals(2)
        self.spinGap.setToolTip(
            translate(
                "CAM_Nesting",
                "Minimum gap between parts (accounts for both spacing "
                "and cutter kerf)",
            )
        )
        form_pack.addRow(translate("CAM_Nesting", "Gap:"), self.spinGap)
        self.spinGap.valueChanged.connect(self._onSettingChanged)

        layout.addWidget(grp_packing)

        # --- Rotation parameters ------------------------------------------
        grp_rot = QtGui.QGroupBox(translate("CAM_Nesting", "Rotation"))
        form_rot = QtGui.QFormLayout(grp_rot)

        self.chkRotation = QtGui.QCheckBox()
        self.chkRotation.setChecked(True)
        self.chkRotation.setToolTip(
            translate(
                "CAM_Nesting",
                "Allow the packer to try rotated orientations",
            )
        )
        form_rot.addRow(
            translate("CAM_Nesting", "Allow rotation:"), self.chkRotation
        )

        self.spinRotStep = QtGui.QDoubleSpinBox()
        self.spinRotStep.setRange(1.0, 180.0)
        self.spinRotStep.setValue(90.0)
        self.spinRotStep.setSuffix("\u00b0")
        self.spinRotStep.setDecimals(1)
        self.spinRotStep.setToolTip(
            translate(
                "CAM_Nesting",
                "Rotation increment in degrees. "
                "Smaller values try more orientations (slower).",
            )
        )
        form_rot.addRow(
            translate("CAM_Nesting", "Rotation step:"), self.spinRotStep
        )

        self.chkRotation.toggled.connect(self.spinRotStep.setEnabled)
        self.chkRotation.toggled.connect(self._onSettingChanged)
        self.spinRotStep.valueChanged.connect(self._onSettingChanged)

        layout.addWidget(grp_rot)

        # --- Packing bias (directional gravity) ---------------------------
        grp_bias = QtGui.QGroupBox(
            translate("CAM_Nesting", "Packing Direction")
        )
        form_bias = QtGui.QFormLayout(grp_bias)

        self.comboBias = QtGui.QComboBox()
        for label in PathNesting.PackingBias.LABELS:
            self.comboBias.addItem(translate("CAM_Nesting", label))
        self.comboBias.setCurrentIndex(0)  # BottomLeft
        self.comboBias.setToolTip(
            translate(
                "CAM_Nesting",
                "Direction towards which parts are attracted "
                "during packing",
            )
        )
        form_bias.addRow(
            translate("CAM_Nesting", "Bias:"), self.comboBias
        )

        # Custom gravity point (X, Y) -- only enabled for "Custom Point".
        self.spinGravityX = QtGui.QDoubleSpinBox()
        self.spinGravityX.setRange(-10000.0, 10000.0)
        self.spinGravityX.setValue(0.0)
        self.spinGravityX.setSuffix(" mm")
        self.spinGravityX.setDecimals(2)
        self.spinGravityX.setToolTip(
            translate(
                "CAM_Nesting",
                "X coordinate of the custom gravity point "
                "(relative to the stock origin)",
            )
        )
        form_bias.addRow(
            translate("CAM_Nesting", "Point X:"), self.spinGravityX
        )

        self.spinGravityY = QtGui.QDoubleSpinBox()
        self.spinGravityY.setRange(-10000.0, 10000.0)
        self.spinGravityY.setValue(0.0)
        self.spinGravityY.setSuffix(" mm")
        self.spinGravityY.setDecimals(2)
        self.spinGravityY.setToolTip(
            translate(
                "CAM_Nesting",
                "Y coordinate of the custom gravity point "
                "(relative to the stock origin)",
            )
        )
        form_bias.addRow(
            translate("CAM_Nesting", "Point Y:"), self.spinGravityY
        )

        # Enable/disable custom-point spinners based on combo selection.
        self.comboBias.currentIndexChanged.connect(self._onBiasChanged)
        self.spinGravityX.valueChanged.connect(self._onSettingChanged)
        self.spinGravityY.valueChanged.connect(self._onSettingChanged)
        self.spinGravityX.setEnabled(False)
        self.spinGravityY.setEnabled(False)

        layout.addWidget(grp_bias)

        # --- Update button ------------------------------------------------
        self.btnUpdate = QtGui.QPushButton(
            translate("CAM_Nesting", "Update Preview")
        )
        self.btnUpdate.setToolTip(
            translate(
                "CAM_Nesting",
                "Re-run nesting with the current settings",
            )
        )
        self.btnUpdate.clicked.connect(self._onUpdatePreview)
        layout.addWidget(self.btnUpdate)

        layout.addStretch()
        return widget

    # ----- Preview / nesting helpers --------------------------------------

    def _addNewBodies(self):
        """Add Mode-B bodies as model resource clones (once)."""
        if not self.new_bodies:
            return
        for body in self.new_bodies:
            clone = PathJob.createModelResourceClone(self.job, body)
            self.job.Model.addObject(clone)
            self._new_clone_map[body.Name] = clone.Name
            FreeCAD.Console.PrintMessage(
                "  added clone '%s' for body '%s'\n"
                % (clone.Label, body.Label)
            )
        # Clear so they aren't added again on subsequent previews.
        self.new_bodies = []

    def _savePlacements(self):
        """Snapshot every model clone's Placement before nesting."""
        models = (
            list(self.job.Model.Group)
            if hasattr(self.job, "Model") and self.job.Model
            else []
        )
        for m in models:
            if m.Name not in self._orig_placements:
                self._orig_placements[m.Name] = m.Placement.copy()

    def _restorePlacements(self):
        """Reset every model clone to its pre-nest Placement."""
        for name, plc in self._orig_placements.items():
            obj = FreeCAD.ActiveDocument.getObject(name)
            if obj is not None:
                obj.Placement = plc.copy()

    def _runNesting(self):
        """Execute nesting with current UI settings."""
        bias, gravity_point = self._getBiasParams()
        try:
            report = PathNesting.nestModels(
                self.job,
                spacing=self.spinGap.value(),
                allow_rotation=self.chkRotation.isChecked(),
                rotation_step=self.spinRotStep.value(),
                bias=bias,
                gravity_point=gravity_point,
            )
            FreeCAD.Console.PrintMessage(report + "\n")
        except Exception as e:
            FreeCAD.Console.PrintError("Nesting failed: %s\n" % str(e))
            import traceback
            traceback.print_exc()
        FreeCAD.ActiveDocument.recompute()

    def _onSettingChanged(self, _value=None):
        """Auto-update preview when a packing parameter changes."""
        self._onUpdatePreview()

    def _onBiasChanged(self, index):
        """Enable custom-point spinners only for 'Custom Point' bias."""
        is_custom = (
            PathNesting.PackingBias.ALL[index]
            == PathNesting.PackingBias.CustomPoint
        )
        self.spinGravityX.setEnabled(is_custom)
        self.spinGravityY.setEnabled(is_custom)
        self._onSettingChanged()

    def _getBiasParams(self):
        """Return ``(bias, gravity_point)`` from the current UI state."""
        idx = self.comboBias.currentIndex()
        bias = PathNesting.PackingBias.ALL[idx]
        gravity_point = None
        if bias == PathNesting.PackingBias.CustomPoint:
            gravity_point = (
                self.spinGravityX.value(),
                self.spinGravityY.value(),
            )
        return bias, gravity_point

    def _onUpdatePreview(self):
        """Re-run nesting with current settings."""
        self._restorePlacements()
        self._runNesting()

    # ----- Task panel protocol (duck-typed for FreeCADGui.Control) ---------

    def open(self):
        """Called by FreeCAD when the panel is shown."""
        FreeCAD.ActiveDocument.openTransaction("Nest Models")
        self._addNewBodies()
        self._savePlacements()
        self._runNesting()

    def accept(self):
        """OK -- restore original placements, run final nesting, commit."""
        self._restorePlacements()
        self._runNesting()
        FreeCAD.ActiveDocument.commitTransaction()
        FreeCADGui.Control.closeDialog()
        return True

    def reject(self):
        """Cancel -- abort the transaction (reverts everything)."""
        FreeCAD.ActiveDocument.abortTransaction()
        FreeCADGui.Control.closeDialog()
        return True

    def getStandardButtons(self):
        return int(
            QtGui.QDialogButtonBox.Ok | QtGui.QDialogButtonBox.Cancel
        )


# ---------------------------------------------------------------------------
#   FreeCAD command
# ---------------------------------------------------------------------------


class CommandPathNesting:
    """Arranges the models of the active CAM Job within its stock bounds."""

    def GetResources(self):
        return {
            "Pixmap": "CAM_Nesting",
            "MenuText": QT_TRANSLATE_NOOP("CAM_Nesting", "Nesting"),
            "ToolTip": QT_TRANSLATE_NOOP(
                "CAM_Nesting",
                "Arrange the models of a CAM Job to fit within the stock area.\n"
                "Select additional bodies first to add them to the Job.",
            ),
        }

    def IsActive(self):
        if FreeCAD.ActiveDocument is None:
            return False
        for obj in FreeCAD.ActiveDocument.Objects:
            if obj.Name[:3] == "Job":
                return True
        return False

    def Activated(self):
        sel = FreeCADGui.Selection.getSelection()

        # -- Find the active Job -------------------------------------------
        jobs = PathUtils.GetJobs()
        if not jobs:
            FreeCAD.Console.PrintError(
                translate(
                    "CAM_Nesting",
                    "No CAM Job found in the active document.",
                )
                + "\n"
            )
            return

        job = None
        if sel:
            job = PathUtils.findParentJob(sel[0])
        if job is None:
            if len(jobs) == 1:
                job = jobs[0]
            else:
                job = PathUtils.UserInput.chooseJob(jobs)
        if job is None:
            return

        # -- Classify selection --------------------------------------------
        _job_objs, new_bodies = _classifySelection(sel, job)

        has_existing = (
            hasattr(job, "Model")
            and job.Model is not None
            and len(job.Model.Group) > 0
        )
        if not has_existing and not new_bodies:
            FreeCAD.Console.PrintError(
                translate(
                    "CAM_Nesting",
                    "The selected Job has no models and no bodies are "
                    "selected to add.",
                )
                + "\n"
            )
            return

        if job.Stock is None:
            FreeCAD.Console.PrintError(
                translate(
                    "CAM_Nesting",
                    "The selected Job has no stock defined.",
                )
                + "\n"
            )
            return

        # -- Show the task panel -------------------------------------------
        panel = NestingTaskPanel(job, new_bodies)
        FreeCADGui.Control.closeDialog()
        FreeCADGui.Control.showDialog(panel)


if FreeCAD.GuiUp:
    FreeCADGui.addCommand("CAM_Nesting", CommandPathNesting())

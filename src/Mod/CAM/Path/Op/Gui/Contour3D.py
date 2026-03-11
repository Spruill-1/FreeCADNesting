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

from PySide import QtCore, QtGui
from PySide.QtCore import QT_TRANSLATE_NOOP
import FreeCAD
import FreeCADGui
import Path
import Path.Op.Contour3D as PathContour3D
import Path.Op.Gui.Base as PathOpGui
import Path.Base.Gui.Util as PathGuiUtil

HAS_CONTOUR3D_GUI = True
try:
    import Path.Op.Gui.Waterline as PathWaterlineGui
except ImportError:
    HAS_CONTOUR3D_GUI = False
    PathWaterlineGui = None

__title__ = "CAM 3D Contour Operation UI"
__author__ = "FreeCAD contributors"
__url__ = "https://www.freecad.org"
__doc__ = "3D Contour operation page controller and command implementation."

translate = FreeCAD.Qt.translate

if False:
    Path.Log.setLevel(Path.Log.Level.DEBUG, Path.Log.thisModule())
    Path.Log.trackModule(Path.Log.thisModule())
else:
    Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())


if HAS_CONTOUR3D_GUI:

    class TaskPanelOpPage(PathWaterlineGui.TaskPanelOpPage):
        """Page controller for the 3D Contour operation."""

        def getForm(self):
            # Start from Waterline's mature 3D panel, then layer the bevel-
            # specific widgets on top so the operation inherits the existing
            # 3D CAM editing experience.
            form = super().getForm()
            self._ensureContourWidgets(form)
            self._restrictAlgorithmChoices(form)
            return form

        def initPage(self, obj):
            self.setTitle("3D Contour - " + obj.Label)
            self.updateVisibility()

        def getFields(self, obj):
            super().getFields(obj)

            # The Waterline page controller owns the shared 3D properties.
            # Everything below copies only the Contour3D-specific extensions.
            if obj.PassMode != str(self.form.contourPassMode.currentData()):
                obj.PassMode = str(self.form.contourPassMode.currentData())

            if obj.SpringPass != self.form.contourSpringPass.isChecked():
                obj.SpringPass = self.form.contourSpringPass.isChecked()

            if obj.BoundaryCleanupPass != self.form.contourBoundaryCleanup.isChecked():
                obj.BoundaryCleanupPass = self.form.contourBoundaryCleanup.isChecked()

            if obj.CutMode != str(self.form.contourCutMode.currentData()):
                obj.CutMode = str(self.form.contourCutMode.currentData())

            if obj.HandleMultipleFeatures != str(self.form.contourFeatureHandling.currentData()):
                obj.HandleMultipleFeatures = str(self.form.contourFeatureHandling.currentData())

            if obj.BoundaryEnforcement != self.form.contourBoundaryEnforcement.isChecked():
                obj.BoundaryEnforcement = self.form.contourBoundaryEnforcement.isChecked()

            if obj.KeepToolDown != self.form.contourKeepToolDown.isChecked():
                obj.KeepToolDown = self.form.contourKeepToolDown.isChecked()

            if float(obj.MinStayDownDistance.Value) != float(self.form.contourMinStayDownDistance.value()):
                obj.MinStayDownDistance = self.form.contourMinStayDownDistance.value()

            if float(obj.MinBevelTilt) != float(self.form.contourMinBevelTilt.value()):
                obj.MinBevelTilt = self.form.contourMinBevelTilt.value()

        def setFields(self, obj):
            super().setFields(obj)
            self.selectInComboBox(obj.PassMode, self.form.contourPassMode)
            self.selectInComboBox(obj.CutMode, self.form.contourCutMode)
            self.selectInComboBox(
                obj.HandleMultipleFeatures,
                self.form.contourFeatureHandling,
            )
            self.form.contourSpringPass.setChecked(bool(obj.SpringPass))
            self.form.contourBoundaryCleanup.setChecked(bool(obj.BoundaryCleanupPass))
            self.form.contourBoundaryEnforcement.setChecked(bool(obj.BoundaryEnforcement))
            self.form.contourKeepToolDown.setChecked(bool(obj.KeepToolDown))
            self.form.contourMinStayDownDistance.setValue(float(obj.MinStayDownDistance.Value))
            self.form.contourMinBevelTilt.setValue(float(obj.MinBevelTilt))
            self.updateVisibility()

        def getSignalsForUpdate(self, obj):
            signals = super().getSignalsForUpdate(obj)
            signals.append(self.form.contourPassMode.currentIndexChanged)
            signals.append(self.form.contourCutMode.currentIndexChanged)
            signals.append(self.form.contourFeatureHandling.currentIndexChanged)

            if hasattr(self.form.contourSpringPass, "checkStateChanged"):
                signals.append(self.form.contourSpringPass.checkStateChanged)
                signals.append(self.form.contourBoundaryCleanup.checkStateChanged)
                signals.append(self.form.contourBoundaryEnforcement.checkStateChanged)
                signals.append(self.form.contourKeepToolDown.checkStateChanged)
            else:
                signals.append(self.form.contourSpringPass.stateChanged)
                signals.append(self.form.contourBoundaryCleanup.stateChanged)
                signals.append(self.form.contourBoundaryEnforcement.stateChanged)
                signals.append(self.form.contourKeepToolDown.stateChanged)

            signals.append(self.form.contourMinStayDownDistance.editingFinished)
            signals.append(self.form.contourMinBevelTilt.valueChanged)
            return signals

        def registerSignalHandlers(self, obj):
            super().registerSignalHandlers(obj)
            self.form.contourPassMode.currentIndexChanged.connect(self.updateVisibility)
            self.form.contourKeepToolDown.toggled.connect(self.updateVisibility)
            self.form.contourSpringPass.toggled.connect(self.updateVisibility)
            self.form.contourBoundaryCleanup.toggled.connect(self.updateVisibility)

        def updateVisibility(self, sentObj=None):
            super().updateVisibility(sentObj)

            # Spring pass and boundary cleanup only make sense when a dedicated
            # finish pass exists, so hide them in slices-only mode.
            show_finish_options = self.form.contourPassMode.currentData() == "SlicesPlusFinish"
            self.form.contourSpringPass.setVisible(show_finish_options)
            self.form.contourBoundaryCleanup.setVisible(show_finish_options)

            if hasattr(self.form, "contourSpringPass_label"):
                self.form.contourSpringPass_label.setVisible(show_finish_options)
            if hasattr(self.form, "contourBoundaryCleanup_label"):
                self.form.contourBoundaryCleanup_label.setVisible(show_finish_options)

            # The numeric threshold is only relevant when stay-down linking is
            # enabled.
            keep_tool_down = self.form.contourKeepToolDown.isChecked()
            self.form.contourMinStayDownDistance.setVisible(keep_tool_down)
            self.form.contourMinStayDownDistance_label.setVisible(keep_tool_down)

        def _ensureContourWidgets(self, form):
            if hasattr(form, "contourOptionsGroup"):
                return

            # These widgets are created dynamically so the operation can reuse
            # Waterline's UI file without needing a dedicated .ui resource yet.
            form.contourOptionsGroup = QtGui.QGroupBox(
                translate("CAM_Contour3D", "Bevel finishing")
            )
            layout = QtGui.QGridLayout(form.contourOptionsGroup)

            form.contourPassMode_label = QtGui.QLabel(
                translate("CAM_Contour3D", "Pass mode")
            )
            form.contourPassMode = QtGui.QComboBox()
            enum_tups = PathContour3D._contour_property_enumerations(dataType="raw")
            # Merge the local contour enumerations with the underlying
            # Waterline enumerations so shared controls such as CutMode and
            # HandleMultipleFeatures can be populated from one map.
            enum_tups.update(PathContour3D.PathWaterline.ObjectWaterline.propertyEnumerations(dataType="raw"))
            PathGuiUtil.populateCombobox(
                form,
                enum_tups,
                [("contourPassMode", "PassMode")],
            )
            form.contourPassMode.setToolTip(
                translate(
                    "CAM_Contour3D",
                    "Choose whether the toolpath runs slice passes only or adds a finish pass.",
                )
            )

            form.contourSpringPass_label = QtGui.QLabel(
                translate("CAM_Contour3D", "Spring pass")
            )
            form.contourSpringPass = QtGui.QCheckBox(
                translate("CAM_Contour3D", "Repeat the final finishing pass")
            )

            form.contourCutMode_label = QtGui.QLabel(
                translate("CAM_Contour3D", "Cut direction")
            )
            form.contourCutMode = QtGui.QComboBox()
            PathGuiUtil.populateCombobox(
                form,
                enum_tups,
                [("contourCutMode", "CutMode")],
            )

            form.contourFeatureHandling_label = QtGui.QLabel(
                translate("CAM_Contour3D", "Multiple face handling")
            )
            form.contourFeatureHandling = QtGui.QComboBox()
            PathGuiUtil.populateCombobox(
                form,
                enum_tups,
                [("contourFeatureHandling", "HandleMultipleFeatures")],
            )

            form.contourBoundaryCleanup_label = QtGui.QLabel(
                translate("CAM_Contour3D", "Boundary cleanup")
            )
            form.contourBoundaryCleanup = QtGui.QCheckBox(
                translate("CAM_Contour3D", "Profile the longest deepest bevel loop")
            )

            form.contourBoundaryEnforcement = QtGui.QCheckBox(
                translate("CAM_Contour3D", "Constrain passes to the selected bevel faces")
            )

            form.contourKeepToolDown = QtGui.QCheckBox(
                translate("CAM_Contour3D", "Keep tool down between nearby loops")
            )

            form.contourMinStayDownDistance_label = QtGui.QLabel(
                translate("CAM_Contour3D", "Min stay-down distance")
            )
            form.contourMinStayDownDistance = QtGui.QDoubleSpinBox()
            form.contourMinStayDownDistance.setDecimals(3)
            form.contourMinStayDownDistance.setRange(0.0, 1000000.0)
            form.contourMinStayDownDistance.setSingleStep(0.5)
            form.contourMinStayDownDistance.setSuffix(" mm")

            form.contourMinBevelTilt_label = QtGui.QLabel(
                translate("CAM_Contour3D", "Min bevel tilt")
            )
            form.contourMinBevelTilt = QtGui.QDoubleSpinBox()
            form.contourMinBevelTilt.setDecimals(1)
            form.contourMinBevelTilt.setRange(0.0, 89.9)
            form.contourMinBevelTilt.setSingleStep(1.0)
            form.contourMinBevelTilt.setSuffix("°")
            form.contourMinBevelTilt.setToolTip(
                translate(
                    "CAM_Contour3D",
                    "Faces flatter than this angle from horizontal are ignored by the bevel validation.",
                )
            )

            form.contourHint = QtGui.QLabel(
                translate(
                    "CAM_Contour3D",
                    "Select the inside bevel faces of the frame. Horizontal top and bottom faces are ignored.",
                )
            )
            form.contourHint.setWordWrap(True)
            form.contourHint.setStyleSheet("color: palette(mid);")

            layout.addWidget(form.contourPassMode_label, 0, 0)
            layout.addWidget(form.contourPassMode, 0, 1)
            layout.addWidget(form.contourCutMode_label, 1, 0)
            layout.addWidget(form.contourCutMode, 1, 1)
            layout.addWidget(form.contourFeatureHandling_label, 2, 0)
            layout.addWidget(form.contourFeatureHandling, 2, 1)
            layout.addWidget(form.contourSpringPass_label, 3, 0)
            layout.addWidget(form.contourSpringPass, 3, 1)
            layout.addWidget(form.contourBoundaryCleanup_label, 4, 0)
            layout.addWidget(form.contourBoundaryCleanup, 4, 1)
            layout.addWidget(form.contourBoundaryEnforcement, 5, 0, 1, 2)
            layout.addWidget(form.contourKeepToolDown, 6, 0, 1, 2)
            layout.addWidget(form.contourMinStayDownDistance_label, 7, 0)
            layout.addWidget(form.contourMinStayDownDistance, 7, 1)
            layout.addWidget(form.contourMinBevelTilt_label, 8, 0)
            layout.addWidget(form.contourMinBevelTilt, 8, 1)
            layout.addWidget(form.contourHint, 9, 0, 1, 2)

            form.verticalLayout.insertWidget(2, form.contourOptionsGroup)

        def _restrictAlgorithmChoices(self, form):
            # ``Contour3D`` deliberately fixes the Waterline algorithm to the
            # face-aware OCL Adaptive path.  Reflect that in the UI so the user
            # is not offered unsupported choices.
            form.algorithmSelect.clear()
            for _label, value in PathContour3D._contour_property_enumerations(dataType="raw")["Algorithm"]:
                form.algorithmSelect.addItem(value, value)
            form.algorithmSelect.setEnabled(False)
            if hasattr(form, "algorithmSelect_label"):
                form.algorithmSelect_label.setText(
                    translate("CAM_Contour3D", "Algorithm (fixed)")
                )

else:

    class TaskPanelOpPage(PathOpGui.TaskPanelPage):
        """Placeholder page when OCL-backed Waterline UI is unavailable."""

        def initPage(self, obj):
            self.setTitle("3D Contour - " + obj.Label)

        def getForm(self):
            form = FreeCADGui.PySideUic.loadUi(":/panels/PageOpCustomEdit.ui")
            return form

        def getFields(self, obj):
            return

        def setFields(self, obj):
            return

        def getSignalsForUpdate(self, obj):
            return []


Command = PathOpGui.SetupOperation(
    "Contour3D",
    PathContour3D.Create,
    TaskPanelOpPage,
    "CAM_Waterline",
    QT_TRANSLATE_NOOP("CAM_Contour3D", "3D Contour"),
    QT_TRANSLATE_NOOP(
        "CAM_Contour3D", "Create a 3D contour / bevel finishing toolpath from selected faces"
    ),
    PathContour3D.SetupProperties,
)

FreeCAD.Console.PrintLog("Loading PathContour3DGui... done\n")
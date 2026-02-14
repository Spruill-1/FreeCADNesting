# SPDX-License-Identifier: LGPL-2.1-or-later

# ***************************************************************************
# *   Copyright (c) 2017 sliptonic <shopinthewoods@gmail.com>               *
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

import FreeCAD
import FreeCADGui
import Path
import Path.Base.Gui.Util as PathGuiUtil
import Path.Op.Gui.Base as PathOpGui
import Path.Op.Profile as PathProfile
import Path.Op.TagUtils as TagUtils
import PathGui
from PySide.QtCore import QT_TRANSLATE_NOOP

try:
    from pivy import coin
except ImportError:
    coin = None


__title__ = "CAM Profile Operation UI"
__author__ = "sliptonic (Brad Collette)"
__url__ = "https://www.freecad.org"
__doc__ = "Profile operation page controller and command implementation."


FeatureSide = 0x01
FeatureProcessing = 0x02


class TaskPanelOpPage(PathOpGui.TaskPanelPage):
    """Base class for profile operation page controllers. Two sub features are supported:
    FeatureSide       ... Is the Side property exposed in the UI
    FeatureProcessing ... Are the processing check boxes supported by the operation
    """

    # def initPage(self, obj):
    #     self.setTitle("Profile - " + obj.Label)
    #     self.updateVisibility()

    def initPage(self, obj):
        """initPage(obj) ... set up coin3d nodes for tab preview markers."""
        self._tabSwitch = None
        if coin is not None:
            self._tabSwitch = coin.SoSwitch()
            self._tabSwitch.whichChild = coin.SO_SWITCH_NONE
            try:
                obj.ViewObject.RootNode.addChild(self._tabSwitch)
            except Exception:
                self._tabSwitch = None

    def cleanupPage(self, obj):
        """cleanupPage(obj) ... remove coin3d tab preview nodes."""
        if self._tabSwitch is not None:
            try:
                obj.ViewObject.RootNode.removeChild(self._tabSwitch)
            except (ReferenceError, RuntimeError):
                pass
            self._tabSwitch = None

    def profileFeatures(self):
        """profileFeatures() ... return which of the optional profile features are supported.
        Currently two features are supported and returned:
            FeatureSide       ... Is the Side property exposed in the UI
            FeatureProcessing ... Are the processing check boxes supported by the operation
        ."""
        return FeatureSide | FeatureProcessing

    def getForm(self):
        """getForm() ... returns UI customized according to profileFeatures()"""
        form = FreeCADGui.PySideUic.loadUi(":/panels/PageOpProfileFullEdit.ui")

        comboToPropertyMap = [("cutSide", "Side"), ("direction", "Direction")]
        enumTups = PathProfile.ObjectProfile.areaOpPropertyEnumerations(dataType="raw")

        self.populateCombobox(form, enumTups, comboToPropertyMap)
        return form

    def getFields(self, obj):
        """getFields(obj) ... transfers values from UI to obj's properties"""
        self.updateToolController(obj, self.form.toolController)
        self.updateCoolant(obj, self.form.coolantController)

        if obj.Side != str(self.form.cutSide.currentData()):
            obj.Side = str(self.form.cutSide.currentData())
        if obj.Direction != str(self.form.direction.currentData()):
            obj.Direction = str(self.form.direction.currentData())
        PathGuiUtil.updateInputField(obj, "OffsetExtra", self.form.extraOffset)
        obj.NumPasses = self.form.numPasses.value()
        PathGuiUtil.updateInputField(obj, "Stepover", self.form.stepover)

        if obj.UseComp != self.form.useCompensation.isChecked():
            obj.UseComp = self.form.useCompensation.isChecked()
        if obj.UseStartPoint != self.form.useStartPoint.isChecked():
            obj.UseStartPoint = self.form.useStartPoint.isChecked()

        if obj.processHoles != self.form.processHoles.isChecked():
            obj.processHoles = self.form.processHoles.isChecked()
        if obj.processPerimeter != self.form.processPerimeter.isChecked():
            obj.processPerimeter = self.form.processPerimeter.isChecked()
        if obj.processCircles != self.form.processCircles.isChecked():
            obj.processCircles = self.form.processCircles.isChecked()

        # Tabs
        if obj.UseTabs != self.form.useTabs.isChecked():
            obj.UseTabs = self.form.useTabs.isChecked()
        PathGuiUtil.updateInputField(obj, "TabSpacing", self.form.tabSpacing)
        PathGuiUtil.updateInputField(obj, "TabWidth", self.form.tabWidth)
        PathGuiUtil.updateInputField(obj, "TabHeight", self.form.tabHeight)
        if obj.TabAngle != self.form.tabAngle.value():
            obj.TabAngle = self.form.tabAngle.value()
        if obj.PruneTabs != self.form.pruneTabs.isChecked():
            obj.PruneTabs = self.form.pruneTabs.isChecked()

    def setFields(self, obj):
        """setFields(obj) ... transfers obj's property values to UI"""
        self.setupToolController(obj, self.form.toolController)
        self.setupCoolant(obj, self.form.coolantController)

        self.selectInComboBox(obj.Side, self.form.cutSide)
        self.selectInComboBox(obj.Direction, self.form.direction)
        self.form.extraOffset.setText(
            FreeCAD.Units.Quantity(obj.OffsetExtra.Value, FreeCAD.Units.Length).UserString
        )
        self.form.numPasses.setValue(obj.NumPasses)
        self.form.stepover.setText(
            FreeCAD.Units.Quantity(obj.Stepover.Value, FreeCAD.Units.Length).UserString
        )

        self.form.useCompensation.setChecked(obj.UseComp)
        self.form.useStartPoint.setChecked(obj.UseStartPoint)
        self.form.processHoles.setChecked(obj.processHoles)
        self.form.processPerimeter.setChecked(obj.processPerimeter)
        self.form.processCircles.setChecked(obj.processCircles)

        # Tabs
        self.form.useTabs.setChecked(obj.UseTabs)
        self.form.tabSpacing.setText(
            FreeCAD.Units.Quantity(obj.TabSpacing.Value, FreeCAD.Units.Length).UserString
        )
        self.form.tabWidth.setText(
            FreeCAD.Units.Quantity(obj.TabWidth.Value, FreeCAD.Units.Length).UserString
        )
        self.form.tabHeight.setText(
            FreeCAD.Units.Quantity(obj.TabHeight.Value, FreeCAD.Units.Length).UserString
        )
        self.form.tabAngle.setValue(obj.TabAngle)
        self.form.pruneTabs.setChecked(obj.PruneTabs)

        self.updateVisibility()

    def getSignalsForUpdate(self, obj):
        """getSignalsForUpdate(obj) ... return list of signals for updating obj"""
        signals = []
        signals.append(self.form.toolController.currentIndexChanged)
        signals.append(self.form.coolantController.currentIndexChanged)
        signals.append(self.form.cutSide.currentIndexChanged)
        signals.append(self.form.direction.currentIndexChanged)
        signals.append(self.form.extraOffset.editingFinished)
        signals.append(self.form.numPasses.editingFinished)
        signals.append(self.form.stepover.editingFinished)
        if hasattr(self.form.useCompensation, "checkStateChanged"):  # Qt version >= 6.7.0
            signals.append(self.form.useCompensation.checkStateChanged)
            signals.append(self.form.useStartPoint.checkStateChanged)
            signals.append(self.form.processHoles.checkStateChanged)
            signals.append(self.form.processPerimeter.checkStateChanged)
            signals.append(self.form.processCircles.checkStateChanged)
            signals.append(self.form.useTabs.checkStateChanged)
        else:  # Qt version < 6.7.0
            signals.append(self.form.useCompensation.stateChanged)
            signals.append(self.form.useStartPoint.stateChanged)
            signals.append(self.form.processHoles.stateChanged)
            signals.append(self.form.processPerimeter.stateChanged)
            signals.append(self.form.processCircles.stateChanged)
            signals.append(self.form.useTabs.stateChanged)

        # Tab property signals
        signals.append(self.form.tabSpacing.editingFinished)
        signals.append(self.form.tabWidth.editingFinished)
        signals.append(self.form.tabHeight.editingFinished)
        signals.append(self.form.tabAngle.editingFinished)
        if hasattr(self.form.pruneTabs, "checkStateChanged"):  # Qt version >= 6.7.0
            signals.append(self.form.pruneTabs.checkStateChanged)
        else:
            signals.append(self.form.pruneTabs.stateChanged)

        return signals

    def updateVisibility(self):
        hasFace = False
        objBase = list()

        if hasattr(self.obj, "Base"):
            objBase = self.obj.Base

        if objBase.__len__() > 0:
            for base, subsList in objBase:
                for sub in subsList:
                    if sub[:4] == "Face":
                        hasFace = True
                        break

        if hasFace:
            self.form.processCircles.show()
            self.form.processHoles.show()
            self.form.processPerimeter.show()
        else:
            self.form.processCircles.hide()
            self.form.processHoles.hide()
            self.form.processPerimeter.hide()

        self.form.stepover.setEnabled(self.obj.NumPasses > 1)

        # Tab controls enabled/disabled based on UseTabs checkbox
        tabsEnabled = self.form.useTabs.isChecked()
        self.form.tabSpacing.setEnabled(tabsEnabled)
        self.form.tabWidth.setEnabled(tabsEnabled)
        self.form.tabHeight.setEnabled(tabsEnabled)
        self.form.tabAngle.setEnabled(tabsEnabled)
        self.form.pruneTabs.setEnabled(tabsEnabled)

        self._updateTabPreview()

    def _updateTabPreview(self):
        """_updateTabPreview() ... rebuild coin3d sphere markers at computed tab positions."""
        if self._tabSwitch is None:
            return

        # Clear previous markers
        self._tabSwitch.removeAllChildren()
        self._tabSwitch.whichChild = coin.SO_SWITCH_NONE

        if not self.form.useTabs.isChecked():
            return

        obj = self.obj
        try:
            path = obj.Path
        except Exception:
            return
        if not path or not path.Commands:
            return

        wire, rapid, _ = Path.Geom.wireForPath(path)
        if not wire:
            return

        rapidEdges = TagUtils._RapidEdges(rapid)
        edges = wire.Edges
        (minZ, maxZ) = TagUtils._findZLimits(edges, rapidEdges)
        bottomWires = TagUtils._findBottomWires(edges, minZ)
        if not bottomWires:
            return

        spacing = obj.TabSpacing.Value
        if spacing <= 0:
            return

        # Compute tag positions on each contour, optionally prune conflicting ones.
        tagsByWire = []
        for bw in bottomWires:
            n = max(1, round(bw.Length / spacing))
            wireTags = TagUtils.distributeTags(
                bw, n,
                obj.TabWidth.Value,
                obj.TabHeight.Value,
                obj.TabAngle,
                0,  # no tool radius padding for preview position
            )
            tagsByWire.append(wireTags)

        if getattr(obj, "PruneTabs", True) and self.form.pruneTabs.isChecked():
            allTags = TagUtils.pruneConflictingTags(tagsByWire, bottomWires, 0)
        else:
            allTags = [t for group in tagsByWire for t in group]

        if not allTags:
            return

        # Build a coin3d separator for each marker.
        # Enabled tabs are green; pruned (disabled) tabs are red/transparent.
        for tag in allTags:
            sep = coin.SoSeparator()

            pos = coin.SoTranslation()
            pos.translation = (tag.x, tag.y, minZ)

            mat = coin.SoMaterial()
            if tag.enabled:
                mat.diffuseColor = (0.0, 0.8, 0.0)  # green
                mat.transparency = 0.3
            else:
                mat.diffuseColor = (0.8, 0.0, 0.0)  # red
                mat.transparency = 0.6

            sphere = coin.SoSphere()
            scale = coin.SoType.fromName("SoShapeScale").createInstance()
            scale.setPart("shape", sphere)
            scale.scaleFactor.setValue(14)

            sep.addChild(pos)
            sep.addChild(mat)
            sep.addChild(scale)
            self._tabSwitch.addChild(sep)

        self._tabSwitch.whichChild = coin.SO_SWITCH_ALL

    def registerSignalHandlers(self, obj):
        if hasattr(self.form.useCompensation, "checkStateChanged"):  # Qt version >= 6.7.0
            self.form.useCompensation.checkStateChanged.connect(self.updateVisibility)
            self.form.useTabs.checkStateChanged.connect(self.updateVisibility)
            self.form.pruneTabs.checkStateChanged.connect(self.updateVisibility)
        else:  # Qt version < 6.7.0
            self.form.useCompensation.stateChanged.connect(self.updateVisibility)
            self.form.useTabs.stateChanged.connect(self.updateVisibility)
            self.form.pruneTabs.stateChanged.connect(self.updateVisibility)
        self.form.numPasses.editingFinished.connect(self.updateVisibility)


# Eclass


Command = PathOpGui.SetupOperation(
    "Profile",
    PathProfile.Create,
    TaskPanelOpPage,
    "CAM_Profile",
    QT_TRANSLATE_NOOP("CAM_Profile", "Profile"),
    QT_TRANSLATE_NOOP("CAM_Profile", "Profile entire model, selected face(s) or selected edge(s)"),
    PathProfile.SetupProperties,
)

FreeCAD.Console.PrintLog("Loading PathProfileGui ... done\n")

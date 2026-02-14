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

from Path.Dressup.Gui.TagPreferences import HoldingTagPreferences
from PathScripts.PathUtils import waiting_effects
from PySide.QtCore import QT_TRANSLATE_NOOP
import FreeCAD
import Path
import Path.Dressup.Utils as PathDressup
import PathScripts.PathUtils as PathUtils
import copy
import math

from Path.Op.TagUtils import Tag, MapWireToTag, _RapidEdges  # noqa: F401

# lazily loaded modules
from lazy_loader.lazy_loader import LazyLoader

Part = LazyLoader("Part", globals(), "Part")

if False:
    Path.Log.setLevel(Path.Log.Level.DEBUG, Path.Log.thisModule())
    Path.Log.trackModule(Path.Log.thisModule())
else:
    Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())

translate = FreeCAD.Qt.translate


def debugEdge(edge, prefix, force=False):
    if force or Path.Log.getLevel(Path.Log.thisModule()) == Path.Log.Level.DEBUG:
        pf = edge.valueAt(edge.FirstParameter)
        pl = edge.valueAt(edge.LastParameter)
        if type(edge.Curve) in [Part.Line, Part.LineSegment]:
            print(
                "%s %s((%.2f, %.2f, %.2f) - (%.2f, %.2f, %.2f))"
                % (prefix, type(edge.Curve), pf.x, pf.y, pf.z, pl.x, pl.y, pl.z)
            )
        else:
            pm = edge.valueAt((edge.FirstParameter + edge.LastParameter) / 2)
            print(
                "%s %s((%.2f, %.2f, %.2f) - (%.2f, %.2f, %.2f) - (%.2f, %.2f, %.2f))"
                % (
                    prefix,
                    type(edge.Curve),
                    pf.x,
                    pf.y,
                    pf.z,
                    pm.x,
                    pm.y,
                    pm.z,
                    pl.x,
                    pl.y,
                    pl.z,
                )
            )


def debugMarker(vector, label, color=None, radius=0.5):
    if Path.Log.getLevel(Path.Log.thisModule()) == Path.Log.Level.DEBUG:
        obj = FreeCAD.ActiveDocument.addObject("Part::Sphere", label)
        obj.Label = label
        obj.Radius = radius
        obj.Placement = FreeCAD.Placement(vector, FreeCAD.Rotation(FreeCAD.Vector(0, 0, 1), 0))
        if color:
            obj.ViewObject.ShapeColor = color


def debugCylinder(vector, r, height, label, color=None):
    if Path.Log.getLevel(Path.Log.thisModule()) == Path.Log.Level.DEBUG:
        obj = FreeCAD.ActiveDocument.addObject("Part::Cylinder", label)
        obj.Label = label
        obj.Radius = r
        obj.Height = height
        obj.Placement = FreeCAD.Placement(vector, FreeCAD.Rotation(FreeCAD.Vector(0, 0, 1), 0))
        obj.ViewObject.Transparency = 90
        if color:
            obj.ViewObject.ShapeColor = color


def debugCone(vector, r1, r2, height, label, color=None):
    if Path.Log.getLevel(Path.Log.thisModule()) == Path.Log.Level.DEBUG:
        obj = FreeCAD.ActiveDocument.addObject("Part::Cone", label)
        obj.Label = label
        obj.Radius1 = r1
        obj.Radius2 = r2
        obj.Height = height
        obj.Placement = FreeCAD.Placement(vector, FreeCAD.Rotation(FreeCAD.Vector(0, 0, 1), 0))
        obj.ViewObject.Transparency = 90
        if color:
            obj.ViewObject.ShapeColor = color


# Tag class is now imported from Path.Op.TagUtils


# MapWireToTag class is now imported from Path.Op.TagUtils


# _RapidEdges class is now imported from Path.Op.TagUtils


class PathData:
    def __init__(self, obj):
        Path.Log.track(obj.Base.Name)
        self.obj = obj
        path = PathUtils.getPathWithPlacement(obj.Base)
        self.wire, rapid, rapid_indexes = Path.Geom.wireForPath(path)
        self.rapid = _RapidEdges(rapid)
        if self.wire:
            self.edges = self.wire.Edges
        else:
            self.edges = []
        self.baseWire = self.findBottomWire(self.edges)

    def findBottomWire(self, edges):
        (minZ, maxZ) = self.findZLimits(edges)
        self.minZ = minZ
        self.maxZ = maxZ
        bottom = [
            e
            for e in edges
            if Path.Geom.isRoughly(e.Vertexes[0].Point.z, minZ)
            and Path.Geom.isRoughly(e.Vertexes[1].Point.z, minZ)
        ]
        self.bottomEdges = bottom
        try:
            wire = Part.Wire(bottom)
            if wire.isClosed():
                return wire
        except Exception:
            return None

    def supportsTagGeneration(self):
        return self.baseWire is not None

    def findZLimits(self, edges):
        # not considering arcs and spheres in Z direction, find the highest and lowest Z values
        minZ = 99999999999
        maxZ = -99999999999
        for e in edges:
            if self.rapid.isRapid(e):
                continue
            for v in e.Vertexes:
                if v.Point.z < minZ:
                    minZ = v.Point.z
                if v.Point.z > maxZ:
                    maxZ = v.Point.z
        return (minZ, maxZ)

    def shortestAndLongestPathEdge(self):
        edges = sorted(self.bottomEdges, key=lambda e: e.Length)
        return (edges[0], edges[-1])

    def generateTags(
        self, obj, count, width=None, height=None, angle=None, radius=None, spacing=None
    ):
        Path.Log.track(count, width, height, angle, spacing)
        # for e in self.baseWire.Edges:
        #    debugMarker(e.Vertexes[0].Point, 'base', (0.0, 1.0, 1.0), 0.2)

        if spacing:
            tagDistance = spacing
        else:
            tagDistance = self.baseWire.Length / (count if count else 4)

        W = width if width else self.defaultTagWidth()
        H = height if height else self.defaultTagHeight()
        A = angle if angle else self.defaultTagAngle()
        R = radius if radius else self.defaultTagRadius()

        # start assigning tags on the longest segment
        (shortestEdge, longestEdge) = self.shortestAndLongestPathEdge()
        startIndex = 0
        for i in range(0, len(self.baseWire.Edges)):
            edge = self.baseWire.Edges[i]
            Path.Log.debug("  %d: %.2f" % (i, edge.Length))
            if Path.Geom.isRoughly(edge.Length, longestEdge.Length):
                startIndex = i
                break

        startEdge = self.baseWire.Edges[startIndex]
        startCount = int(startEdge.Length / tagDistance)
        if (longestEdge.Length - shortestEdge.Length) > shortestEdge.Length:
            startCount = int(startEdge.Length / tagDistance) + 1

        lastTagLength = (startEdge.Length + (startCount - 1) * tagDistance) / 2
        currentLength = startEdge.Length

        minLength = min(2.0 * W, longestEdge.Length)

        Path.Log.debug(
            "length=%.2f shortestEdge=%.2f(%.2f) longestEdge=%.2f(%.2f) minLength=%.2f"
            % (
                self.baseWire.Length,
                shortestEdge.Length,
                shortestEdge.Length / self.baseWire.Length,
                longestEdge.Length,
                longestEdge.Length / self.baseWire.Length,
                minLength,
            )
        )
        Path.Log.debug(
            "   start: index=%-2d count=%d (length=%.2f, distance=%.2f)"
            % (startIndex, startCount, startEdge.Length, tagDistance)
        )
        Path.Log.debug("               -> lastTagLength=%.2f)" % lastTagLength)
        Path.Log.debug("               -> currentLength=%.2f)" % currentLength)

        edgeDict = {startIndex: startCount}

        for i in range(startIndex + 1, len(self.baseWire.Edges)):
            edge = self.baseWire.Edges[i]
            (currentLength, lastTagLength) = self.processEdge(
                i, edge, currentLength, lastTagLength, tagDistance, minLength, edgeDict
            )
        for i in range(0, startIndex):
            edge = self.baseWire.Edges[i]
            (currentLength, lastTagLength) = self.processEdge(
                i, edge, currentLength, lastTagLength, tagDistance, minLength, edgeDict
            )

        tags = []

        for i, count in edgeDict.items():
            edge = self.baseWire.Edges[i]
            Path.Log.debug(" %d: %d" % (i, count))
            # debugMarker(edge.Vertexes[0].Point, 'base', (1.0, 0.0, 0.0), 0.2)
            # debugMarker(edge.Vertexes[1].Point, 'base', (0.0, 1.0, 0.0), 0.2)
            if 0 != count:
                distance = (edge.LastParameter - edge.FirstParameter) / count
                for j in range(0, count):
                    tag = edge.Curve.value((j + 0.5) * distance)
                    tags.append(Tag(j, tag.x, tag.y, W, H, A, R, True))

        return tags

    def copyTags(self, obj, fromObj, width, height, angle, radius, production=True):
        print(
            "copyTags(%s, %s, %.2f, %.2f, %.2f, %.2f"
            % (obj.Label, fromObj.Label, width, height, angle, radius)
        )
        W = width if width else self.defaultTagWidth()
        H = height if height else self.defaultTagHeight()
        A = angle if angle else self.defaultTagAngle()
        R = radius if radius else self.defaultTagRadius()

        tags = []
        j = 0
        for i, pos in enumerate(fromObj.Positions):
            print("tag[%d]" % i)
            if i not in fromObj.Disabled:
                dist = self.baseWire.distToShape(
                    Part.Vertex(FreeCAD.Vector(pos.x, pos.y, self.minZ))
                )
                if production or dist[0] < W:
                    # russ4262:: `production` variable was a `True` declaration, forcing True branch to be processed always
                    #   The application of the `production` argument/variable is to appease LGTM
                    print("tag[%d/%d]: (%.2f, %.2f, %.2f)" % (i, j, pos.x, pos.y, self.minZ))
                    at = dist[1][0][0]
                    tags.append(Tag(j, at.x, at.y, W, H, A, R, True))
                    j += 1
                else:
                    Path.Log.warning(
                        "Tag[%d] (%.2f, %.2f, %.2f) is too far away to copy: %.2f (%.2f)"
                        % (i, pos.x, pos.y, self.minZ, dist[0], W)
                    )
            else:
                Path.Log.info("tag[%d]: not enabled, skipping" % i)
        print("copied %d tags" % len(tags))
        return tags

    def processEdge(
        self,
        index,
        edge,
        currentLength,
        lastTagLength,
        tagDistance,
        minLength,
        edgeDict,
    ):
        tagCount = 0
        currentLength += edge.Length
        if edge.Length >= minLength:
            while lastTagLength + tagDistance < currentLength:
                tagCount += 1
                lastTagLength += tagDistance
            if tagCount > 0:
                Path.Log.debug("      index=%d -> count=%d" % (index, tagCount))
                edgeDict[index] = tagCount
        else:
            Path.Log.debug("      skipping=%-2d (%.2f)" % (index, edge.Length))

        return (currentLength, lastTagLength)

    def defaultTagHeight(self):
        op = PathDressup.baseOp(self.obj.Base)
        if hasattr(op, "StartDepth") and hasattr(op, "FinalDepth"):
            pathHeight = (op.StartDepth - op.FinalDepth).Value
        else:
            pathHeight = self.maxZ - self.minZ
        height = HoldingTagPreferences.defaultHeight(pathHeight / 2)
        if height > pathHeight:
            return pathHeight
        return height

    def defaultTagWidth(self):
        width = self.shortestAndLongestPathEdge()[1].Length / 10
        return HoldingTagPreferences.defaultWidth(width)

    def defaultTagAngle(self):
        return HoldingTagPreferences.defaultAngle()

    def defaultTagRadius(self):
        return HoldingTagPreferences.defaultRadius()

    def sortedTags(self, tags):
        ordered = []
        for edge in self.bottomEdges:
            ts = [
                t
                for t in tags
                if Path.Geom.isRoughly(
                    0, Part.Vertex(t.originAt(self.minZ)).distToShape(edge)[0], 0.1
                )
            ]
            for t in sorted(
                ts,
                key=lambda t, edge=edge: (
                    t.originAt(self.minZ) - edge.valueAt(edge.FirstParameter)
                ).Length,
            ):
                tags.remove(t)
                ordered.append(t)
        # disable all tags that are not on the base wire.
        for tag in tags:
            Path.Log.info(
                "Tag #%d (%.2f, %.2f, %.2f) not on base wire - disabling\n"
                % (len(ordered), tag.x, tag.y, self.minZ)
            )
            tag.enabled = False
            ordered.append(tag)
        return ordered

    def pointIsOnPath(self, p):
        v = Part.Vertex(self.pointAtBottom(p))
        Path.Log.debug("pt = (%f, %f, %f)" % (v.X, v.Y, v.Z))
        for e in self.bottomEdges:
            indent = "{} ".format(e.distToShape(v)[0])
            debugEdge(e, indent, True)
            if Path.Geom.isRoughly(0.0, v.distToShape(e)[0], 0.1):
                return True
        return False

    def pointAtBottom(self, p):
        return FreeCAD.Vector(p.x, p.y, self.minZ)


class ObjectTagDressup:
    def __init__(self, obj, base):

        obj.addProperty(
            "App::PropertyLink",
            "Base",
            "Base",
            QT_TRANSLATE_NOOP("App::Property", "The base path to modify"),
        )
        obj.addProperty(
            "App::PropertyLength",
            "Width",
            "Tag",
            QT_TRANSLATE_NOOP("App::Property", "Width of tags."),
        )
        obj.addProperty(
            "App::PropertyLength",
            "Height",
            "Tag",
            QT_TRANSLATE_NOOP("App::Property", "Height of tags."),
        )
        obj.addProperty(
            "App::PropertyAngle",
            "Angle",
            "Tag",
            QT_TRANSLATE_NOOP("App::Property", "Angle of tag plunge and ascent."),
        )
        obj.addProperty(
            "App::PropertyLength",
            "Radius",
            "Tag",
            QT_TRANSLATE_NOOP("App::Property", "Radius of the fillet for the tag."),
        )
        obj.addProperty(
            "App::PropertyVectorList",
            "Positions",
            "Tag",
            QT_TRANSLATE_NOOP("App::Property", "Locations of inserted holding tags"),
        )
        obj.addProperty(
            "App::PropertyIntegerList",
            "Disabled",
            "Tag",
            QT_TRANSLATE_NOOP("App::Property", "IDs of disabled holding tags"),
        )

        self.obj = obj
        self.solids = []
        self.tags = []
        self.pathData = None
        self.toolRadius = None
        self.mappers = []

        obj.Proxy = self
        obj.Base = base

    def dumps(self):
        return None

    def loads(self, state):
        self.obj = state
        self.solids = []
        self.tags = []
        self.pathData = None
        self.toolRadius = None
        self.mappers = []
        return None

    def onChanged(self, obj, prop):
        if prop == "Path" and obj.ViewObject:
            obj.ViewObject.signalChangeIcon()

    def onDocumentRestored(self, obj):
        self.obj = obj

    def supportsTagGeneration(self, obj):
        if not self.pathData:
            self.setup(obj)
        return self.pathData.supportsTagGeneration()

    def generateTags(self, obj, count):
        if self.supportsTagGeneration(obj):
            if self.pathData:
                self.tags = self.pathData.generateTags(
                    obj,
                    count,
                    obj.Width.Value,
                    obj.Height.Value,
                    obj.Angle,
                    obj.Radius.Value,
                    None,
                )
                obj.Positions = [tag.originAt(self.pathData.minZ) for tag in self.tags]
                obj.Disabled = []
                return False
            else:
                self.setup(obj, count)
                self.execute(obj)
                return True
        else:
            self.tags = []
            obj.Positions = []
            obj.Disabled = []
            return False

    def copyTags(self, obj, fromObj):
        obj.Width = fromObj.Width
        obj.Height = fromObj.Height
        obj.Angle = fromObj.Angle
        obj.Radius = fromObj.Radius

        self.tags = self.pathData.copyTags(
            obj, fromObj, obj.Width.Value, obj.Height.Value, obj.Angle, obj.Radius.Value
        )
        obj.Positions = [tag.originAt(self.pathData.minZ) for tag in self.tags]
        obj.Disabled = []
        return False

    def isValidTagStartIntersection(self, edge, i):
        if Path.Geom.pointsCoincide(i, edge.valueAt(edge.LastParameter)):
            return False
        p1 = edge.valueAt(edge.FirstParameter)
        p2 = edge.valueAt(edge.LastParameter)
        if Path.Geom.pointsCoincide(Path.Geom.xy(p1), Path.Geom.xy(p2)):
            # if this vertical goes up, it can't be the start of a tag intersection
            if p1.z < p2.z:
                return False
        return True

    def createPath(self, obj, pathData, tags):
        Path.Log.track()
        commands = []
        lastEdge = 0
        lastTag = 0
        t = 0
        edge = None

        self.mappers = []
        mapper = None

        job = PathUtils.findParentJob(obj)
        tol = job.GeometryTolerance.Value
        tc = PathDressup.toolController(obj.Base)
        horizFeed = tc.HorizFeed.Value
        vertFeed = tc.VertFeed.Value
        horizRapid = tc.HorizRapid.Value
        vertRapid = tc.VertRapid.Value

        while edge or lastEdge < len(pathData.edges):
            Path.Log.debug("------- lastEdge = %d/%d.%d/%d" % (lastEdge, lastTag, t, len(tags)))
            if not edge:
                edge = pathData.edges[lastEdge]
                debugEdge(edge, "=======  new edge: %d/%d" % (lastEdge, len(pathData.edges)))
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
                if i and self.isValidTagStartIntersection(edge, i):
                    mapper = MapWireToTag(
                        edge,
                        tags[tIndex],
                        i,
                        pathData.maxZ,
                        hSpeed=horizFeed,
                        vSpeed=vertFeed,
                        tolerance=tol,
                    )
                    self.mappers.append(mapper)
                    edge = mapper.tail

            if not mapper and t >= len(tags):
                # gone through all tags, consume edge and move on
                if edge:
                    debugEdge(edge, "++++++++")
                    if pathData.rapid.isRapid(edge):
                        v = edge.Vertexes[1]
                        if (
                            not commands
                            and Path.Geom.isRoughly(0, v.X)
                            and Path.Geom.isRoughly(0, v.Y)
                            and not Path.Geom.isRoughly(0, v.Z)
                        ):
                            # The very first move is just to move to ClearanceHeight
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

        return Path.Path(commands)

    def problems(self):
        return list([m for m in self.mappers if m.haveProblem])

    def createTagsPositionDisabled(self, obj, positionsIn, disabledIn):
        rawTags = []
        for i, pos in enumerate(positionsIn):
            tag = Tag(
                i,
                pos.x,
                pos.y,
                obj.Width.Value,
                obj.Height.Value,
                obj.Angle,
                obj.Radius,
                i not in disabledIn,
            )
            tag.createSolidsAt(self.pathData.minZ, self.toolRadius)
            rawTags.append(tag)
        # disable all tags that intersect with their previous tag
        prev = None
        tags = []
        positions = []
        disabled = []
        for i, tag in enumerate(self.pathData.sortedTags(rawTags)):
            if tag.enabled:
                if prev:
                    if prev.solid.common(tag.solid).Faces:
                        Path.Log.info("Tag #%d intersects with previous tag - disabling\n" % i)
                        Path.Log.debug("this tag = %d [%s]" % (i, tag.solid.BoundBox))
                        tag.enabled = False
                elif self.pathData.edges:
                    e = self.pathData.edges[0]
                    p0 = e.valueAt(e.FirstParameter)
                    p1 = e.valueAt(e.LastParameter)
                    if tag.solid.isInside(p0, Path.Geom.Tolerance, True) or tag.solid.isInside(
                        p1, Path.Geom.Tolerance, True
                    ):
                        Path.Log.info("Tag #%d intersects with starting point - disabling\n" % i)
                        tag.enabled = False

            if tag.enabled:
                prev = tag
                Path.Log.debug("previousTag = %d [%s]" % (i, prev))
            else:
                disabled.append(i)
            tag.nr = i  # assign final nr
            tags.append(tag)
            positions.append(tag.originAt(self.pathData.minZ))
        return (tags, positions, disabled)

    def execute(self, obj):
        # import cProfile
        # pr = cProfile.Profile()
        # pr.enable()
        self.doExecute(obj)
        # pr.disable()
        # pr.print_stats()

    def doExecute(self, obj):
        if not obj.Base:
            return
        if not obj.Base.isDerivedFrom("Path::Feature"):
            return
        if not obj.Base.Path:
            return
        if not obj.Base.Path.Commands:
            return

        pathData = self.setup(obj)
        if not pathData:
            Path.Log.debug("execute - no pathData")
            return

        self.tags = []
        if hasattr(obj, "Positions"):
            self.tags, positions, disabled = self.createTagsPositionDisabled(
                obj, obj.Positions, obj.Disabled
            )
            if obj.Disabled != disabled:
                Path.Log.debug("Updating properties.... %s vs. %s" % (obj.Disabled, disabled))
                obj.Positions = positions
                obj.Disabled = disabled

        if not self.tags:
            Path.Log.debug("execute - no tags")
            obj.Path = PathUtils.getPathWithPlacement(obj.Base)
            return

        try:
            self.processTags(obj)
        except Exception as e:
            Path.Log.error("processing tags failed clearing all tags… '%s'" % (e.args[0]))
            obj.Path = PathUtils.getPathWithPlacement(obj.Base)

        # update disabled in case there are some additional ones
        disabled = copy.copy(self.obj.Disabled)
        solids = []
        for tag in self.tags:
            solids.append(tag.solid)
            if not tag.enabled and tag.nr not in disabled:
                disabled.append(tag.nr)
        self.solids = solids
        if obj.Disabled != disabled:
            obj.Disabled = disabled

    @waiting_effects
    def processTags(self, obj):
        tagID = 0
        if Path.Log.getLevel(Path.Log.thisModule()) == Path.Log.Level.DEBUG:
            for tag in self.tags:
                tagID += 1
                if tag.enabled:
                    Path.Log.debug("x=%s, y=%s, z=%s" % (tag.x, tag.y, self.pathData.minZ))
                    # debugMarker(FreeCAD.Vector(tag.x, tag.y, self.pathData.minZ), "tag-%02d" % tagID , (1.0, 0.0, 1.0), 0.5)
                    # if not Path.Geom.isRoughly(90, tag.angle):
                    #    debugCone(tag.originAt(self.pathData.minZ), tag.r1, tag.r2, tag.actualHeight, "tag-%02d" % tagID)
                    # else:
                    #    debugCylinder(tag.originAt(self.pathData.minZ), tag.fullWidth()/2, tag.actualHeight, "tag-%02d" % tagID)

        obj.Path = self.createPath(obj, self.pathData, self.tags)

    def setup(self, obj, generate=False):
        Path.Log.debug("setup")
        self.obj = obj
        try:
            pathData = PathData(obj)
        except ValueError:
            Path.Log.error(
                translate(
                    "CAM_DressupTag",
                    "Cannot insert holding tags for this path - select a profile path",
                )
                + "\n"
            )
            return None

        self.toolRadius = float(PathDressup.toolController(obj.Base).Tool.Diameter) / 2
        self.pathData = pathData
        if generate:
            obj.Height = self.pathData.defaultTagHeight()
            obj.Width = self.pathData.defaultTagWidth()
            obj.Angle = self.pathData.defaultTagAngle()
            obj.Radius = self.pathData.defaultTagRadius()
            count = HoldingTagPreferences.defaultCount()
            self.generateTags(obj, count)
        return self.pathData

    def setXyEnabled(self, triples):
        Path.Log.track()
        if not self.pathData:
            self.setup(self.obj)
        positions = []
        disabled = []
        for i, (x, y, enabled) in enumerate(triples):
            # print("%d: (%.2f, %.2f) %d" % (i, x, y, enabled))
            positions.append(FreeCAD.Vector(x, y, 0))
            if not enabled:
                disabled.append(i)
        (
            self.tags,
            self.obj.Positions,
            self.obj.Disabled,
        ) = self.createTagsPositionDisabled(self.obj, positions, disabled)
        self.processTags(self.obj)

    def pointIsOnPath(self, obj, point):
        if not self.pathData:
            self.setup(obj)
        return self.pathData.pointIsOnPath(point)

    def pointAtBottom(self, obj, point):
        if not self.pathData:
            self.setup(obj)
        return self.pathData.pointAtBottom(point)


def Create(baseObject, name="DressupTag"):
    """
    Create(basePath, name='DressupTag') … create tag dressup object for the given base path.
    """
    if not baseObject.isDerivedFrom("Path::Feature"):
        Path.Log.error(translate("CAM_DressupTag", "The selected object is not a path") + "\n")
        return None

    if baseObject.isDerivedFrom("Path::FeatureCompoundPython"):
        Path.Log.error(translate("CAM_DressupTag", "Select a profile object"))
        return None

    obj = FreeCAD.ActiveDocument.addObject("Path::FeaturePython", name)
    dbo = ObjectTagDressup(obj, baseObject)
    job = PathUtils.findParentJob(baseObject)
    job.Proxy.addOperation(obj, baseObject)
    dbo.setup(obj, True)
    return obj


Path.Log.notice("Loading CAM_DressupTag… done\n")

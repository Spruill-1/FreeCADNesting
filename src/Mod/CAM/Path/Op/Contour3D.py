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

"""Bevel-oriented 3D contour operation.

This first implementation intentionally reuses the Waterline backend so we can
deliver a dedicated sloped-face finishing operation with minimal risk. Future
revisions can diverge into a more bevel-focused toolpath engine while keeping
the same user-facing command and operation object.
"""

import FreeCAD
import Path
import Path.Geom as PathGeom
import Path.Op.Base as PathOp
import PathScripts.PathUtils as PathUtils
import math

from lazy_loader.lazy_loader import LazyLoader

Part = LazyLoader("Part", globals(), "Part")

HAS_OCL = True
try:
    import Path.Op.Waterline as PathWaterline
except ImportError:
    HAS_OCL = False
    PathWaterline = None

__title__ = "CAM 3D Contour Operation"
__author__ = "FreeCAD contributors"
__url__ = "https://www.freecad.org"
__doc__ = "Class and implementation of a 3D Contour / bevel finishing operation."

translate = FreeCAD.Qt.translate

# ``Contour3D`` intentionally exposes only the Waterline algorithm variant
# that still respects selected-face processing.  Other Waterline modes can
# ignore face selections or route through code paths that are better suited to
# whole-model machining, which is not what this bevel-focused operation wants.
SUPPORTED_ALGORITHMS = ["OCL Adaptive"]

if False:
    Path.Log.setLevel(Path.Log.Level.DEBUG, Path.Log.thisModule())
    Path.Log.trackModule(Path.Log.thisModule())
else:
    Path.Log.setLevel(Path.Log.Level.INFO, Path.Log.thisModule())


def _contour_property_definitions():
    """Return 3D contour specific property definitions."""
    return [
        (
            "App::PropertyEnumeration",
            "PassMode",
            "3D Contour",
            FreeCAD.Qt.translate(
                "App::Property",
                "Choose whether the operation emits slices only or adds a dedicated finish pass.",
            ),
        ),
        (
            "App::PropertyBool",
            "FinishPass",
            "3D Contour",
            FreeCAD.Qt.translate(
                "App::Property",
                "Emit an additional finish pass after the main slice passes.",
            ),
        ),
        (
            "App::PropertyBool",
            "SpringPass",
            "3D Contour",
            FreeCAD.Qt.translate(
                "App::Property",
                "Repeat the final finishing pass at the same depth.",
            ),
        ),
        (
            "App::PropertyBool",
            "BoundaryCleanupPass",
            "3D Contour",
            FreeCAD.Qt.translate(
                "App::Property",
                "Add a final boundary cleanup pass using the longest deepest contour loop.",
            ),
        ),
        (
            "App::PropertyBool",
            "KeepToolDown",
            "3D Contour",
            FreeCAD.Qt.translate(
                "App::Property",
                "Prefer linking nearby slice loops without retracting when safe.",
            ),
        ),
        (
            "App::PropertyDistance",
            "MinStayDownDistance",
            "3D Contour",
            FreeCAD.Qt.translate(
                "App::Property",
                "Minimum direct distance before stay-down linking is considered worthwhile.",
            ),
        ),
        (
            "App::PropertyFloat",
            "MinBevelTilt",
            "3D Contour",
            FreeCAD.Qt.translate(
                "App::Property",
                "Minimum face tilt, in degrees from horizontal, for the face to be treated as a bevel candidate.",
            ),
        ),
    ]


def _contour_property_enumerations(dataType="data"):
    """Return contour-specific enumeration values."""
    enums = {
        "Algorithm": [
            (translate("CAM_Contour3D", "OCL Adaptive"), "OCL Adaptive"),
        ],
        "PassMode": [
            (translate("CAM_Contour3D", "Slices only"), "SlicesOnly"),
            (translate("CAM_Contour3D", "Slices plus finish"), "SlicesPlusFinish"),
        ],
    }

    if dataType == "raw":
        return enums

    data = []
    idx = 0 if dataType == "translated" else 1
    for key in enums:
        data.append((key, [tup[idx] for tup in enums[key]]))
    return data


def _ensure_contour_properties(obj):
    """Add contour-specific properties to *obj* when missing."""
    added = []
    for prop_type, name, group, tooltip in _contour_property_definitions():
        if not hasattr(obj, name):
            obj.addProperty(prop_type, name, group, tooltip)
            added.append(name)

    if added:
        # Enumeration properties must receive their option list immediately
        # after creation.  Without this, subsequent assignments can fail
        # because the property has no valid literals yet.
        raw_enums = _contour_property_enumerations(dataType="raw")
        for name in added:
            if name in raw_enums:
                setattr(obj, name, [value for _label, value in raw_enums[name]])


def _apply_contour_defaults(obj):
    """Apply contour-specific defaults."""
    if hasattr(obj, "Algorithm"):
        obj.Algorithm = "OCL Adaptive"
    if hasattr(obj, "PassMode"):
        obj.PassMode = "SlicesOnly"
    if hasattr(obj, "FinishPass"):
        obj.FinishPass = False
    if hasattr(obj, "SpringPass"):
        obj.SpringPass = False
    if hasattr(obj, "BoundaryCleanupPass"):
        obj.BoundaryCleanupPass = True
    if hasattr(obj, "KeepToolDown"):
        obj.KeepToolDown = False
    if hasattr(obj, "MinStayDownDistance"):
        obj.MinStayDownDistance = 0.0
    if hasattr(obj, "MinBevelTilt"):
        obj.MinBevelTilt = 5.0


def _sync_contour_modes(obj, prop=None):
    """Keep contour-mode properties internally consistent."""
    if hasattr(obj, "Algorithm") and obj.Algorithm not in SUPPORTED_ALGORITHMS:
        # Coerce restored documents or scripted edits back onto the one
        # supported algorithm so the operation always runs through the
        # face-aware execution path.
        obj.Algorithm = SUPPORTED_ALGORITHMS[0]

    if hasattr(obj, "PassMode") and hasattr(obj, "FinishPass"):
        # ``PassMode`` is the user-facing high-level switch, while
        # ``FinishPass`` is the low-level boolean consumed by execution.
        # Keep them synchronized in both directions.
        if prop == "PassMode":
            finish_enabled = obj.PassMode == "SlicesPlusFinish"
            if obj.FinishPass != finish_enabled:
                obj.FinishPass = finish_enabled
        elif prop == "FinishPass":
            pass_mode = "SlicesPlusFinish" if obj.FinishPass else "SlicesOnly"
            if obj.PassMode != pass_mode:
                obj.PassMode = pass_mode

    if hasattr(obj, "FinishPass") and hasattr(obj, "SpringPass"):
        if not obj.FinishPass and obj.SpringPass:
            obj.SpringPass = False

    if hasattr(obj, "FinishPass") and hasattr(obj, "BoundaryCleanupPass"):
        obj.setEditorMode("BoundaryCleanupPass", 0 if obj.FinishPass else 2)

    if hasattr(obj, "MinStayDownDistance") and hasattr(obj, "KeepToolDown"):
        obj.setEditorMode("MinStayDownDistance", 0 if obj.KeepToolDown else 2)

    if hasattr(obj, "SpringPass") and hasattr(obj, "FinishPass"):
        obj.setEditorMode("SpringPass", 0 if obj.FinishPass else 2)


def _face_center_parameters(face):
    """Return a stable `(u, v)` sample point near the center of *face*."""
    try:
        u0, u1, v0, v1 = face.ParameterRange
        return ((u0 + u1) / 2.0, (v0 + v1) / 2.0)
    except Exception:
        return (0.0, 0.0)


def _face_normal(face):
    """Return an approximate normal vector for *face*, or `None`."""
    try:
        u, v = _face_center_parameters(face)
        normal = face.normalAt(u, v)
        if normal.Length <= 0:
            return None
        normal.normalize()
        return normal
    except Exception:
        try:
            surface = face.Surface
            axis = getattr(surface, "Axis", None)
            if axis and axis.Length > 0:
                axis = FreeCAD.Vector(axis)
                axis.normalize()
                return axis
        except Exception:
            pass
    return None


def _is_bevel_candidate_face(face, min_tilt_deg=5.0):
    """Return `True` if *face* is not approximately horizontal.

    This accepts both sloped and vertical faces; it rejects top/bottom planar
    faces which are poor candidates for a bevel/3D contour finishing op.
    """
    normal = _face_normal(face)
    if normal is None:
        return False

    z_alignment = abs(normal.z)
    z_alignment = max(-1.0, min(1.0, z_alignment))
    tilt_deg = math.degrees(math.acos(z_alignment))
    return tilt_deg >= float(min_tilt_deg)


def _selected_faces(obj):
    """Return list of selected faces referenced by `obj.Base`."""
    faces = []
    if not hasattr(obj, "Base") or not obj.Base:
        return faces

    for base, sublist in obj.Base:
        for sub in sublist:
            if isinstance(sub, str) and sub.startswith("Face"):
                try:
                    # The operation only accepts face selections, but keep the
                    # lookup defensive so broken/restored references degrade
                    # gracefully instead of aborting the whole op.
                    faces.append(base.Shape.getElement(sub))
                except Exception:
                    continue
    return faces


def _normalize_base_to_job_models(obj):
    """Map base selections to job model resource clones when available."""
    if not hasattr(obj, "Base") or not obj.Base:
        return

    job = PathUtils.findParentJob(obj)
    if not job or not hasattr(job, "Proxy"):
        return

    normalized = []
    changed = False
    for base, sublist in obj.Base:
        # GUI selections typically point at the job's resource clones, but
        # scripted tests and programmatic callers often pass the original body.
        # Waterline internals expect the job-model objects, so normalize here.
        mapped = job.Proxy.resourceClone(job, base)
        if mapped is not None and mapped is not base:
            base = mapped
            changed = True
        normalized.append((base, sublist))

    if changed:
        obj.Base = normalized


def _make_face_compound(faces):
    """Return a compound shape containing *faces*."""
    valid_faces = [face for face in faces if face is not None]
    if not valid_faces:
        return None
    try:
        # Prefer a shell because several downstream FreeCAD operations treat a
        # shell as more topologically meaningful than an arbitrary compound of
        # disconnected faces.
        shell = Part.makeShell(valid_faces)
        if shell and getattr(shell, "Faces", None):
            return shell
    except Exception:
        pass
    return Part.makeCompound(valid_faces)


def _command_params(cmd):
    """Return a mutable copy of the command parameters."""
    params = getattr(cmd, "Parameters", None) or {}
    return dict(params)


def _clone_command(cmd):
    """Clone a Path command."""
    return Path.Command(cmd.Name, _command_params(cmd))


def _clone_commands(commands):
    return [_clone_command(cmd) for cmd in commands]


def _motion_name(cmd):
    return str(getattr(cmd, "Name", "")).upper()


def _is_rapid(cmd):
    return _motion_name(cmd) in ("G0", "G00")


def _is_feed_motion(cmd):
    return _motion_name(cmd) in ("G1", "G01", "G2", "G02", "G3", "G03")


def _extract_cut_sections(commands):
    """Extract motion sections from a command list.

    Sections are split by rapid moves and retain their original command list,
    minimum Z depth, and approximate XY travel length.
    """
    sections = []
    section = []
    section_has_cut = False
    section_min_z = None
    section_xy_length = 0.0
    x = y = z = None

    def _commit_section():
        nonlocal section, section_has_cut, section_min_z, section_xy_length
        if section and section_has_cut:
            # Store a cloned copy so later post-processing can freely mutate
            # the result without changing the original command list.
            sections.append(
                {
                    "commands": _clone_commands(section),
                    "min_z": section_min_z,
                    "xy_length": section_xy_length,
                }
            )
        section = []
        section_has_cut = False
        section_min_z = None
        section_xy_length = 0.0

    for cmd in commands:
        name = _motion_name(cmd)
        params = _command_params(cmd)
        new_x = params.get("X", x)
        new_y = params.get("Y", y)
        new_z = params.get("Z", z)

        if _is_rapid(cmd):
            # Rapids delimit cutting sections.  Keep a leading rapid if it is
            # part of the section entry, but commit the previous section first.
            _commit_section()
            if params:
                section = [_clone_command(cmd)]
        elif _is_feed_motion(cmd):
            if not section:
                section = []
            section.append(_clone_command(cmd))

            if new_z is not None:
                # Track the minimum reached Z rather than only the final Z so
                # arc/plunge combinations still report their true cut depth.
                section_min_z = new_z if section_min_z is None else min(section_min_z, new_z)

            if ("X" in params or "Y" in params) and x is not None and y is not None:
                dx = float(new_x) - float(x)
                dy = float(new_y) - float(y)
                section_xy_length += math.hypot(dx, dy)
                section_has_cut = True
            elif "X" in params or "Y" in params:
                section_has_cut = True

        x, y, z = new_x, new_y, new_z

    _commit_section()
    return sections


def _deepest_sections(sections, tol=1e-7):
    """Return the subset of sections at the deepest cut depth."""
    if not sections:
        return []

    depths = [sec["min_z"] for sec in sections if sec["min_z"] is not None]
    if not depths:
        return []

    deepest = min(depths)
    return [sec for sec in sections if sec["min_z"] is not None and abs(sec["min_z"] - deepest) <= tol]


def _append_labeled_sections(commands, label, sections):
    """Append a labeled set of cloned sections."""
    if not sections:
        return
    commands.append(Path.Command("N ({})".format(label), {}))
    for section in sections:
        commands.extend(_clone_commands(section["commands"]))


def _append_labeled_commands(commands, label, extra_commands):
    """Append a labeled block of commands."""
    if not extra_commands:
        return
    commands.append(Path.Command("N ({})".format(label), {}))
    commands.extend(_clone_commands(extra_commands))


def _has_xy(cmd):
    params = _command_params(cmd)
    return "X" in params or "Y" in params


def _has_z(cmd):
    return "Z" in _command_params(cmd)


def _is_motion(cmd):
    return _is_rapid(cmd) or _is_feed_motion(cmd)


def _transition_feed_rate(commands, start_index, fallback=0.0):
    """Return a suitable XY feed rate for a stay-down link."""
    for cmd in commands[start_index:]:
        if not _is_feed_motion(cmd):
            continue
        params = _command_params(cmd)
        if _has_xy(cmd) and "F" in params:
            return float(params["F"])
    return float(fallback)


def _optimize_keep_tool_down(commands, min_distance, z_tol=1e-6):
    """Replace short retract-and-reposition moves with feed links when safe."""
    if min_distance is None or float(min_distance) <= 0.0:
        return _clone_commands(commands)

    result = []
    current = FreeCAD.Vector()
    have_current = False
    i = 0
    while i < len(commands):
        if i + 2 < len(commands):
            up = commands[i]
            over = commands[i + 1]
            plunge = commands[i + 2]
            if (
                _is_rapid(up)
                and _has_z(up)
                and not _has_xy(up)
                and _is_rapid(over)
                and _has_xy(over)
                and _is_feed_motion(plunge)
                and _has_z(plunge)
                and not _has_xy(plunge)
                and have_current
            ):
                # Pattern matched:
                #   rapid retract -> rapid XY move -> plunge back to same Z
                #
                # If the XY gap is very short and the plunge returns to the same
                # working depth, emit a direct feed move instead.  That reduces
                # unnecessary retracts while staying conservative.
                over_end = PathGeom.commandEndPoint(over, current)
                plunge_end = PathGeom.commandEndPoint(plunge, over_end)
                xy_gap = math.hypot(over_end.x - current.x, over_end.y - current.y)
                same_depth = abs(float(plunge_end.z) - float(current.z)) <= z_tol
                if same_depth and xy_gap <= float(min_distance) + z_tol:
                    result.append(Path.Command("N (3D Contour stay-down link)", {}))
                    if xy_gap > z_tol:
                        params = {"X": over_end.x, "Y": over_end.y}
                        feed = _transition_feed_rate(commands, i + 3, plunge.Parameters.get("F", 0.0))
                        if feed > 0:
                            params["F"] = feed
                        result.append(Path.Command("G1", params))
                        current = PathGeom.commandEndPoint(result[-1], current)
                    i += 3
                    continue

        result.append(_clone_command(commands[i]))
        if _is_motion(commands[i]):
            current = PathGeom.commandEndPoint(commands[i], current)
            have_current = True
        i += 1
    return result


def _edge_midpoint(edge):
    """Return an approximate midpoint on *edge*."""
    try:
        return edge.valueAt((edge.FirstParameter + edge.LastParameter) / 2.0)
    except Exception:
        pts = edge.discretize(3)
        return pts[len(pts) // 2] if pts else FreeCAD.Vector()


def _edge_key(edge, digits=6):
    """Return a stable key for de-duplicating coincident edges."""
    pts = []
    if getattr(edge, "Vertexes", None):
        pts = [v.Point for v in edge.Vertexes]
    if len(pts) < 2:
        pts = edge.discretize(2)
    mid = _edge_midpoint(edge)
    data = [
        (round(p.x, digits), round(p.y, digits), round(p.z, digits)) for p in pts[:2]
    ]
    data.sort()
    data.append((round(mid.x, digits), round(mid.y, digits), round(mid.z, digits)))
    return tuple(data)


def _dedupe_edges(edges):
    """Remove duplicate coincident edges while preserving order."""
    seen = set()
    unique = []
    for edge in edges:
        key = _edge_key(edge)
        if key in seen:
            continue
        seen.add(key)
        unique.append(edge)
    return unique


def _depth_section_edges_from_faces(faces, depth, tol=1e-6):
    """Return exact section edges from selected faces at *depth*."""
    valid_faces = [face for face in faces if face is not None]
    if not valid_faces:
        return []

    # Work on a copied compound so any sectioning artifacts stay isolated from
    # the original selection geometry.
    compound = Part.makeCompound([face.copy() for face in valid_faces])
    bb = compound.BoundBox
    margin = max(bb.DiagonalLength * 0.1, 1.0)
    xmin = bb.XMin - margin
    xmax = bb.XMax + margin
    ymin = bb.YMin - margin
    ymax = bb.YMax + margin
    poly = Part.makePolygon(
        [
            FreeCAD.Vector(xmin, ymin, depth),
            FreeCAD.Vector(xmax, ymin, depth),
            FreeCAD.Vector(xmax, ymax, depth),
            FreeCAD.Vector(xmin, ymax, depth),
            FreeCAD.Vector(xmin, ymin, depth),
        ]
    )
    # Use an oversized planar face so the intersection is clipped only by the
    # bevel geometry, not by the section plane bounds.
    plane = Part.Face(poly)

    try:
        section = compound.section(plane)
    except Exception:
        section = None

    edges = []
    if section and getattr(section, "Edges", None):
        for edge in section.Edges:
            if getattr(edge, "Length", 0.0) > tol:
                edges.append(edge)
    return _dedupe_edges(edges)


def _boundary_edges_at_depth(faces, depth, tol=1e-5):
    """Return exact or fallback boundary edges at *depth*."""
    edges = _depth_section_edges_from_faces(faces, depth, tol=tol)
    if edges:
        return edges

    # Fallback path for cases where exact face/plane sectioning yields no edges
    # (for example because of model tolerances or degenerate intersection
    # topology).  Here we look for source edges that already lie on the target
    # depth.
    for face in faces:
        for wire in getattr(face, "Wires", []):
            for edge in wire.Edges:
                try:
                    pts = [v.Point for v in edge.Vertexes]
                except Exception:
                    pts = edge.discretize(3)
                if not pts:
                    continue
                zmin = min(p.z for p in pts)
                zmax = max(p.z for p in pts)
                mid = _edge_midpoint(edge)
                if zmin - tol <= depth <= zmax + tol and abs(mid.z - depth) <= tol * 10.0:
                    edges.append(edge)
    return _dedupe_edges(edges)


def _edges_to_wires(edges):
    """Convert edge list to ordered wires where possible."""
    if not edges:
        return []
    wires = []
    for group in Part.sortEdges(edges):
        try:
            wires.append(Part.Wire(group))
        except Exception:
            continue
    return wires


def _ordered_wire_edges(wire):
    """Return wire edges paired with flip flags for continuous traversal."""
    if not getattr(wire, "Edges", None):
        return []

    ordered = []
    edges = list(wire.Edges)
    first = edges[0]
    ordered.append((first, False))
    current = first.valueAt(first.LastParameter)

    for edge in edges[1:]:
        start = edge.valueAt(edge.FirstParameter)
        end = edge.valueAt(edge.LastParameter)
        if PathGeom.pointsCoincide(start, current):
            ordered.append((edge, False))
            current = end
        elif PathGeom.pointsCoincide(end, current):
            ordered.append((edge, True))
            current = start
        else:
            ordered.append((edge, False))
            current = end
    return ordered


def _wire_start_point(edge, flip):
    """Return the start point for an oriented edge."""
    return edge.valueAt(edge.LastParameter if flip else edge.FirstParameter)


def _build_boundary_cleanup_commands(faces, depth, obj, proxy):
    """Build exact boundary cleanup commands from selected faces at *depth*."""
    edges = _boundary_edges_at_depth(faces, depth)
    wires = _edges_to_wires(edges)
    if not wires:
        return []

    commands = []
    first_wire = True
    for wire in wires:
        oriented = _ordered_wire_edges(wire)
        if not oriented:
            continue
        start = _wire_start_point(*oriented[0])

        # First wire enters from clearance; subsequent disconnected wires only
        # rise to safe height to avoid unnecessary long retracts.
        commands.append(
            Path.Command(
                "G0",
                {"Z": obj.ClearanceHeight.Value if first_wire else obj.SafeHeight.Value, "F": proxy.vertRapid},
            )
        )
        commands.append(Path.Command("G0", {"X": start.x, "Y": start.y, "F": proxy.horizRapid}))
        commands.append(Path.Command("G1", {"Z": start.z, "F": proxy.vertFeed}))

        for edge, flip in oriented:
            commands.extend(
                PathGeom.cmdsForEdge(
                    edge,
                    flip=flip,
                    hSpeed=proxy.horizFeed,
                    vSpeed=proxy.vertFeed,
                )
            )
        first_wire = False
    return commands


def _slice_depths_for_faces(faces, obj, tol=1e-6):
    """Return descending slice depths spanning the selected faces."""
    valid_faces = [face for face in faces if face is not None]
    if not valid_faces:
        return []

    z_max = max(face.BoundBox.ZMax for face in valid_faces)
    z_min = min(face.BoundBox.ZMin for face in valid_faces)
    # Clamp the requested depths to the actual bevel span so the operation does
    # not waste passes above the selected faces or below their deepest extent.
    start_depth = min(float(obj.StartDepth.Value), z_max)
    final_depth = max(float(obj.FinalDepth.Value), z_min)
    if start_depth < final_depth:
        start_depth, final_depth = z_max, z_min

    # ``StepDown`` defines the constant-Z slice spacing.  When it is zero or
    # effectively disabled, collapse to a single pass spanning the full face.
    step_down = abs(float(getattr(obj, "StepDown", 0.0).Value))
    if step_down <= tol:
        step_down = max((start_depth - final_depth), 0.0)

    depths = []
    depth = start_depth
    depths.append(depth)
    if step_down > tol:
        while depth - step_down > final_depth + tol:
            depth -= step_down
            depths.append(depth)
    if not depths or abs(depths[-1] - final_depth) > tol:
        depths.append(final_depth)
    return depths


def _build_slice_commands(faces, obj, proxy):
    """Build direct slice commands from selected-face section geometry."""
    commands = []
    first_depth = True
    for depth in _slice_depths_for_faces(faces, obj):
        section_commands = _build_boundary_cleanup_commands(faces, depth, obj, proxy)
        if not section_commands:
            continue
        if not first_depth and section_commands:
            first = section_commands[0]
            if _motion_name(first) in ("G0", "G00"):
                # After the first slice, only return to safe height between
                # slices.  That keeps the direct slicing path efficient while
                # still producing conservative entry moves.
                params = _command_params(first)
                params["Z"] = obj.SafeHeight.Value
                section_commands[0] = Path.Command(first.Name, params)
        commands.extend(section_commands)
        first_depth = False
    return commands


def _deepest_depth_from_commands(commands):
    """Return the deepest cut depth seen in *commands*."""
    deepest = _deepest_sections(_extract_cut_sections(commands))
    if not deepest:
        return None
    return min(sec["min_z"] for sec in deepest if sec["min_z"] is not None)


def _post_process_contour_commands(commands, add_finish_pass=False,
                                   add_boundary_cleanup=False,
                                   add_spring_pass=False,
                                   boundary_cleanup_commands=None,
                                   keep_tool_down=False,
                                   min_stay_down_distance=0.0):
    """Return a post-processed command list for contour finishing passes."""
    output = _clone_commands(commands)
    sections = _extract_cut_sections(commands)
    deepest = _deepest_sections(sections)
    if not deepest:
        return output

    boundary_section = max(deepest, key=lambda sec: sec["xy_length"])

    if add_finish_pass:
        _append_labeled_sections(output, "3D Contour finish pass", deepest)

    if add_boundary_cleanup:
        if boundary_cleanup_commands:
            _append_labeled_commands(output, "3D Contour boundary cleanup", boundary_cleanup_commands)
        else:
            _append_labeled_sections(output, "3D Contour boundary cleanup", [boundary_section])

    if add_spring_pass:
        if add_boundary_cleanup and boundary_cleanup_commands:
            _append_labeled_commands(output, "3D Contour spring pass", boundary_cleanup_commands)
        else:
            spring_source = [boundary_section] if add_boundary_cleanup else deepest
            _append_labeled_sections(output, "3D Contour spring pass", spring_source)

    if keep_tool_down:
        output = _optimize_keep_tool_down(output, min_stay_down_distance)

    return output


if HAS_OCL:

    class ObjectContour3D(PathWaterline.ObjectWaterline):
        """Proxy object for a bevel-oriented 3D contour operation."""

        def initOperation(self, obj):
            super().initOperation(obj)
            _ensure_contour_properties(obj)
            _sync_contour_modes(obj)

        def opPropertyDefinitions(self):
            return PathWaterline.ObjectWaterline.opPropertyDefinitions(self) + _contour_property_definitions()

        @classmethod
        def propertyEnumerations(cls, dataType="data"):
            enums = PathWaterline.ObjectWaterline.propertyEnumerations(dataType="raw")
            enums.update(_contour_property_enumerations(dataType="raw"))

            if dataType == "raw":
                return enums

            data = []
            idx = 0 if dataType == "translated" else 1
            for key in enums:
                data.append((key, [tup[idx] for tup in enums[key]]))
            return data

        def opOnChanged(self, obj, prop):
            _sync_contour_modes(obj, prop)

        def opOnDocumentRestored(self, obj):
            _ensure_contour_properties(obj)
            _sync_contour_modes(obj)

        def opRejectAddBase(self, obj, base, sub):
            return not (isinstance(sub, str) and sub.startswith("Face"))

        def opPropertyDefaults(self, obj, job):
            # Override Waterline defaults so newly-created operations land in a
            # bevel-appropriate state even before the user opens the task panel.
            defaults = super().opPropertyDefaults(obj, job)
            defaults["Algorithm"] = "OCL Adaptive"
            defaults["LayerMode"] = "Multi-pass"
            defaults["CutPattern"] = "None"
            defaults["BoundaryEnforcement"] = True
            defaults["StepOver"] = 25.0
            return defaults

        def opSetDefaultValues(self, obj, job):
            super().opSetDefaultValues(obj, job)

            # Bevel finishing defaults.
            if hasattr(obj, "Algorithm"):
                obj.Algorithm = "OCL Adaptive"
            if hasattr(obj, "BoundBox"):
                obj.BoundBox = "BaseBoundBox"
            if hasattr(obj, "LayerMode"):
                obj.LayerMode = "Multi-pass"
            if hasattr(obj, "CutPattern"):
                obj.CutPattern = "None"
            if hasattr(obj, "BoundaryEnforcement"):
                obj.BoundaryEnforcement = True
            if hasattr(obj, "StepOver"):
                obj.StepOver = 25
            _apply_contour_defaults(obj)
            _sync_contour_modes(obj)

        def opExecute(self, obj):
            # Synchronize first so restored documents / scripted edits have a
            # coherent property state before validation or execution.
            _sync_contour_modes(obj)
            _normalize_base_to_job_models(obj)
            faces = _selected_faces(obj)
            if not faces:
                FreeCAD.Console.PrintError(
                    translate(
                        "CAM_Contour3D",
                        "3D Contour requires one or more selected faces.\n",
                    )
                )
                return

            min_tilt = float(getattr(obj, "MinBevelTilt", 5.0))
            if not any(_is_bevel_candidate_face(face, min_tilt) for face in faces):
                FreeCAD.Console.PrintError(
                    translate(
                        "CAM_Contour3D",
                        "3D Contour requires at least one face steeper than %.1f° from horizontal.\n"
                        % min_tilt,
                    )
                )
                return

            # Prefer the direct face-slicing implementation because it produces
            # robust bevel-following passes from the selected faces alone.
            self.commandlist = _build_slice_commands(faces, obj, self)
            result = bool(self.commandlist)
            if not self.commandlist:
                # Fall back to a temporary surrogate model if the direct slicing
                # path cannot derive any section curves.  This retains a bridge
                # to Waterline for edge cases while keeping the common path fast.
                result = self._execute_selected_face_surrogate(obj, faces)
            cleanup_commands = None
            if getattr(obj, "FinishPass", False) and getattr(obj, "BoundaryCleanupPass", False):
                # Cleanup is built from the deepest actually-generated slice,
                # not from nominal final depth, so it matches the path that was
                # just produced even if depth clamping occurred.
                cleanup_depth = _deepest_depth_from_commands(self.commandlist)
                if cleanup_depth is not None:
                    cleanup_commands = _build_boundary_cleanup_commands(faces, cleanup_depth, obj, self)
            self.commandlist = _post_process_contour_commands(
                self.commandlist,
                add_finish_pass=bool(getattr(obj, "FinishPass", False)),
                add_boundary_cleanup=bool(
                    getattr(obj, "FinishPass", False)
                    and getattr(obj, "BoundaryCleanupPass", False)
                ),
                add_spring_pass=bool(getattr(obj, "SpringPass", False)),
                boundary_cleanup_commands=cleanup_commands,
                keep_tool_down=bool(getattr(obj, "KeepToolDown", False)),
                min_stay_down_distance=float(getattr(obj, "MinStayDownDistance", 0.0).Value),
            )
            return result

        def _execute_selected_face_surrogate(self, obj, faces):
            """Execute Waterline against a temporary model built from the selected faces."""
            job = PathUtils.findParentJob(obj)
            doc = getattr(obj, "Document", None) or FreeCAD.ActiveDocument
            if not job or not doc:
                return super().opExecute(obj)

            face_compound = _make_face_compound(faces)
            if face_compound is None:
                return super().opExecute(obj)

            # Create a throwaway feature that exposes the selected faces as a
            # stand-alone model.  This lets Waterline operate on just the bevel
            # geometry without permanently modifying the job contents.
            temp_feature = doc.addObject("Part::Feature", "Contour3D_SelectedFaces")
            temp_feature.Label = "Contour3D Selected Faces"
            temp_feature.Shape = face_compound
            if hasattr(temp_feature, "ViewObject") and temp_feature.ViewObject:
                temp_feature.ViewObject.Visibility = False

            original_group = list(job.Model.Group)
            original_base = list(obj.Base) if obj.Base else []

            try:
                # Replace the job model group only for the duration of the base
                # Waterline execution, then restore everything in ``finally``.
                job.Model.Group = [temp_feature]
                obj.Base = []
                return super().opExecute(obj)
            finally:
                job.Model.Group = original_group
                obj.Base = original_base
                try:
                    doc.removeObject(temp_feature.Name)
                except Exception:
                    pass

else:

    class ObjectContour3D(PathOp.ObjectOp):
        """Import-safe placeholder when OpenCamLib is unavailable."""

        def opFeatures(self, obj):
            return (
                PathOp.FeatureTool
                | PathOp.FeatureDepths
                | PathOp.FeatureHeights
                | PathOp.FeatureStepDown
                | PathOp.FeatureCoolant
                | PathOp.FeatureBaseFaces
            )

        @classmethod
        def opPropertyEnumerations(cls, dataType="data"):
            enums = PathOp.ObjectOp.opPropertyEnumerations(dataType="raw")
            enums.update(_contour_property_enumerations(dataType="raw"))

            if dataType == "raw":
                return enums

            data = []
            idx = 0 if dataType == "translated" else 1
            for key in enums:
                data.append((key, [tup[idx] for tup in enums[key]]))
            return data

        def initOperation(self, obj):
            _ensure_contour_properties(obj)
            _sync_contour_modes(obj)

        def opSetDefaultValues(self, obj, job):
            _apply_contour_defaults(obj)
            _sync_contour_modes(obj)

        def opOnChanged(self, obj, prop):
            _sync_contour_modes(obj, prop)

        def opOnDocumentRestored(self, obj):
            _ensure_contour_properties(obj)
            _sync_contour_modes(obj)

        def opPropertyDefinitions(self):
            return _contour_property_definitions()

        def opRejectAddBase(self, obj, base, sub):
            return not (isinstance(sub, str) and sub.startswith("Face"))

        def opExecute(self, obj):
            FreeCAD.Console.PrintError(
                translate(
                    "CAM_Contour3D",
                    "3D Contour requires OpenCamLib to be installed.\n",
                )
            )


def SetupProperties():
    """Return list of setup-sheet properties required for operation."""
    if not HAS_OCL:
        # In no-OCL mode only the contour-specific properties exist locally;
        # Waterline's properties are unavailable because the base class could
        # not be imported.
        return [tup[1] for tup in _contour_property_definitions()]
    return [tup[1] for tup in ObjectContour3D.opPropertyDefinitions(False)]


def Create(name, obj=None, parentJob=None):
    """Create and return a 3D Contour operation."""
    if obj is None:
        obj = FreeCAD.ActiveDocument.addObject("Path::FeaturePython", name)
    obj.Proxy = ObjectContour3D(obj, name, parentJob)
    return obj
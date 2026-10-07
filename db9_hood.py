"""
Parametric DB9 Connector Hood Enclosure for 3D Printing (FDM)
Author: Miguel / parametric-models

Description:
    Procedural Blender Python script that generates a two-part 3D-printable
    enclosure (Base and Lid) for a DB9 (D-Sub 9) connector.

Key Design Features:
    - Zero Front Obstruction:
      The entire enclosure body sits strictly behind the DB9 front mounting plate
      (Y >= 0). The front face is a flat 30.0 mm x 12.0 mm mating wall that matches
      the connector plate footprint with zero overhang, preventing any disconnection
      or chassis interference when plugged into panel-mounted DB9 ports.
    - M3 Heat-Set Threaded Inserts:
      Two 3.85 mm diameter pilot holes (depth 5.5 mm) at X = +/-12.5 mm on the front
      mating face to receive standard 4.0 mm OD knurled brass heat-set inserts.
      The DB9 plate screws directly into these brass inserts from the front.
    - Compact Solder Cavity:
      Total depth of 14.0 mm from the front mating plane to the cable collar,
      providing ample room for soldered wires and heat-shrink tubing.
    - Integrated Strain Relief for Zip-Tie (Cincho):
      Houses a 5.0 mm diameter cable. Features an internal trapped pocket
      (5.6 mm wide in Y, 15.6 mm diameter) designed specifically to enclose a
      standard 2.0 x 1.0 mm cable tie with a 5.0 x 5.0 x 3.0 mm ratchet head.
    - Supportless 3D Printing:
      Both parts (Base and Lid) are laid flat on the build plate (Z = 0) with their
      cavities facing upwards. 100% printable without any support material.
"""

import math
import sys
import bpy
import bmesh
from mathutils import Matrix, Vector

# ==============================================================================
# PARAMETRIC CONFIGURATION (All units in millimeters)
# ==============================================================================

# Front DB9 plate specifications
PLATE_WIDTH = 30.0              # Front plate total width
PLATE_HEIGHT = 12.0             # Front plate total height
HOLE_DISTANCE_X = 12.5          # Center-to-hole distance (pitch = 25.0 mm)

# Front heat-set brass threaded inserts (M3 internal thread, 4.0 mm OD)
INSERT_PILOT_DIA = 3.85         # Hole diameter for thermal insertion
INSERT_DEPTH = 5.5              # Blind hole depth

# Central D-Sub 9 pass-through trapezoid cutout
# Internal connector body dimensions (solder side)
TRAP_WIDTH_TOP = 19.1
TRAP_WIDTH_BOTTOM = 17.2
TRAP_HEIGHT = 10.4
TRAP_CORNER_RADIUS = 2.0
TRAP_CORNER_SEGMENTS = 8

# Solder & wire internal chamber
FRONT_WALL_THICKNESS = 2.5      # Front mating wall thickness
SOLDER_CAVITY_DEPTH = 11.5      # Net cavity depth (Total depth from front = 2.5 + 11.5 = 14.0 mm)
SOLDER_CAVITY_WIDTH = 22.0
SOLDER_CAVITY_HEIGHT = 10.0

# Cable conduit & zip-tie strain relief
CABLE_DIAMETER = 5.0            # Cable pass-through diameter
ZIP_POCKET_Y_LENGTH = 5.6       # Pocket width along cable axis (fits 5.0 mm head)
ZIP_POCKET_RADIUS = 7.8         # Pocket radius (15.6 mm dia -> 5.3 mm depth from cable wall)
ZIP_POCKET_Y_POS = 18.0         # Center position along Y axis

# Enclosure assembly fasteners (2x M3 screws joining Base and Lid)
M3_SCREW_DIA = 3.3              # M3 clearance hole
M3_HEAD_DIA = 6.2               # Socket / button head counterbore diameter
M3_HEAD_DEPTH = 4.0             # Counterbore depth
M3_NUT_RADIUS = 3.25            # M3 hex nut circumscribed radius (5.6 mm across flats)
M3_NUT_DEPTH = 4.0              # Hex pocket depth
SCREW_POS_X = 10.5              # Screw X position
SCREW_POS_Y = 9.0               # Screw Y position

# Planarization planes to guarantee flat FDM print-bed faces after splitting:
# +/-6.0 mm matches the 12.0 mm front plate envelope and removes the extra
# +/-2.0 mm introduced by the rear 16.0 mm cable collar height.
BASE_PLANAR_Z = -6.0
LID_PLANAR_Z = 6.0
PLANAR_CUTTER_SIZE = 80.0


# ==============================================================================
# GEOMETRY GENERATOR HELPERS
# ==============================================================================

def create_mesh_object(name: str, bm: bmesh.types.BMesh) -> bpy.types.Object:
    """Creates a new Blender object from a BMesh instance and links it to the active collection."""
    mesh = bpy.data.meshes.new(f"{name}_Mesh")
    bm.to_mesh(mesh)
    bm.free()
    obj = bpy.data.objects.new(name, mesh)
    bpy.context.collection.objects.link(obj)
    return obj


def apply_boolean_difference(target: bpy.types.Object, cutter: bpy.types.Object) -> None:
    """Applies an exact boolean difference operation and cleans up the cutter object."""
    mod = target.modifiers.new("BoolDiff", 'BOOLEAN')
    mod.operation = 'DIFFERENCE'
    mod.object = cutter
    mod.solver = 'EXACT'
    bpy.context.view_layer.objects.active = target
    bpy.ops.object.modifier_apply(modifier=mod.name)
    bpy.data.objects.remove(cutter, do_unlink=True)


def make_cylinder(
    name: str,
    radius: float,
    depth: float,
    location: tuple[float, float, float],
    rot_euler: tuple[float, float, float] = (0.0, 0.0, 0.0),
    segments: int = 32
) -> bpy.types.Object:
    """Creates a cylinder/cone primitive object with specified transformation."""
    bm = bmesh.new()
    bmesh.ops.create_cone(
        bm,
        cap_ends=True,
        cap_tris=False,
        segments=segments,
        radius1=radius,
        radius2=radius,
        depth=depth
    )
    rx = Matrix.Rotation(rot_euler[0], 4, 'X')
    ry = Matrix.Rotation(rot_euler[1], 4, 'Y')
    rz = Matrix.Rotation(rot_euler[2], 4, 'Z')
    t = Matrix.Translation(location)
    bmesh.ops.transform(bm, matrix=t @ rx @ ry @ rz, verts=bm.verts)
    return create_mesh_object(name, bm)


def make_box(name: str, size: tuple[float, float, float], location: tuple[float, float, float]) -> bpy.types.Object:
    """Creates a rectangular box primitive object with specified dimensions and center."""
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    s = Matrix.Diagonal((size[0], size[1], size[2], 1.0))
    t = Matrix.Translation(location)
    bmesh.ops.transform(bm, matrix=t @ s, verts=bm.verts)
    return create_mesh_object(name, bm)


def get_trapezoid_filleted_coords(
    w_top: float = TRAP_WIDTH_TOP,
    w_bot: float = TRAP_WIDTH_BOTTOM,
    h: float = TRAP_HEIGHT,
    radius: float = TRAP_CORNER_RADIUS,
    segs: int = TRAP_CORNER_SEGMENTS
) -> list[Vector]:
    """Calculates 2D coordinates for an isosceles trapezoid with filleted corners in the X-Z plane."""
    y_top, y_bot = h / 2.0, -h / 2.0
    half_wt, half_wb = w_top / 2.0, w_bot / 2.0
    alpha = math.atan2(h, half_wt - half_wb)
    half_alpha_top = alpha / 2.0
    half_alpha_bot = (math.pi - alpha) / 2.0

    c_tr = Vector((half_wt - radius / math.tan(half_alpha_top), y_top - radius))
    c_tl = Vector((-c_tr.x, c_tr.y))
    c_br = Vector((half_wb - radius / math.tan(half_alpha_bot), y_bot + radius))
    c_bl = Vector((-c_br.x, c_br.y))

    pts: list[Vector] = []
    # Bottom Right
    for i in range(segs + 1):
        a = -math.pi / 2.0 + alpha * (i / segs)
        pts.append(c_br + Vector((radius * math.cos(a), radius * math.sin(a))))
    # Top Right
    for i in range(1, segs + 1):
        a = (alpha - math.pi / 2.0) + (math.pi - alpha) * (i / segs)
        pts.append(c_tr + Vector((radius * math.cos(a), radius * math.sin(a))))
    # Top Left
    for i in range(1, segs + 1):
        a = math.pi / 2.0 + alpha * (i / segs)
        pts.append(c_tl + Vector((radius * math.cos(a), radius * math.sin(a))))
    # Bottom Left
    for i in range(1, segs + 1):
        a = (math.pi - (alpha - math.pi / 2.0)) + (math.pi - alpha) * (i / segs)
        pts.append(c_bl + Vector((radius * math.cos(a), radius * math.sin(a))))
    return pts


def make_trapezoid_cutter(y_start: float, y_end: float) -> bpy.types.Object:
    """Creates a solid extruded trapezoid cutter along the Y axis."""
    pts2d = get_trapezoid_filleted_coords()
    bm = bmesh.new()
    vf = [bm.verts.new(Vector((p.x, y_start, p.y))) for p in pts2d]
    vb = [bm.verts.new(Vector((p.x, y_end, p.y))) for p in pts2d]
    bm.verts.ensure_lookup_table()
    bm.faces.new(vf)
    bm.faces.new(reversed(vb))
    n = len(pts2d)
    for i in range(n):
        bm.faces.new([vf[i], vf[(i + 1) % n], vb[(i + 1) % n], vb[i]])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return create_mesh_object("Trap_Cutter", bm)


def make_outer_hull() -> bpy.types.Object:
    """
    Constructs the lofted outer enclosure hull.
    Station 0 matches the front plate exactly (30x12 mm).
    Station 1 widens slightly behind the front face to 34 mm for screw boss reinforcement.
    Station 4 tapers to a reinforced 21x16 mm collar around the cable tie pocket.
    """
    bm = bmesh.new()
    stations = [
        (0.0,  30.0, 12.0, 1.2),  # Front mating face flush with DB9 plate (30x12 mm)
        (2.0,  34.0, 14.0, 1.5),  # Side wings for insert wall thickness
        (13.0, 34.0, 14.0, 1.5),  # Cavity end
        (15.5, 26.0, 15.0, 1.5),  # Collar transition
        (23.5, 21.0, 16.0, 1.5)   # Reinforced cable collar
    ]

    def rounded_rect_points(y: float, w: float, h: float, corner_r: float, segs: int = 4) -> list[Vector]:
        hw, hh = w / 2.0, h / 2.0
        pts: list[Vector] = []
        c_br = Vector((hw - corner_r, -hh + corner_r))
        for i in range(segs + 1):
            a = -math.pi / 2 + (math.pi / 2) * (i / segs)
            pts.append(Vector((c_br.x + corner_r * math.cos(a), y, c_br.y + corner_r * math.sin(a))))
        c_tr = Vector((hw - corner_r, hh - corner_r))
        for i in range(1, segs + 1):
            a = (math.pi / 2) * (i / segs)
            pts.append(Vector((c_tr.x + corner_r * math.cos(a), y, c_tr.y + corner_r * math.sin(a))))
        c_tl = Vector((-hw + corner_r, hh - corner_r))
        for i in range(1, segs + 1):
            a = math.pi / 2 + (math.pi / 2) * (i / segs)
            pts.append(Vector((c_tl.x + corner_r * math.cos(a), y, c_tl.y + corner_r * math.sin(a))))
        c_bl = Vector((-hw + corner_r, -hh + corner_r))
        for i in range(1, segs + 1):
            a = math.pi + (math.pi / 2) * (i / segs)
            pts.append(Vector((c_bl.x + corner_r * math.cos(a), y, c_bl.y + corner_r * math.sin(a))))
        return pts

    rings = [[bm.verts.new(p) for p in rounded_rect_points(y, w, h, cr)] for (y, w, h, cr) in stations]
    bm.verts.ensure_lookup_table()
    bm.faces.new(reversed(rings[0]))
    bm.faces.new(rings[-1])
    n = len(rings[0])
    for r in range(len(stations) - 1):
        v1, v2 = rings[r], rings[r + 1]
        for i in range(n):
            bm.faces.new([v1[i], v2[i], v2[(i + 1) % n], v1[(i + 1) % n]])
    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return create_mesh_object("DB9_Shell_Raw", bm)


# ==============================================================================
# MAIN MODEL BUILDER
# ==============================================================================

def build_db9_hood(cleanup_existing: bool = True) -> tuple[bpy.types.Object, bpy.types.Object]:
    """Generates the DB9 Base and Lid 3D printable objects in the active Blender scene."""
    if cleanup_existing:
        for obj in list(bpy.data.objects):
            if "DB9_Estuche" in obj.name or "DB9_Shell" in obj.name:
                bpy.data.objects.remove(obj, do_unlink=True)

    # 1. Create lofted outer shell
    shell = make_outer_hull()

    # 2. Subtract central D-Sub trapezoid pass-through window
    apply_boolean_difference(shell, make_trapezoid_cutter(-0.5, FRONT_WALL_THICKNESS + 0.1))

    # 3. Subtract blind pilot holes for M3 heat-set brass inserts (4.0 mm OD)
    apply_boolean_difference(
        shell,
        make_cylinder("InsertL", INSERT_PILOT_DIA / 2.0, INSERT_DEPTH * 2, (-HOLE_DISTANCE_X, 0.0, 0.0), (math.pi / 2, 0, 0))
    )
    apply_boolean_difference(
        shell,
        make_cylinder("InsertR", INSERT_PILOT_DIA / 2.0, INSERT_DEPTH * 2, (HOLE_DISTANCE_X, 0.0, 0.0), (math.pi / 2, 0, 0))
    )

    # 4. Subtract solder and wire internal chamber
    cavity_y_center = FRONT_WALL_THICKNESS + SOLDER_CAVITY_DEPTH / 2.0
    apply_boolean_difference(
        shell,
        make_box("SolderCavity", (SOLDER_CAVITY_WIDTH, SOLDER_CAVITY_DEPTH, SOLDER_CAVITY_HEIGHT), (0.0, cavity_y_center, 0.0))
    )

    # 5. Subtract 5.0 mm cable pass-through tunnel
    apply_boolean_difference(
        shell,
        make_cylinder("CableTunnel", CABLE_DIAMETER / 2.0, 15.0, (0.0, 19.0, 0.0), (math.pi / 2, 0, 0))
    )

    # 6. Subtract zip-tie pocket (fits 5x5x3 mm ratchet head)
    apply_boolean_difference(
        shell,
        make_cylinder("ZipPocket", ZIP_POCKET_RADIUS, ZIP_POCKET_Y_LENGTH, (0.0, ZIP_POCKET_Y_POS, 0.0), (math.pi / 2, 0, 0))
    )

    # 7. Split into Base (with monolithic front plate) and Top Lid
    base_obj = bpy.data.objects.new("DB9_Estuche_Base", shell.data.copy())
    bpy.context.collection.objects.link(base_obj)
    tapa_obj = bpy.data.objects.new("DB9_Estuche_Tapa", shell.data.copy())
    bpy.context.collection.objects.link(tapa_obj)
    bpy.data.objects.remove(shell, do_unlink=True)

    # Lid: cut away lower half and front wall
    apply_boolean_difference(tapa_obj, make_box("CutTapaBot", (50.0, 50.0, 25.0), (0.0, 12.0, -12.5)))
    apply_boolean_difference(tapa_obj, make_box("CutTapaFront", (50.0, FRONT_WALL_THICKNESS + 0.1, 25.0), (0.0, (FRONT_WALL_THICKNESS + 0.1) / 2.0, 6.0)))

    # Base: cut away upper cavity half, retaining the full front plate
    apply_boolean_difference(base_obj, make_box("CutBaseTop", (50.0, 50.0, 25.0), (0.0, FRONT_WALL_THICKNESS + 25.0, 12.5)))

    # 8. Planarize print-bed faces (local Z = -6.0 for Base, +6.0 for Lid)
    apply_boolean_difference(
        base_obj,
        make_box(
            "PlanarCutBaseBottom",
            (PLANAR_CUTTER_SIZE, PLANAR_CUTTER_SIZE, PLANAR_CUTTER_SIZE),
            (0.0, 0.0, BASE_PLANAR_Z - PLANAR_CUTTER_SIZE / 2.0)
        )
    )
    apply_boolean_difference(
        tapa_obj,
        make_box(
            "PlanarCutLidTop",
            (PLANAR_CUTTER_SIZE, PLANAR_CUTTER_SIZE, PLANAR_CUTTER_SIZE),
            (0.0, 0.0, LID_PLANAR_Z + PLANAR_CUTTER_SIZE / 2.0)
        )
    )

    # 9. Assembly screw holes (M3 clearance, nut pockets and counterbores)
    for obj in [base_obj, tapa_obj]:
        apply_boolean_difference(obj, make_cylinder("ScrewL", M3_SCREW_DIA / 2.0, 25.0, (-SCREW_POS_X, SCREW_POS_Y, 0.0)))
        apply_boolean_difference(obj, make_cylinder("ScrewR", M3_SCREW_DIA / 2.0, 25.0, (SCREW_POS_X, SCREW_POS_Y, 0.0)))

    # Screw counterbores in Lid
    apply_boolean_difference(tapa_obj, make_cylinder("HeadL", M3_HEAD_DIA / 2.0, M3_HEAD_DEPTH, (-SCREW_POS_X, SCREW_POS_Y, 7.0)))
    apply_boolean_difference(tapa_obj, make_cylinder("HeadR", M3_HEAD_DIA / 2.0, M3_HEAD_DEPTH, (SCREW_POS_X, SCREW_POS_Y, 7.0)))

    # Hex nut pockets in Base
    apply_boolean_difference(base_obj, make_cylinder("NutL", M3_NUT_RADIUS, M3_NUT_DEPTH, (-SCREW_POS_X, SCREW_POS_Y, -6.5), segments=6))
    apply_boolean_difference(base_obj, make_cylinder("NutR", M3_NUT_RADIUS, M3_NUT_DEPTH, (SCREW_POS_X, SCREW_POS_Y, -6.5), segments=6))

    # 10. Material assignments
    mat = bpy.data.materials.get("Mat_DB9_Case")
    if not mat:
        mat = bpy.data.materials.new("Mat_DB9_Case")
        mat.use_nodes = True
        bsdf = mat.node_tree.nodes.get("Principled BSDF")
        if bsdf:
            bsdf.inputs["Base Color"].default_value = (0.12, 0.12, 0.14, 1.0)
            bsdf.inputs["Roughness"].default_value = 0.35

    base_obj.data.materials.append(mat)
    tapa_obj.data.materials.append(mat)

    # 11. Position flat on the build plate (Z = 0) ready for 3D printing
    base_obj.location = Vector((-22.0, 0.0, 6.0))
    tapa_obj.rotation_euler = (0.0, math.pi, 0.0)
    tapa_obj.location = Vector((22.0, 0.0, 6.0))

    print(f"Generated: {base_obj.name} ({len(base_obj.data.vertices)} vertices)")
    print(f"Generated: {tapa_obj.name} ({len(tapa_obj.data.vertices)} vertices)")
    return base_obj, tapa_obj


# ==============================================================================
# SCRIPT EXECUTION
# ==============================================================================

if __name__ == "__main__":
    base, lid = build_db9_hood(cleanup_existing=True)

    # Optional CLI STL export if '--export' flag is passed
    if "--export" in sys.argv:
        bpy.ops.object.select_all(action='SELECT')
        bpy.ops.wm.stl_export(filepath="DB9_Hood_Enclosure.stl")
        print("Exported: DB9_Hood_Enclosure.stl")

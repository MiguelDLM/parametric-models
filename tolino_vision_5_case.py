"""
Parametric Protective Case for Tolino Vision 5 / Kobo Libra H2O E-Reader (3D Printing)
Author: Miguel / parametric-models

Description:
    Procedural Blender Python script that generates a form-fitting protective bumper
    case for the Tolino Vision 5 (Kobo Libra H2O) e-reader.

Physical Device Geometry:
    - Width: 144.0 mm (left to right)
    - Length: 164.0 mm (top to bottom)
    - Flat Back:
      The back of the device is completely flat, resting evenly on a table or print bed.
    - Asymmetric Front Thickness Profile:
      * Stays at 7.0 mm thickness for the first 122.0 mm (from the thin left edge).
      * Ramps up from 7.0 mm to 10.0 mm over the remaining 22.0 mm on the front
        face (the right ergonomic button grip side).
    - Asymmetric Corner Radii:
      * Left corners (top-left, bottom-left) are tighter / less curved (3.0 mm).
      * Right corners (top-right, bottom-right) are widely rounded (8.0 mm).

Key Design Features:
    - Highly Parametric:
      Easily tweak flat width, thin/thick depths, ramp transition style (smooth vs linear),
      corner radii, edge bevels, wall thicknesses, and tolerances at the top of the file.
    - Flat Interior Floor & 100% Supportless 3D Printing:
      The case prints flat on the bed at Z = 0. The interior cavity floor is completely flat
      at Z = 1.6 mm, matching the flat back of the e-reader.
    - Front Ramping Side Walls & Retaining Lip:
      Side walls and perimeter bezel ramp from 7mm on the left to 10mm on the right,
      securing the e-reader while keeping the display and physical buttons accessible.
    - Right-Side Micro-USB Cutout:
      Single edge cutout positioned on the thick right grip edge between 32.0 mm
      and 40.0 mm from the top edge (8.0 x 3.0 mm plus connector shroud clearance).
    - Rear Circular Power Button Cutout:
      Circular finger access hole through the flat back floor starting 9.0 mm from top
      and 9.0 mm from left (10.0 mm diameter plus comfortable finger clearance).
"""

import math
import sys
import bpy
import bmesh
from mathutils import Matrix, Vector

# ==============================================================================
# PARAMETRIC CONFIGURATION (All units in millimeters)
# ==============================================================================

# Device dimensions (Tolino Vision 5 / Kobo Libra H2O)
DEVICE_WIDTH = 144.0            # Total width across X axis (14.4 cm)
DEVICE_LENGTH = 164.0           # Total length across Y axis (16.4 cm)

# Asymmetric thickness profile
THICKNESS_FLAT = 7.0            # Flat section thickness on the left (7.0 mm)
THICKNESS_MAX = 10.0            # Maximum thickness on the right grip (10.0 mm / 1 cm)
FLAT_WIDTH = 122.0              # Extent of the flat 7mm region from left to right (12.2 cm)
RAMP_STYLE = 'SMOOTH'           # 'SMOOTH' (cubic S-curve) or 'LINEAR' (straight incline)

# Corner Curvature (Asymmetric left vs right)
# The left corners are tighter / less curved (more square), while the right corners are more rounded
CORNER_RADIUS_LEFT = 3.0        # Radius for top-left and bottom-left corners (less curved / tighter angle)
CORNER_RADIUS_RIGHT = 8.0       # Radius for top-right and bottom-right corners (more curved / rounded grip)
EDGE_BEVEL = 1.8                # Radius / size of the back edge perimeter bevel
BEVEL_SEGMENTS = 3              # 1 for a flat 45-degree chamfer, >=3 for smooth rounded fillet

# Case Wall and Tolerance Dimensions
WALL_THICKNESS = 1.8            # Outer perimeter side wall thickness
BOTTOM_THICKNESS = 1.6          # Back floor wall thickness
FIT_TOLERANCE = 0.4             # Clearance gap between case and device (0.3 - 0.5 mm)

# Front Retaining Lip & Screen Access
LIP_WIDTH = 3.5                 # Width of front retaining lip overlapping the bezel
LIP_THICKNESS = 1.4             # Height of the front retaining lip over the device front

# Right Edge Micro-USB Port (Located on the thick 10mm right side)
# Positioned from 32.0 mm to 40.0 mm measuring from top to bottom (8.0 mm wide x 3.0 mm high)
USB_DIST_FROM_TOP_START = 32.0   # Distance from top edge to top boundary of port (mm)
USB_DIST_FROM_TOP_END = 40.0     # Distance from top edge to bottom boundary of port (mm)
USB_PORT_HEIGHT = 3.0            # Nominal height of micro-USB receptacle along Z (mm)
USB_PLUG_CLEARANCE_Y = 2.5       # Extra clearance along Y for cable plug overmold shroud
USB_PLUG_CLEARANCE_Z = 2.5       # Extra vertical clearance along Z for cable plug overmold shroud

# Rear Power Button (Circular, Top-Left Back Corner)
# Starts at 9.0 mm from top edge and 9.0 mm from left edge, diameter is 10.0 mm
POWER_BUTTON_OFFSET_TOP = 9.0    # Distance from top edge to button outer perimeter (mm)
POWER_BUTTON_OFFSET_LEFT = 9.0   # Distance from left edge to button outer perimeter (mm)
POWER_BUTTON_DIAMETER = 10.0     # Physical diameter of the button (10.0 mm / 1.0 cm)
POWER_BUTTON_CLEARANCE = 2.0     # Extra radial clearance for comfortable finger press (hole = 12.0 mm)

# Model Generation Options
CREATE_DEVICE_MOCKUP = True     # If True, generates a mockup of the Tolino e-reader for fit verification


# ==============================================================================
# GEOMETRY HELPERS
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
    """Applies an exact boolean difference operation and removes the cutter object."""
    mod = target.modifiers.new("BoolDiff", 'BOOLEAN')
    mod.operation = 'DIFFERENCE'
    mod.object = cutter
    mod.solver = 'EXACT'
    bpy.context.view_layer.objects.active = target
    bpy.ops.object.modifier_apply(modifier=mod.name)
    bpy.data.objects.remove(cutter, do_unlink=True)


def make_box(name: str, size: tuple[float, float, float], location: tuple[float, float, float]) -> bpy.types.Object:
    """Creates a rectangular box primitive object centered at location."""
    bm = bmesh.new()
    bmesh.ops.create_cube(bm, size=1.0)
    s = Matrix.Diagonal((size[0], size[1], size[2], 1.0))
    t = Matrix.Translation(location)
    bmesh.ops.transform(bm, matrix=t @ s, verts=bm.verts)
    return create_mesh_object(name, bm)


def make_cylinder(
    name: str,
    radius: float,
    depth: float,
    location: tuple[float, float, float],
    segments: int = 32
) -> bpy.types.Object:
    """Creates a cylinder primitive object along Z axis centered at location."""
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
    t = Matrix.Translation(location)
    bmesh.ops.transform(bm, matrix=t, verts=bm.verts)
    return create_mesh_object(name, bm)


def calculate_thickness_at_x(
    x: float,
    flat_w: float = FLAT_WIDTH,
    total_w: float = DEVICE_WIDTH,
    t_thin: float = THICKNESS_FLAT,
    t_thick: float = THICKNESS_MAX,
    style: str = RAMP_STYLE
) -> float:
    """
    Evaluates the asymmetric thickness at a given X coordinate.
    For x <= flat_w, returns constant t_thin.
    For x > flat_w, transitions up to t_thick using either a smooth S-curve or linear ramp.
    """
    if x <= flat_w:
        return t_thin
    t = (x - flat_w) / (total_w - flat_w)
    t = max(0.0, min(1.0, t))
    if style == 'SMOOTH':
        factor = t * t * (3.0 - 2.0 * t)  # Smoothstep
    else:
        factor = t
    return t_thin + (t_thick - t_thin) * factor


def calculate_y_bounds_at_x(x: float, w: float, l: float, corner_r_left: float, corner_r_right: float) -> float:
    """Calculates the half-length in Y for a given X coordinate with asymmetric left/right corner radii."""
    hl = l / 2.0
    if x < corner_r_left:
        dx = corner_r_left - x
        dy = math.sqrt(max(0.0, corner_r_left**2 - dx**2))
        return (hl - corner_r_left) + dy
    elif x > (w - corner_r_right):
        dx = x - (w - corner_r_right)
        dy = math.sqrt(max(0.0, corner_r_right**2 - dx**2))
        return (hl - corner_r_right) + dy
    else:
        return hl


def generate_ereader_body(
    name: str,
    w: float,
    l: float,
    flat_w: float,
    t_thin: float,
    t_thick: float,
    corner_r_left: float,
    corner_r_right: float,
    edge_bevel: float,
    z_bottom: float = 0.0
) -> bpy.types.Object:
    """
    Procedurally constructs a solid body with a flat back (resting flat on table/print bed),
    asymmetric thickness along X (thicker on the front button grip side), asymmetric corner
    radii in X-Y (tighter/less curved on the left, rounded on the right), and beveled bottom edges.
    """
    bm = bmesh.new()

    # Generate dense X sample stations across curved corners and ramp boundaries
    x_vals = []
    # Left corner arc (tighter / less curved)
    for i in range(8):
        x_vals.append(corner_r_left * (1.0 - math.cos(i * (math.pi / 2.0) / 7.0)))
    # Flat linear region
    x_vals.extend([corner_r_left + (flat_w - corner_r_left) * (i / 10.0) for i in range(1, 10)])
    x_vals.append(flat_w)
    # Ramp region
    x_vals.extend([flat_w + (w - corner_r_right - flat_w) * (i / 10.0) for i in range(1, 10)])
    x_vals.append(w - corner_r_right)
    # Right corner arc (more curved / rounded)
    for i in range(1, 8):
        x_vals.append((w - corner_r_right) + corner_r_right * math.sin(i * (math.pi / 2.0) / 7.0))
    x_vals = sorted(list(set([round(val, 4) for val in x_vals if 0.0 <= val <= w])))

    rings = []
    for x in x_vals:
        y_max = calculate_y_bounds_at_x(x, w, l, corner_r_left, corner_r_right)
        thick = calculate_thickness_at_x(x, flat_w, w, t_thin, t_thick, RAMP_STYLE)
        eb = min(edge_bevel, thick / 2.0 - 0.2, y_max / 2.0 - 0.2)
        z_top = z_bottom + thick

        # Cross section profile in Y-Z for this X slice:
        # Bottom floor is flat at z_bottom with edge bevel eb; top face is at z_top (ramping up)
        pts = [
            Vector((x, y_max, z_top)),
            Vector((x, -y_max, z_top)),
            Vector((x, -y_max, z_bottom + eb)),
            Vector((x, -y_max + eb, z_bottom)),
            Vector((x, y_max - eb, z_bottom)),
            Vector((x, y_max, z_bottom + eb))
        ]
        rings.append([bm.verts.new(p) for p in pts])

    bm.verts.ensure_lookup_table()
    bm.faces.new(rings[0])
    bm.faces.new(reversed(rings[-1]))
    n_pts = len(rings[0])
    for r in range(len(rings) - 1):
        r1, r2 = rings[r], rings[r + 1]
        for i in range(n_pts):
            bm.faces.new([r1[i], r1[(i + 1) % n_pts], r2[(i + 1) % n_pts], r2[i]])

    bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
    return create_mesh_object(name, bm)


# ==============================================================================
# MAIN BUILDER
# ==============================================================================

def build_tolino_case(cleanup_existing: bool = True) -> tuple[bpy.types.Object, bpy.types.Object | None]:
    """
    Builds the Tolino Vision 5 protective case and optional reference device mockup.
    """
    if cleanup_existing:
        for obj in list(bpy.data.objects):
            if "Tolino_Vision_5" in obj.name or "Cavity_Cutter" in obj.name:
                bpy.data.objects.remove(obj, do_unlink=True)

    # 1. Device Reference Mockup
    device_obj = None
    if CREATE_DEVICE_MOCKUP:
        device_obj = generate_ereader_body(
            "Tolino_Vision_5_Device",
            w=DEVICE_WIDTH,
            l=DEVICE_LENGTH,
            flat_w=FLAT_WIDTH,
            t_thin=THICKNESS_FLAT,
            t_thick=THICKNESS_MAX,
            corner_r_left=CORNER_RADIUS_LEFT,
            corner_r_right=CORNER_RADIUS_RIGHT,
            edge_bevel=EDGE_BEVEL,
            z_bottom=0.0
        )

    # 2. Outer Enclosure Shell (Prints flat on the build plate at Z = 0)
    case_w = DEVICE_WIDTH + 2.0 * WALL_THICKNESS
    case_l = DEVICE_LENGTH + 2.0 * WALL_THICKNESS
    case_flat_w = FLAT_WIDTH + WALL_THICKNESS
    case_t_thin = BOTTOM_THICKNESS + THICKNESS_FLAT + LIP_THICKNESS
    case_t_thick = BOTTOM_THICKNESS + THICKNESS_MAX + LIP_THICKNESS
    case_corner_r_left = CORNER_RADIUS_LEFT + WALL_THICKNESS
    case_corner_r_right = CORNER_RADIUS_RIGHT + WALL_THICKNESS
    case_bevel = EDGE_BEVEL + 0.5

    case_obj = generate_ereader_body(
        "Tolino_Vision_5_Funda",
        w=case_w,
        l=case_l,
        flat_w=case_flat_w,
        t_thin=case_t_thin,
        t_thick=case_t_thick,
        corner_r_left=case_corner_r_left,
        corner_r_right=case_corner_r_right,
        edge_bevel=case_bevel,
        z_bottom=0.0
    )
    case_obj.location = Vector((-WALL_THICKNESS, 0.0, 0.0))
    bpy.context.view_layer.objects.active = case_obj
    bpy.ops.object.transform_apply(location=True, rotation=False, scale=False)

    # 3. Inner Cavity Subtraction (Device + Tolerance, flat floor at Z = BOTTOM_THICKNESS)
    cavity_cutter = generate_ereader_body(
        "Cavity_Cutter",
        w=DEVICE_WIDTH + 2.0 * FIT_TOLERANCE,
        l=DEVICE_LENGTH + 2.0 * FIT_TOLERANCE,
        flat_w=FLAT_WIDTH + FIT_TOLERANCE,
        t_thin=THICKNESS_FLAT + FIT_TOLERANCE,
        t_thick=THICKNESS_MAX + FIT_TOLERANCE,
        corner_r_left=CORNER_RADIUS_LEFT + FIT_TOLERANCE,
        corner_r_right=CORNER_RADIUS_RIGHT + FIT_TOLERANCE,
        edge_bevel=0.2,
        z_bottom=BOTTOM_THICKNESS
    )
    cavity_cutter.location = Vector((-FIT_TOLERANCE, 0.0, 0.0))
    bpy.context.view_layer.objects.active = cavity_cutter
    bpy.ops.object.transform_apply(location=True, rotation=False, scale=False)
    apply_boolean_difference(case_obj, cavity_cutter)

    # 4. Front Screen Window Cutout (retains perimeter retaining lip)
    window_w = DEVICE_WIDTH - 2.0 * LIP_WIDTH
    window_l = DEVICE_LENGTH - 2.0 * LIP_WIDTH
    screen_cutter = make_box(
        "Screen_Cutter",
        (window_w, window_l, 20.0),
        (LIP_WIDTH + window_w / 2.0, 0.0, BOTTOM_THICKNESS + THICKNESS_FLAT + 10.0)
    )
    apply_boolean_difference(case_obj, screen_cutter)

    # 5. Right-Side Micro-USB Cutout (thick 10mm right edge)
    # Measured from top edge: 32mm to 40mm down (8x3mm slot with cable clearance)
    usb_y_center = (DEVICE_LENGTH / 2.0) - (USB_DIST_FROM_TOP_START + USB_DIST_FROM_TOP_END) / 2.0
    usb_y_span = (USB_DIST_FROM_TOP_END - USB_DIST_FROM_TOP_START) + USB_PLUG_CLEARANCE_Y
    usb_z_span = USB_PORT_HEIGHT + USB_PLUG_CLEARANCE_Z
    usb_z_pos = BOTTOM_THICKNESS + THICKNESS_MAX / 2.0
    usb_cutter = make_box(
        "USB_Cutter",
        (WALL_THICKNESS * 6.0, usb_y_span, usb_z_span),
        (DEVICE_WIDTH, usb_y_center, usb_z_pos)
    )
    apply_boolean_difference(case_obj, usb_cutter)

    # 6. Rear Circular Power Button Cutout (top-left rear corner, through flat floor)
    # Starts at 9mm from top edge and 9mm from left edge, 10mm diameter
    power_btn_x = POWER_BUTTON_OFFSET_LEFT + POWER_BUTTON_DIAMETER / 2.0
    power_btn_y = (DEVICE_LENGTH / 2.0) - (POWER_BUTTON_OFFSET_TOP + POWER_BUTTON_DIAMETER / 2.0)
    power_hole_radius = (POWER_BUTTON_DIAMETER + POWER_BUTTON_CLEARANCE) / 2.0
    power_cutter = make_cylinder(
        "Power_Cutter",
        radius=power_hole_radius,
        depth=BOTTOM_THICKNESS * 6.0,
        location=(power_btn_x, power_btn_y, BOTTOM_THICKNESS / 2.0),
        segments=32
    )
    apply_boolean_difference(case_obj, power_cutter)

    # 7. Material Assignments
    mat_case = bpy.data.materials.get("Mat_Tolino_Case")
    if not mat_case:
        mat_case = bpy.data.materials.new("Mat_Tolino_Case")
        mat_case.use_nodes = True
        bsdf = mat_case.node_tree.nodes.get("Principled BSDF")
        if bsdf:
            bsdf.inputs["Base Color"].default_value = (0.05, 0.45, 0.8, 1.0)  # Matte TPU Blue
            bsdf.inputs["Roughness"].default_value = 0.35
    case_obj.data.materials.append(mat_case)

    if device_obj:
        mat_dev = bpy.data.materials.get("Mat_Tolino_Device")
        if not mat_dev:
            mat_dev = bpy.data.materials.new("Mat_Tolino_Device")
            mat_dev.use_nodes = True
            bsdf = mat_dev.node_tree.nodes.get("Principled BSDF")
            if bsdf:
                bsdf.inputs["Base Color"].default_value = (0.2, 0.22, 0.25, 1.0)
                bsdf.inputs["Roughness"].default_value = 0.4
        device_obj.data.materials.append(mat_dev)

        # Place side by side for clean inspection
        device_obj.location = Vector((-90.0, 0.0, 0.0))
        case_obj.location = Vector((90.0, 0.0, 0.0))

    print(f"Generated case: {case_obj.name} ({len(case_obj.data.vertices)} vertices)")
    return case_obj, device_obj


# ==============================================================================
# SCRIPT ENTRY POINT
# ==============================================================================

if __name__ == "__main__":
    case, dev = build_tolino_case(cleanup_existing=True)

    if "--export" in sys.argv:
        bpy.ops.object.select_all(action='DESELECT')
        case.select_set(True)
        bpy.context.view_layer.objects.active = case
        bpy.ops.wm.stl_export(filepath="Tolino_Vision_5_Case.stl")
        print("Exported: Tolino_Vision_5_Case.stl")

"""
Wirex: 25-Channel Wire Selector & Heavy-Duty Linear Pusher
Powered by:
- Servo 1: MG996R Standard Metal-Gear Servo (180° rotation native stroke)
  Driving a D=42.5mm direct pinion (Option 1) delivering a full 6.68 cm stroke
  (5.14 cm push beyond housing rim) with over 5.1 kgf (50.8 N) of linear thrust.
- Servo 2: SG90 / MG90S Micro Servo (Turret Angular Selection across 130°).

Architecture (Preserved & Unified):
1. Monolithic Base Chassis (`base_chassis.stl`):
   - 100% flat bottom plate (flush against any wall, board, or tabletop with ZERO gap).
   - 25 enclosed captive wire guide pockets along 130° arc with smooth top-curved lips and stable straight slots.
   - Smooth curved outer perimeter and wall terminations (bullnose rounded caps).
   - 4 heavy-duty external mounting feet with tangential filleted bridges and concave ski-jump gussets.
   - Integrated left/right 50° side walls and support link pillars.
   - Rear 130° sector (180°..310°) is 100% OPEN for complete spindle swing clearance.

2. Screw-Down Top Plate (`top_plate.stl`):
   - Matching enclosed wire clips with underside curved lips and stable straight slots.
   - Fastens directly into the base chassis uprights and pillars with 6x M3 screws.
   - Houses Servo 2 (SG90/MG90S) on the top surface, completely external to the chassis interior.

3. Heavy-Duty Spindle Carriage & 180° Linear Actuator (`spindle_carriage.stl`, `pusher_rack.stl`, `pinion_gear.stl`, `actuator_cover.stl`):
   - Solid continuous foundation floor under the entire rack travel (zero deflection under gear mesh load).
   - Solid 8mm vertical back wall directly resisting 20° gear separating forces.
   - Open side window facing pinion gear for direct 20° involute tooth mesh at Y = 0.
   - Precision servo mounting bumps/stanchions matching MG996R flange ears with 4x M3/M4 tapped pilot holes.
   - Full pass-through rack guide channel with 0.4mm running clearance on all sides (zero collision across 100% stroke).
   - 34-tooth module-1.25 ISO 20° involute pinion (D = 42.5mm, R_p = 21.25mm) directly mounted on MG996R spline.
   - Conjugate 20° trapezoidal toothed rack (125mm length) with front wire cradle notch.
   - Stroke for 180° = 21.25 * pi = 66.76 mm (6.68 cm), pushing wire 5.14 cm beyond outer rim.
"""

import math
import os
import numpy as np
import trimesh
from manifold3d import Manifold, CrossSection, JoinType
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

# -----------------------------------------------------------------------------
# GLOBAL PARAMETERS
# -----------------------------------------------------------------------------
N_POSITIONS = 25
WIRE_SPACING_MM = 5.0
WIRE_ARC_DEG = 130.0
LEFT_SUPPORT_DEG = 50.0
REAR_SWING_DEG = 130.0
RIGHT_SUPPORT_DEG = 50.0

ANGLE_STEP_DEG = WIRE_ARC_DEG / (N_POSITIONS - 1)  # 5.4167 deg

# Radii: 24 spaces * 5mm / rad(130 deg) = 52.89 mm pitch radius
R_PITCH = (WIRE_SPACING_MM * (N_POSITIONS - 1)) / math.radians(WIRE_ARC_DEG)  # ~52.89 mm
OUTER_LINE_THICKNESS = 2.2      # Outer line / enclosing rim thickness
PLATE_OUTER_R = R_PITCH + 5.5   # ~58.4 mm outer perimeter
GROOVE_DEPTH = 8.0              # Depth of wire clips
GROOVE_WIDTH = 2.6              # Width of wire clips
PLATE_THICKNESS = 5.0

# Vertical height scaled for MG996R:
DRUM_HEIGHT = 68.0              # Bottom plate bottom to top plate top
CLEARANCE_H = DRUM_HEIGHT - 2 * PLATE_THICKNESS  # 58.0 mm between plates
WALL_THICKNESS = 5.5

# Support links in the housing space:
LINK_RADIUS = 49.0
LINK_OD = 9.0
LINK_ID = 2.8                   # M3 pilot tap bore
LINK_ANGLES = [155.0, 335.0]

# Heavy-Duty External Base Mounting Feet:
MOUNT_TAB_RADIUS = 74.0         # Center radius of mounting screw holes
MOUNT_TAB_HOLE_D = 4.5          # 4.5mm clearance for M4 bolt/screw
MOUNT_TAB_CBORE_D = 8.5         # 8.5mm counterbore recess for DIN 912 M4 cap screw head
MOUNT_TAB_CBORE_DEPTH = 2.8     # 2.8mm counterbore depth
MOUNT_TAB_WIDTH = 18.0          # Flange ear width
MOUNT_TAB_THICKNESS = 6.0       # Reinforced 6mm thickness
MOUNT_TAB_ANGLES = [138.0, 174.0, 316.0, 352.0]

# Servo 1: MG996R Standard Metal-Gear Servo Dimensions
MG996_L = 40.7
MG996_W = 20.0
MG996_H = 36.5                  # Case height
MG996_TOTAL_H = 43.0            # Height to spline tip
MG996_EAR_SPAN = 54.0
MG996_SPLINE_OFFSET = 10.0      # Spline center from front edge

# Servo 2: SG90 / MG90S Micro Servo (Selection Turret Drive mounted on Top Plate)
SG90_L = 22.8
SG90_W = 12.2
SG90_H = 24.5
SG90_SPLINE_OFFSET = 5.8        # Distance from front edge to spline axis

# Option 1: 180° Direct Pinion Kinematics
# Stroke = R_p * pi = 21.25 * 3.14159 = 66.76 mm (~6.68 cm)
PINION_PITCH_R = 21.25          # 42.5mm pitch diameter
PINION_HEIGHT = 8.0
PINION_TEETH = 34               # Module 1.25
RACK_WIDTH = 8.0
RACK_HEIGHT = 9.0
RACK_LENGTH = 125.0             # Extended rack for 6.68cm stroke
PUSHER_HEAD_W = 14.0
PUSHER_HEAD_H = 13.0
PUSHER_HEAD_L = 5.0

PIVOT_SHAFT_D = 4.2             # 4mm steel pivot shaft
PIVOT_BEARING_OD = 10.0         # 624 bearing pocket (10mm OD x 4mm ID)

HERE = os.path.dirname(os.path.abspath(__file__))
STL_DIR = os.path.join(HERE, "..", "stl")
IMG_DIR = os.path.join(HERE, "..", "images")


def polar(r, deg):
    rad = math.radians(deg)
    return r * math.cos(rad), r * math.sin(rad)


def cylinder(height, radius, center=True, fn=64):
    cyl = Manifold.cylinder(height, radius, radius, fn)
    if center:
        cyl = cyl.translate([0, 0, -height / 2.0])
    return cyl


def box(x, y, z, center=True):
    b = Manifold.cube([x, y, z])
    if center:
        b = b.translate([-x / 2.0, -y / 2.0, -z / 2.0])
    return b


def to_trimesh(m: Manifold) -> trimesh.Trimesh:
    mesh_data = m.to_mesh()
    verts = np.array(mesh_data.vert_properties)[:, :3]
    faces = np.array(mesh_data.tri_verts)
    return trimesh.Trimesh(vertices=verts, faces=faces)


# -----------------------------------------------------------------------------
# CAD MODEL GENERATORS
# -----------------------------------------------------------------------------

def make_plate_2d_contour():
    """High-Resolution 2D profile with smooth S-curve transitions (460+ points):
       - 0°..180°: wire fan arc + left support sector (R = PLATE_OUTER_R, 240 steps)
       - 180°..188°: smooth tangent cosine S-curve blend from 58.4mm to 28.0mm (20 steps)
       - 188°..302°: rear spindle swing sector (R = 28.0mm, 100 steps)
       - 302°..310°: smooth tangent cosine S-curve blend from 28.0mm to 58.4mm (20 steps)
       - 310°..360°: right support sector (R = PLATE_OUTER_R, 80 steps)
       Completely eliminates sharp radial cliffs and polygon faceting!
    """
    recess_r = 28.0
    pts = []
    for i in range(241):
        a = i * 180.0 / 240.0
        pts.append(list(polar(PLATE_OUTER_R, a)))

    for i in range(1, 21):
        a = 180.0 + i * 8.0 / 20.0
        t = (1.0 - math.cos(i * math.pi / 20.0)) / 2.0
        r = PLATE_OUTER_R + (recess_r - PLATE_OUTER_R) * t
        pts.append(list(polar(r, a)))

    for i in range(1, 101):
        a = 188.0 + i * (302.0 - 188.0) / 100.0
        pts.append(list(polar(recess_r, a)))

    for i in range(1, 21):
        a = 302.0 + i * 8.0 / 20.0
        t = (1.0 - math.cos(i * math.pi / 20.0)) / 2.0
        r = recess_r + (PLATE_OUTER_R - recess_r) * t
        pts.append(list(polar(r, a)))

    for i in range(1, 81):
        a = 310.0 + i * 50.0 / 80.0
        pts.append(list(polar(PLATE_OUTER_R, a)))

    return pts


def make_stable_top_curved_groove_cutter(thickness, round_top=True, round_bottom=False):
    """Stable rectangular groove slot in 2D (zero circular notches at bottom/joints - user red circle reverted),
       with smooth circular fillet curve on the top section of the groove walls (user green circle area).
       When the wire is pushed, it glides over this smooth curved radius without catching.
    """
    R_FILLET = 0.8
    N_STEP = 12
    poly_yz = []

    poly_yz.append([-GROOVE_WIDTH / 2.0, -1.0])

    if round_top:
        poly_yz.append([-GROOVE_WIDTH / 2.0, thickness - R_FILLET])
        for i in range(N_STEP + 1):
            th = (i / N_STEP) * (math.pi / 2.0)
            y = (-GROOVE_WIDTH / 2.0 - R_FILLET) + R_FILLET * math.cos(th)
            z = (thickness - R_FILLET) + R_FILLET * math.sin(th)
            poly_yz.append([y, z])
        poly_yz.append([-GROOVE_WIDTH / 2.0 - R_FILLET - 0.2, thickness + 1.0])
        poly_yz.append([GROOVE_WIDTH / 2.0 + R_FILLET + 0.2, thickness + 1.0])
        for i in range(N_STEP + 1):
            th = math.pi / 2.0 - (i / N_STEP) * (math.pi / 2.0)
            y = (GROOVE_WIDTH / 2.0 + R_FILLET) - R_FILLET * math.cos(th)
            z = (thickness - R_FILLET) + R_FILLET * math.sin(th)
            poly_yz.append([y, z])
        poly_yz.append([GROOVE_WIDTH / 2.0, thickness - R_FILLET])
    else:
        poly_yz.append([-GROOVE_WIDTH / 2.0, thickness + 1.0])
        poly_yz.append([GROOVE_WIDTH / 2.0, thickness + 1.0])

    if round_bottom:
        poly_yz.append([GROOVE_WIDTH / 2.0, R_FILLET])
        for i in range(N_STEP + 1):
            th = (i / N_STEP) * (math.pi / 2.0)
            y = (GROOVE_WIDTH / 2.0 + R_FILLET) - R_FILLET * math.cos(th)
            z = R_FILLET - R_FILLET * math.sin(th)
            poly_yz.append([y, z])
        poly_yz.append([GROOVE_WIDTH / 2.0 + R_FILLET + 0.2, -1.0])
        poly_yz.append([-GROOVE_WIDTH / 2.0 - R_FILLET - 0.2, -1.0])
        for i in range(N_STEP + 1):
            th = math.pi / 2.0 - (i / N_STEP) * (math.pi / 2.0)
            y = (-GROOVE_WIDTH / 2.0 - R_FILLET) + R_FILLET * math.cos(th)
            z = R_FILLET - R_FILLET * math.sin(th)
            poly_yz.append([y, z])
        poly_yz.append([-GROOVE_WIDTH / 2.0, R_FILLET])
    else:
        poly_yz.append([GROOVE_WIDTH / 2.0, -1.0])

    cs_yz = CrossSection([poly_yz[::-1]])  # CCW winding
    # Extrude along Z, then rotate [90, 0, 90] so length is along X, width along Y, height along Z
    cutter_3d = Manifold.extrude(cs_yz, GROOVE_DEPTH + 1.0).translate([0, 0, -(GROOVE_DEPTH + 1.0) / 2.0]).rotate([90, 0, 90])
    return cutter_3d


def make_inner_rim_fillet_cutter(r_inner=56.6, z_edge=5.0, r_fillet=0.8, round_top=True):
    """Revolved annular cutter that rounds the inside corner of the outer retaining rim
       (where the wire exits the groove), creating a smooth circular fillet for silky wire glide.
    """
    pts = []
    if round_top:
        pts.append([r_inner - 2.0, z_edge + 1.0])
        pts.append([r_inner + r_fillet, z_edge + 1.0])
        N = 16
        for i in range(N + 1):
            th = math.pi / 2.0 + (i / N) * (math.pi / 2.0)
            r = (r_inner + r_fillet) + r_fillet * math.cos(th)
            z = (z_edge - r_fillet) + r_fillet * math.sin(th)
            pts.append([r, z])
        pts.append([r_inner - 2.0, z_edge - r_fillet])
    else:
        pts.append([r_inner - 2.0, z_edge - 1.0])
        pts.append([r_inner + r_fillet, z_edge - 1.0])
        N = 16
        for i in range(N + 1):
            th = math.pi + (i / N) * (math.pi / 2.0)
            r = (r_inner + r_fillet) + r_fillet * math.cos(th)
            z = (z_edge + r_fillet) + r_fillet * math.sin(th)
            pts.append([r, z])
        pts.append([r_inner - 2.0, z_edge + r_fillet])

    cs = CrossSection([pts[::-1]])
    return Manifold.revolve(cs, circular_segments=240)


def make_plate_base(thickness, round_top=True, round_bottom=False, round_rim_top=True, round_rim_bottom=False, r_fillet=1.2):
    """Base plate with:
       - Smooth circular curved fillet (R = 1.2mm) on outer rim corners (both top and underside if requested).
       - Smooth circular curved fillet (R = 0.8mm) on inside rim corners (where wires exit the grooves).
       - Clean straight rectangular wire channels (stable variant).
       - Rounded top section on groove dividers (green circle fix).
       - 100% Watertight manifold architecture with zero boolean slivers.
    """
    r_outer = PLATE_OUTER_R
    N = 16
    poly_rz = []
    poly_rz.append([0.0, 0.0])

    if round_rim_bottom:
        poly_rz.append([r_outer - r_fillet, 0.0])
        for i in range(N + 1):
            phi = -math.pi / 2.0 + (i / N) * (math.pi / 2.0)
            r = (r_outer - r_fillet) + r_fillet * math.cos(phi)
            z = r_fillet + r_fillet * math.sin(phi)
            poly_rz.append([r, z])
    else:
        poly_rz.append([r_outer, 0.0])

    if round_rim_top:
        poly_rz.append([r_outer, thickness - r_fillet])
        for i in range(N + 1):
            phi = (i / N) * (math.pi / 2.0)
            r = (r_outer - r_fillet) + r_fillet * math.cos(phi)
            z = (thickness - r_fillet) + r_fillet * math.sin(phi)
            poly_rz.append([r, z])
        poly_rz.append([0.0, thickness])
    else:
        poly_rz.append([r_outer, thickness])
        poly_rz.append([0.0, thickness])

    cs = CrossSection([poly_rz])
    disc = Manifold.revolve(cs, circular_segments=240)

    # Cut out rear spindle swing sector (180° to 310°) beyond R = 28.0mm
    poly_cut = []
    recess_r = 28.0
    r_max = r_outer + 12.0

    for i in range(21):
        a = 180.0 + i * 8.0 / 20.0
        t = (1.0 - math.cos(i * math.pi / 20.0)) / 2.0
        r = recess_r + (r_outer - recess_r) * (1.0 - t)
        poly_cut.append(list(polar(r, a)))

    for i in range(1, 101):
        a = 188.0 + i * (302.0 - 188.0) / 100.0
        poly_cut.append(list(polar(recess_r, a)))

    for i in range(1, 21):
        a = 302.0 + i * 8.0 / 20.0
        t = (1.0 - math.cos(i * math.pi / 20.0)) / 2.0
        r = recess_r + (r_outer - recess_r) * t
        poly_cut.append(list(polar(r, a)))

    for i in range(61):
        a = 310.0 - i * (310.0 - 180.0) / 60.0
        poly_cut.append(list(polar(r_max, a)))

    cs_cut = CrossSection([poly_cut[::-1]])
    cut_3d = Manifold.extrude(cs_cut, thickness + 4.0).translate([0, 0, -2.0])
    base = disc - cut_3d

    # Center shaft bore
    center_hole = cylinder(thickness + 4.0, PIVOT_SHAFT_D / 2.0, center=False, fn=64).translate([0, 0, -2.0])
    base = base - center_hole

    # 25 wire grooves with top curved lips & stable straight slots
    slot_center_r = PLATE_OUTER_R - OUTER_LINE_THICKNESS - GROOVE_DEPTH / 2.0
    cutter = make_stable_top_curved_groove_cutter(thickness, round_top=round_top, round_bottom=round_bottom)
    cutters = None
    for i in range(N_POSITIONS):
        angle = i * ANGLE_STEP_DEG
        c = cutter.translate([slot_center_r, 0, 0]).rotate([0, 0, angle])
        cutters = c if cutters is None else cutters + c

    plate = base - cutters

    # Add inner rim fillet curves on top face (Base Chassis) and underside (Top Plate)
    if round_rim_top:
        cut_in_top = make_inner_rim_fillet_cutter(56.6, thickness, 0.8, round_top=True)
        plate = plate - cut_in_top
    if round_rim_bottom:
        cut_in_bot = make_inner_rim_fillet_cutter(56.6, 0.0, 0.8, round_top=False)
        plate = plate - cut_in_bot

    return plate


def make_curved_wall_manifold(start_deg, span_deg, height):
    """High-Resolution Curved vertical wall with semicircular bullnose rounded ends."""
    r_outer = PLATE_OUTER_R
    r_inner = PLATE_OUTER_R - WALL_THICKNESS
    r_mid = (r_outer + r_inner) / 2.0
    end_deg = start_deg + span_deg
    n_pts = 60

    outer_pts = [list(polar(r_outer, start_deg + i * span_deg / n_pts)) for i in range(n_pts + 1)]

    # Semicircular bullnose cap at end
    cap_end = []
    ang_rad = math.radians(end_deg)
    for i in range(13):
        th = -math.pi / 2.0 + i * math.pi / 12.0
        dr = (WALL_THICKNESS / 2.0) * math.cos(th)
        dt = (WALL_THICKNESS / 2.0) * math.sin(th)
        px = (r_mid + dr) * math.cos(ang_rad) - dt * math.sin(ang_rad)
        py = (r_mid + dr) * math.sin(ang_rad) + dt * math.cos(ang_rad)
        cap_end.append([px, py])

    inner_pts = [list(polar(r_inner, start_deg + (n_pts - i) * span_deg / n_pts)) for i in range(n_pts + 1)]

    # Semicircular bullnose cap at start
    cap_start = []
    ang_rad_s = math.radians(start_deg)
    for i in range(13):
        th = math.pi / 2.0 + i * math.pi / 12.0
        dr = (WALL_THICKNESS / 2.0) * math.cos(th)
        dt = (WALL_THICKNESS / 2.0) * math.sin(th)
        px = (r_mid + dr) * math.cos(ang_rad_s) - dt * math.sin(ang_rad_s)
        py = (r_mid + dr) * math.sin(ang_rad_s) + dt * math.cos(ang_rad_s)
        cap_start.append([px, py])

    poly = outer_pts + cap_end + inner_pts + cap_start
    cs = CrossSection([poly])
    return Manifold.extrude(cs, height)


def make_base_chassis():
    """Unified Monolithic Base Chassis:
       - 100% flat bottom plate (flush against any wall, board, or tabletop with ZERO gap).
       - Integrated left/right 50° side walls with bullnose rounded termination ends.
       - Integrated support link pillars.
       - 4 curved heavy-duty mounting feet with tangential filleted bridges and concave ski-jump gussets.
       - 25 captive wire grooves with stable straight channels and top-curved dividers.
       - Lower 624 bearing pocket recessed into UPPER face.
    """
    base = make_plate_base(PLATE_THICKNESS, round_top=True, round_bottom=False)

    # Bearing pocket for lower pivot on the UPPER face (624 bearing: 10mm OD x 4mm deep)
    bearing_pocket = cylinder(4.0, PIVOT_BEARING_OD / 2.0, center=False, fn=64).translate([0, 0, PLATE_THICKNESS - 4.0])
    base = base - bearing_pocket

    # 1. Extrude Left 50° Wall and Right 50° Wall directly from base with bullnose ends
    left_wall = make_curved_wall_manifold(130.0, 50.0, CLEARANCE_H).translate([0, 0, PLATE_THICKNESS])
    right_wall = make_curved_wall_manifold(310.0, 50.0, CLEARANCE_H).translate([0, 0, PLATE_THICKNESS])

    # 2. Extrude Support Link Pillars (at 155° and 335°)
    lx, ly = polar(LINK_RADIUS, 155.0)
    rx, ry = polar(LINK_RADIUS, 335.0)
    left_pillar = cylinder(CLEARANCE_H, LINK_OD / 2.0, center=False, fn=64).translate([lx, ly, PLATE_THICKNESS])
    right_pillar = cylinder(CLEARANCE_H, LINK_OD / 2.0, center=False, fn=64).translate([rx, ry, PLATE_THICKNESS])

    # 3. 4 Smooth Curved Mounting Feet with Tangential Fillets & Concave Ski-Jump Gussets
    poly_foot = []
    poly_foot.append([45.0, -(MOUNT_TAB_WIDTH / 2.0 + 3.0)])
    poly_foot.append([MOUNT_TAB_RADIUS - 2.0, -MOUNT_TAB_WIDTH / 2.0])
    N_LUG = 24
    for i in range(N_LUG + 1):
        th = -math.pi / 2.0 + i * math.pi / N_LUG
        px = MOUNT_TAB_RADIUS + (MOUNT_TAB_WIDTH / 2.0) * math.cos(th)
        py = (MOUNT_TAB_WIDTH / 2.0) * math.sin(th)
        poly_foot.append([px, py])
    poly_foot.append([MOUNT_TAB_RADIUS - 2.0, MOUNT_TAB_WIDTH / 2.0])
    poly_foot.append([45.0, MOUNT_TAB_WIDTH / 2.0 + 3.0])
    cs_foot = CrossSection([poly_foot]).offset(-1.0, JoinType.Round, circular_segments=32).offset(1.0, JoinType.Round, circular_segments=32)
    foot_m = Manifold.extrude(cs_foot, MOUNT_TAB_THICKNESS)

    # Concave ski-jump gusset sweeping smoothly from 24mm wall height down to 6mm foot
    poly_g = []
    N_G = 16
    for i in range(N_G + 1):
        t = i / N_G
        gx = 50.0 + (66.0 - 50.0) * t
        gz = MOUNT_TAB_THICKNESS + (24.0 - MOUNT_TAB_THICKNESS) * ((1.0 - t) ** 2)
        poly_g.append([gx, gz])
    poly_g.append([50.0, MOUNT_TAB_THICKNESS])
    cs_g = CrossSection([poly_g[::-1]])  # CCW winding
    gusset_m = Manifold.extrude(cs_g, 4.0).translate([0, 0, -2.0]).rotate([90, 0, 0])

    full_single_foot = foot_m + gusset_m
    shole = cylinder(MOUNT_TAB_THICKNESS + 4.0, MOUNT_TAB_HOLE_D / 2.0, center=False, fn=32).translate([MOUNT_TAB_RADIUS, 0, -2])
    cbore = cylinder(MOUNT_TAB_CBORE_DEPTH + 1.0, MOUNT_TAB_CBORE_D / 2.0, center=False, fn=32).translate([MOUNT_TAB_RADIUS, 0, MOUNT_TAB_THICKNESS - MOUNT_TAB_CBORE_DEPTH])
    single_foot_drilled = full_single_foot - shole - cbore

    mounting_feet = None
    for a in MOUNT_TAB_ANGLES:
        f = single_foot_drilled.rotate([0, 0, a])
        mounting_feet = f if mounting_feet is None else mounting_feet + f

    chassis = base + left_wall + right_wall + left_pillar + right_pillar + mounting_feet

    # Cut M3 screw pilot holes into uprights and pillars
    for a in [142.0, 168.0, 322.0, 348.0]:
        hx, hy = polar(PLATE_OUTER_R - WALL_THICKNESS / 2.0, a)
        chassis = chassis - cylinder(14.0, LINK_ID / 2.0, center=False, fn=32).translate([hx, hy, DRUM_HEIGHT - PLATE_THICKNESS - 12])

    chassis = chassis - cylinder(14.0, LINK_ID / 2.0, center=False, fn=32).translate([lx, ly, DRUM_HEIGHT - PLATE_THICKNESS - 12])
    chassis = chassis - cylinder(14.0, LINK_ID / 2.0, center=False, fn=32).translate([rx, ry, DRUM_HEIGHT - PLATE_THICKNESS - 12])

    return chassis


def make_top_plate():
    """Matching High-Resolution Top Plate that screws directly into the base chassis
       AND houses Servo 2 (SG90 / MG90S turret rotation drive) on top.
       Features matching underside curved groove profile AND smooth curved outer rim corners on both top and bottom (R = 1.2mm).
    """
    plate = make_plate_base(PLATE_THICKNESS, round_top=False, round_bottom=True, round_rim_top=True, round_rim_bottom=True)

    # Upper pivot bearing pocket on underside (624 bearing: 10mm OD x 4mm deep)
    recess = cylinder(4.0, PIVOT_BEARING_OD / 2.0, center=False, fn=64).translate([0, 0, 0])
    plate = plate - recess

    # Countersunk M3 screw holes matching side walls and pillars
    for a in [142.0, 168.0, 322.0, 348.0]:
        hx, hy = polar(PLATE_OUTER_R - WALL_THICKNESS / 2.0, a)
        hole = cylinder(PLATE_THICKNESS + 4, 1.6, center=False, fn=32).translate([hx, hy, -2])
        csink = cylinder(2.5, 3.2, center=False, fn=32).translate([hx, hy, PLATE_THICKNESS - 2.5])
        plate = plate - hole - csink

    lx, ly = polar(LINK_RADIUS, 155.0)
    rx, ry = polar(LINK_RADIUS, 335.0)
    for lx_, ly_ in [(lx, ly), (rx, ry)]:
        hole = cylinder(PLATE_THICKNESS + 4, 1.6, center=False, fn=32).translate([lx_, ly_, -2])
        csink = cylinder(2.5, 3.2, center=False, fn=32).translate([lx_, ly_, PLATE_THICKNESS - 2.5])
        plate = plate - hole - csink

    # Servo 2 (SG90/MG90S) Top-Mount Housing on upper face:
    s2_cx = -(SG90_L / 2.0 - SG90_SPLINE_OFFSET)  # -5.6 mm
    bracket_box = box(SG90_L + 12.0, SG90_W + 8.0, 18.0, center=True).translate([s2_cx, 0, PLATE_THICKNESS + 9.0])
    s2_cavity = box(SG90_L + 0.8, SG90_W + 0.8, 22.0, center=True).translate([s2_cx, 0, PLATE_THICKNESS + 10.0])
    spline_bore = cylinder(PLATE_THICKNESS + 4.0, 6.0, center=False, fn=48).translate([0, 0, -2.0])

    s2_ear1 = cylinder(24.0, 1.2, center=False, fn=32).translate([s2_cx + SG90_L / 2.0 + 2.5, 0, 4.0])
    s2_ear2 = cylinder(24.0, 1.2, center=False, fn=32).translate([s2_cx - SG90_L / 2.0 - 2.5, 0, 4.0])

    plate = (plate + bracket_box) - s2_cavity - spline_bore - s2_ear1 - s2_ear2

    return plate.translate([0, 0, DRUM_HEIGHT - PLATE_THICKNESS])


def make_spindle_bushing():
    """Precision 3D Printable Spindle Bushing / Bearing Adapter (10mm OD x 4.1mm ID x 4.8mm H).
       Fits directly into the Base Chassis pivot pocket, allowing the Spindle Carriage lower pivot pin
       to rotate smoothly with zero play even without a metal 624ZZ ball bearing.
    """
    outer = cylinder(4.0, 4.95, center=False, fn=64)  # 9.9mm OD for snug press-fit into 10.0mm pocket
    bore = cylinder(10.0, 2.05, center=True, fn=64)   # 4.1mm ID for low-friction rotation around 3.9mm pivot pin
    flange = cylinder(0.8, 6.0, center=False, fn=64).translate([0, 0, 4.0])  # 12mm OD top retaining flange
    return (outer + flange) - bore


def make_spindle_carriage():
    """Heavy-Duty Solid Monolithic Spindle Carriage for MG996R with D=42.5mm Direct Pinion.
       RIGID INTEGRATED ARCHITECTURE (ZERO LOOSE / CORNER-ONLY CONNECTIONS):
       - Integrated Lower Spindle Pivot Pin (Ø3.9mm x 3.8mm L with Ø6.5mm spacer collar):
         Extends down into the Base Chassis 624ZZ bearing (or 3D printed spindle bushing),
         providing the central physical pivot connection on which the carriage rotates.
       - 100% Continuous Solid Base Plate (Z = 6.0mm to 11.0mm): Fully unites the central pivot
         hub with both servo mounting stanchions and the rack support bedrock. Zero thin bridges!
       - 100% Solid Continuous Foundation Floor: The entire bottom under the pusher rack is solid
         plastic (from Z = 6.0mm to Z = 49.1mm, a massive 43mm solid bedrock!) directly resisting downward
         gear meshing thrust and eliminating rack deflection or popping out.
       - Solid 8mm Backing Wall: Firmly supports the rack back against the 20° gear separating force.
       - Top Turret Horn Socket + 4x M2 Screw Pilot Holes: Keys positively into Servo 2 horn from the top plate,
         with 4 screw pilot holes (at radius 7.5mm) allowing the SG90 horn to be firmly bolted to the carriage,
         preventing the carriage from loosely hanging or vibrating.
       - Open Side Window (+Y): Gear teeth directly mesh with rack teeth at pitch line Y = 0.
       - Servo Mounting Bumps/Stanchions: Precision ledges matching MG996R flange ears with 4x M3/M4 tapped pilot holes
         and 100% vertical line-of-sight screwdriver access.
       - Retainer Cover Bores: 2x M3 pilot holes (at X=-24.0, X=+18.0) to clamp the Actuator Cover over the track.
    """
    h = CLEARANCE_H - 2.0  # 56.0 mm tall
    center_z = PLATE_THICKNESS + 1.0  # 6.0 mm
    rack_z = center_z + MG996_TOTAL_H + 4.0 + 1.0  # ~54.0 mm
    s_cy = PINION_PITCH_R  # 21.25 mm

    # 1. Central Pivot Hub around (0, 0) with Integrated Lower Pivot Spindle Pin
    pivot_hub = cylinder(h, 8.0, center=False, fn=64).translate([0, 0, center_z])
    # Lower Spindle Pivot Pin & Spacer Collar to seat into Base Chassis bearing/pocket:
    spacer_collar = cylinder(1.0, 3.25, center=False, fn=48).translate([0, 0, center_z - 1.0])
    pivot_pin = cylinder(3.8, 1.95, center=False, fn=48).translate([0, 0, center_z - 4.8])
    pivot_bore = cylinder(h + 8.0, PIVOT_SHAFT_D / 2.0, center=False, fn=48).translate([0, 0, center_z - 6.0])

    # Top SG90 Servo Horn Pocket + 4x M2 Screw Pilot Tap Holes:
    horn_pocket = box(24.0, 8.0, 4.0, center=True).translate([0, 0, center_z + h - 1.5])
    sg90_screws = None
    for xp, yp in [(-7.5, 0.0), (7.5, 0.0), (0.0, -7.5), (0.0, 7.5)]:
        sh = cylinder(10.0, 0.95, center=False, fn=32).translate([xp, yp, center_z + h - 9.0])
        sg90_screws = sh if sg90_screws is None else sg90_screws + sh

    pivot = (pivot_hub + spacer_collar + pivot_pin) - pivot_bore - horn_pocket - sg90_screws

    # 2. 100% CONTINUOUS SOLID BASE SHELF SUPPORTING SERVO (Zero cutouts! Fully solid 5mm plate!)
    servo_bed = box(56.0, 36.0, 5.0, center=False).translate([-38.0, 0, center_z])

    # 3. Servo Mounting Bumps / Stanchions (flat horizontal landing pads for MG996 flange ears)
    front_stanchion = box(8.0, 26.0, 32.0, center=False).translate([10.0, s_cy - 13.0, center_z])
    rear_stanchion = box(8.0, 26.0, 32.0, center=False).translate([-38.0, s_cy - 13.0, center_z])

    # 4. 100% SOLID CONTINUOUS RACK SUPPORT BEDROCK (Zero hollow windows or thin struts! Full bedrock!):
    rack_support_block = box(80.0, 16.0, h, center=False).translate([-45.0, -16.0, center_z])

    # Central cylindrical reinforcing gusset solidly joining pivot cylinder to base plate and rack block
    gusset_center = cylinder(12.0, 11.0, center=False, fn=64).translate([0, 0, center_z])

    carriage_solid = pivot + servo_bed + front_stanchion + rear_stanchion + rack_support_block + gusset_center

    # 5. Guide Channel Cutter:
    guide_channel_cut = box(100.0, 16.0, 9.8, center=False).translate([-55.0, -8.0, rack_z - 4.9])

    # 6. Pinion gear rotation clearance (above rack floor, Y >= 0)
    pinion_clearance = cylinder(12.0, 22.8, center=False, fn=64).translate([0, s_cy, rack_z - 4.0])

    # 7. Actuator Cover clamping screw pilot holes (2x M3 pilot: 2.8mm dia x 14mm deep)
    cov_hole1 = cylinder(14.0, 1.4, center=False, fn=32).translate([-24.0, -11.0, rack_z - 4.0])
    cov_hole2 = cylinder(14.0, 1.4, center=False, fn=32).translate([18.0, -11.0, rack_z - 4.0])

    # 8. 4x M3/M4 pilot tap holes for MG996R flange ears (open vertical access!)
    s_holes = None
    for xp in [14.0, -34.0]:
        for yo in [-5.0, 5.0]:
            h_ = cylinder(18.0, 1.6, center=False, fn=32).translate([xp, s_cy + yo, center_z + 18.0])
            s_holes = h_ if s_holes is None else s_holes + h_

    # 9. Cable exit notch at bottom of front stanchion
    cable_notch = box(10.0, 12.0, 8.0, center=False).translate([9.0, s_cy - 6.0, center_z])

    carriage = carriage_solid - guide_channel_cut - pinion_clearance - cov_hole1 - cov_hole2 - s_holes - cable_notch
    return carriage


def make_pinion_gear():
    """High-Precision 34-tooth module-1.25 Direct 180° Involute Pinion (D = 42.5mm, R_p = 21.25mm).
       Features:
       - Standard 20° pressure angle involute teeth and tapered flanks for smooth rolling mesh.
       - Standard 25T servo horn recess (21mm dia x 2.5mm deep).
       - M3 center fastener bore.
       - 4x M3 screw mounting clearance holes (dia 3.3mm) at PCD = 14.0mm (radius 7.0mm at 0°, 90°, 180°, 270°)
         to securely screw the pinion directly into the metal circular servo horn plate of the MG996R!
       - 4x circular lightening holes (dia 5.2mm) on outer web at 45°, 135°, 225°, 315°.
    """
    MODULE = 1.25
    TEETH = 34
    PITCH_R = MODULE * TEETH / 2.0  # 21.25 mm
    ADDENDUM = MODULE               # 1.25 mm
    DEDENDUM = 1.25 * MODULE        # 1.5625 mm
    TIP_R = PITCH_R + ADDENDUM       # 22.5 mm
    ROOT_R = PITCH_R - DEDENDUM      # 19.6875 mm
    ANG_PER_TOOTH = 2.0 * math.pi / TEETH
    HALF_TOOTH_ANG = ANG_PER_TOOTH / 4.0
    inv_tip_spread = (TIP_R - PITCH_R) * math.tan(math.radians(20.0)) / PITCH_R
    inv_root_spread = (PITCH_R - ROOT_R) * math.tan(math.radians(20.0)) / PITCH_R

    pts = []
    for i in range(TEETH):
        mid_a = i * ANG_PER_TOOTH
        a1 = mid_a - HALF_TOOTH_ANG - inv_root_spread - (ANG_PER_TOOTH / 8.0)
        a2 = mid_a - HALF_TOOTH_ANG - inv_root_spread
        a3 = mid_a - HALF_TOOTH_ANG
        a4 = mid_a - HALF_TOOTH_ANG + inv_tip_spread
        a5 = mid_a + HALF_TOOTH_ANG - inv_tip_spread
        a6 = mid_a + HALF_TOOTH_ANG
        a7 = mid_a + HALF_TOOTH_ANG + inv_root_spread
        a8 = mid_a + HALF_TOOTH_ANG + inv_root_spread + (ANG_PER_TOOTH / 8.0)
        pts.append([ROOT_R * math.cos(a1), ROOT_R * math.sin(a1)])
        pts.append([ROOT_R * math.cos(a2), ROOT_R * math.sin(a2)])
        pts.append([PITCH_R * math.cos(a3), PITCH_R * math.sin(a3)])
        pts.append([TIP_R * math.cos(a4), TIP_R * math.sin(a4)])
        pts.append([TIP_R * math.cos(a5), TIP_R * math.sin(a5)])
        pts.append([PITCH_R * math.cos(a6), PITCH_R * math.sin(a6)])
        pts.append([ROOT_R * math.cos(a7), ROOT_R * math.sin(a7)])
        pts.append([ROOT_R * math.cos(a8), ROOT_R * math.sin(a8)])

    cs_gear = CrossSection([pts]).offset(-0.15, JoinType.Round, circular_segments=24).offset(0.15, JoinType.Round, circular_segments=24)
    gear = Manifold.extrude(cs_gear, PINION_HEIGHT).translate([0, 0, -PINION_HEIGHT / 2.0])
    bore = cylinder(PINION_HEIGHT + 4.0, 1.6, center=True, fn=32)
    horn_pocket = cylinder(3.0, 10.5, center=True, fn=48).translate([0, 0, -PINION_HEIGHT / 2.0 + 1.5])

    # 4x M3 screw mounting clearance holes for the MG996 round metal servo plate (PCD = 14mm -> R = 7.0mm)
    horn_screws = None
    for i in range(4):
        ang = i * (math.pi / 2.0)
        sx = 7.0 * math.cos(ang)
        sy = 7.0 * math.sin(ang)
        sh = cylinder(PINION_HEIGHT + 4.0, 1.65, center=True, fn=32).translate([sx, sy, 0])
        horn_screws = sh if horn_screws is None else horn_screws + sh

    # 4 circular lightening holes through gear web (R = 15.2mm at 45°, 135°, 225°, 315°)
    lightening_holes = None
    for i in range(4):
        ang = i * (math.pi / 2.0) + (math.pi / 4.0)
        hx = 15.2 * math.cos(ang)
        hy = 15.2 * math.sin(ang)
        lh = cylinder(PINION_HEIGHT + 4.0, 2.6, center=True, fn=32).translate([hx, hy, 0])
        lightening_holes = lh if lightening_holes is None else lightening_holes + lh

    return gear - bore - horn_pocket - horn_screws - lightening_holes


def make_pusher_rack():
    """Heavy-Duty Linear Toothed Rack (125mm length, module-1.25 teeth).
       Features standard 20° trapezoidal gear teeth for smooth, rolling involute meshing,
       plus reinforced front T-head with radiused wire cradle notch.
    """
    MODULE = 1.25
    PITCH = math.pi * MODULE  # 3.927 mm
    n_teeth = int((RACK_LENGTH - 14.0) / PITCH)
    pts = []
    # Back of rack bar (extends to Y = -7.5mm so pitch line is at Y = 0)
    pts.append([-RACK_LENGTH / 2.0, -7.5])
    pts.append([RACK_LENGTH / 2.0, -7.5])
    pts.append([RACK_LENGTH / 2.0, -1.56])

    # 20° trapezoidal teeth rising to Y = +1.25mm
    x_start = RACK_LENGTH / 2.0 - 6.0
    for i in range(n_teeth):
        t_center_x = x_start - i * PITCH
        pts.append([t_center_x + 1.43, -1.56])
        pts.append([t_center_x + 0.52, +1.25])
        pts.append([t_center_x - 0.52, +1.25])
        pts.append([t_center_x - 1.43, -1.56])

    pts.append([-RACK_LENGTH / 2.0, -1.56])
    cs_rack = CrossSection([pts]).offset(-0.15, JoinType.Round, circular_segments=24).offset(0.15, JoinType.Round, circular_segments=24)
    bar = Manifold.extrude(cs_rack, RACK_HEIGHT).translate([0, 0, -RACK_HEIGHT / 2.0])

    # Front pusher head: reinforced T-head with radiused wire notch
    head = box(PUSHER_HEAD_L, PUSHER_HEAD_W, PUSHER_HEAD_H, center=True).translate(
        [RACK_LENGTH / 2.0 + PUSHER_HEAD_L / 2.0, -3.1, 0]
    )
    wire_cradle = cylinder(PUSHER_HEAD_H + 2.0, 2.0, center=True, fn=32).translate(
        [RACK_LENGTH / 2.0 + PUSHER_HEAD_L, -3.1, 0]
    )
    head = head - wire_cradle
    return bar + head


def make_actuator_cover():
    """Heavy-Duty Guide Retainer Clip:
       Clamps across the top of the guide channel with 2x M3 screws, forming the top roof
       of the 4-sided captive slide tunnel. Prevents the rack from lifting upward or jumping
       teeth under wire pushing reaction forces.
    """
    cov = box(56.0, 16.0, 4.0, center=True).translate([-3.0, -8.0, 0])
    screw1 = cylinder(8.0, 1.6, center=True, fn=32).translate([-24.0, -11.0, 0])
    screw2 = cylinder(8.0, 1.6, center=True, fn=32).translate([18.0, -11.0, 0])
    return cov - screw1 - screw2


def make_sg90_model():
    """Detailed 3D visual model of the SG90 micro servo (Turret Drive on Top Plate)."""
    body = box(SG90_L, SG90_W, SG90_H, center=True)
    ears = box(SG90_L + 9.0, SG90_W, 2.0, center=True).translate([0, 0, SG90_H / 2.0 - 5.0])
    spline_center_x = -(SG90_L / 2.0 - SG90_SPLINE_OFFSET)
    tower = cylinder(4.0, 5.0, center=False).translate([spline_center_x, 0, SG90_H / 2.0])
    spline = cylinder(4.0, 2.4, center=False).translate([spline_center_x, 0, SG90_H / 2.0 + 4.0])
    return body + ears + tower + spline


def make_mg996_model():
    """Detailed 3D visual model of the MG996R standard servo."""
    body = box(MG996_L, MG996_W, MG996_H, center=True)
    ears = box(MG996_EAR_SPAN, MG996_W, 2.5, center=True).translate([0, 0, MG996_H / 2.0 - 5.0])
    spline_center_x = MG996_L / 2.0 - MG996_SPLINE_OFFSET
    tower = cylinder(5.0, 6.5, center=False).translate([spline_center_x, 0, MG996_H / 2.0])
    spline = cylinder(4.5, 3.0, center=False).translate([spline_center_x, 0, MG996_H / 2.0 + 5.0])
    return body + ears + tower + spline


# -----------------------------------------------------------------------------
# BUILD AND RENDER PIPELINE
# -----------------------------------------------------------------------------

def build_all():
    os.makedirs(STL_DIR, exist_ok=True)
    os.makedirs(IMG_DIR, exist_ok=True)

    print("=== Exporting Option 1 (180° Direct Pinion D=42.5mm) Wirex Models ===")
    parts = {
        "base_chassis": make_base_chassis(),
        "top_plate": make_top_plate(),
        "spindle_carriage": make_spindle_carriage(),
        "pinion_gear": make_pinion_gear(),
        "pusher_rack": make_pusher_rack(),
        "actuator_cover": make_actuator_cover(),
        "spindle_bushing": make_spindle_bushing(),
    }

    for name, manifold in parts.items():
        tm = to_trimesh(manifold)
        path = os.path.join(STL_DIR, f"{name}.stl")
        tm.export(path)
        print(f"✓ Wrote {path} ({len(tm.vertices)} vertices, {len(tm.faces)} faces)")

    render_multi_panel()
    render_mounting_detail()
    render_spindle_mechanism_fit()
    render_actuator_cover_and_support_detail()
    render_groove_detail_closeup()
    render_spindle_pivot_detail()


def render_multi_panel():
    fig = plt.figure(figsize=(26, 8.5), dpi=220)
    fig.patch.set_facecolor("#FFFFFF")

    LIGHT_DIR = np.array([0.45, -0.4, 0.78])
    LIGHT_DIR = LIGHT_DIR / np.linalg.norm(LIGHT_DIR)

    COLORS = {
        "base_chassis": "#1B5E20",        # Deep Forest Green for Monolithic Chassis
        "top_plate": "#81C784",           # Light Green for Screw-Down Top Plate
        "spindle_carriage": "#512DA8",     # Deep Purple for MG996R Spindle Carriage
        "pusher_rack": "#E65100",          # Heavy-Duty Deep Orange for Pusher Rack
        "pinion_gear": "#FFD600",          # Gold for 34T Pinion
        "actuator_cover": "#0288D1",       # Blue for Retaining Cover
        "mg996_servo": "#212121",          # Charcoal/Black for MG996R Servo
    }

    def shade(color_hex, normal, alpha=1.0):
        rgb = np.array([int(color_hex[i:i + 2], 16) for i in (1, 3, 5)]) / 255.0
        n = normal / (np.linalg.norm(normal) + 1e-9)
        diffuse = max(0.0, float(np.dot(n, LIGHT_DIR)))
        intensity = 0.38 + 0.62 * diffuse
        shaded = np.clip(rgb * intensity, 0, 1)
        return (shaded[0], shaded[1], shaded[2], alpha)

    mesh_names = ["base_chassis", "top_plate", "spindle_carriage", "pinion_gear", "pusher_rack", "actuator_cover"]
    meshes = {k: trimesh.load_mesh(os.path.join(STL_DIR, f"{k}.stl")) for k in mesh_names}
    mg996_mesh = to_trimesh(make_mg996_model())

    aim_angle_deg = 65.0
    aim_rad = math.radians(aim_angle_deg)
    center_z = PLATE_THICKNESS + 1.0
    rack_z = center_z + MG996_TOTAL_H + 4.0 + 1.0
    ext_distance = 46.0  # Tip reaches R = 110mm, pushing 5.14cm beyond rim!

    axes = [
        fig.add_subplot(131, projection="3d"),
        fig.add_subplot(132, projection="3d"),
        fig.add_subplot(133, projection="3d"),
    ]

    configs = [
        {"ax": axes[0], "elev": 24, "azim": -56, "top_mode": "transparent",
         "title": "(A) Full Assembly with 180° Direct Pinion (Option 1)",
         "desc": "D=42.5mm pinion gives 6.68cm stroke from native 180° servo rotation (5.14cm push beyond rim)"},
        {"ax": axes[1], "elev": 32, "azim": -40, "top_mode": "exploded",
         "title": "(B) Exploded Internal View (Safe Clearance to Side Walls)",
         "desc": "Servo parallel to rack maintains >10.5mm clearance to walls | Rear 130° is 100% open"},
        {"ax": axes[2], "elev": 78, "azim": -55, "top_mode": "ghost",
         "title": "(C) Top Kinematic View (25 Captive Clips & 180° Stroke)",
         "desc": "Enclosed wire clips | Full 5.14cm extension beyond rim | 5.2 kgf (51N) push force"},
    ]

    for cfg in configs:
        ax = cfg["ax"]
        ax.set_facecolor("#FFFFFF")
        all_pts = []

        def draw_m(m, color, offset=(0, 0, 0), rot_z=0.0, alpha=1.0, edge_color=None):
            vecs = m.vertices.copy()
            if rot_z != 0.0:
                rad = math.radians(rot_z)
                ca, sa = math.cos(rad), math.sin(rad)
                vx = vecs[:, 0] * ca - vecs[:, 1] * sa
                vy = vecs[:, 0] * sa + vecs[:, 1] * ca
                vecs[:, 0], vecs[:, 1] = vx, vy
            vecs += np.array(offset)

            normals = m.face_normals.copy()
            if rot_z != 0.0:
                rad = math.radians(rot_z)
                ca, sa = math.cos(rad), math.sin(rad)
                nx = normals[:, 0] * ca - normals[:, 1] * sa
                ny = normals[:, 0] * sa + normals[:, 1] * ca
                normals[:, 0], normals[:, 1] = nx, ny

            fc = [shade(color, n, alpha) for n in normals]
            lw = 0.2 if edge_color else 0.0
            coll = Poly3DCollection(vecs[m.faces], facecolor=fc, edgecolor=edge_color, linewidths=lw)
            ax.add_collection3d(coll)
            all_pts.append(vecs.reshape(-1, 3))

        # Base Chassis
        draw_m(meshes["base_chassis"], COLORS["base_chassis"], alpha=0.92)

        # Top Plate
        if cfg["top_mode"] == "transparent":
            draw_m(meshes["top_plate"], COLORS["top_plate"], alpha=0.45)
        elif cfg["top_mode"] == "ghost":
            draw_m(meshes["top_plate"], COLORS["top_plate"], alpha=0.20)
        elif cfg["top_mode"] == "exploded":
            draw_m(meshes["top_plate"], COLORS["top_plate"], offset=(0, 0, 34.0), alpha=0.65)

        # Servo 2 (SG90) on top of Top Plate
        sg90_mesh = to_trimesh(make_sg90_model())
        s2_cx = -(SG90_L / 2.0 - SG90_SPLINE_OFFSET)
        top_offset_z = 34.0 if cfg["top_mode"] == "exploded" else 0.0
        draw_m(sg90_mesh, "#0288D1", offset=(s2_cx, 0, DRUM_HEIGHT + top_offset_z), rot_z=0.0)

        # Spindle Carriage
        draw_m(meshes["spindle_carriage"], COLORS["spindle_carriage"], rot_z=aim_angle_deg)

        # MG996R Servo on carriage
        s_local_x = -10.35
        s_local_y = PINION_PITCH_R
        sx = s_local_x * math.cos(aim_rad) - s_local_y * math.sin(aim_rad)
        sy = s_local_x * math.sin(aim_rad) + s_local_y * math.cos(aim_rad)
        draw_m(mg996_mesh, COLORS["mg996_servo"], offset=(sx, sy, center_z + MG996_H / 2.0 + 4.0), rot_z=aim_angle_deg)

        # Pinion Gear
        pin_local_y = PINION_PITCH_R
        px = -pin_local_y * math.sin(aim_rad)
        py = pin_local_y * math.cos(aim_rad)
        draw_m(meshes["pinion_gear"], COLORS["pinion_gear"], offset=(px, py, rack_z), rot_z=aim_angle_deg, edge_color="#B7950B")

        # Pusher Rack
        rx = ext_distance * math.cos(aim_rad)
        ry = ext_distance * math.sin(aim_rad)
        draw_m(meshes["pusher_rack"], COLORS["pusher_rack"], offset=(rx, ry, rack_z), rot_z=aim_angle_deg, edge_color="#A04000")

        # Actuator Cover Clip
        cov_x = -8.0 * math.sin(aim_rad)
        cov_y = 8.0 * math.cos(aim_rad)
        draw_m(meshes["actuator_cover"], COLORS["actuator_cover"], offset=(cov_x, cov_y, rack_z + RACK_HEIGHT / 2.0 + 2.0), rot_z=aim_angle_deg)

        # 25 Wires inside captive clips
        wire_lines = []
        for i in range(N_POSITIONS):
            w_ang = i * ANGLE_STEP_DEG
            w_rad = math.radians(w_ang)
            r_start = 48.8
            r_end = PLATE_OUTER_R - 0.5
            if i == 12:
                r_start += 51.4
                r_end += 54.0
            wx1, wy1 = r_start * math.cos(w_rad), r_start * math.sin(w_rad)
            wx2, wy2 = r_end * math.cos(w_rad), r_end * math.sin(w_rad)
            wz = rack_z
            wire_lines.append([(wx1, wy1, wz), (wx2, wy2, wz)])

        wire_colors = ["#FFFFFF" if i != 12 else "#00E676" for i in range(N_POSITIONS)]
        lc = Line3DCollection(wire_lines, colors=wire_colors, linewidths=2.4)
        ax.add_collection3d(lc)

        pts = np.concatenate(all_pts, axis=0)
        mins = pts.min(axis=0)
        maxs = pts.max(axis=0)
        centers = (mins + maxs) / 2.0
        max_range = (maxs - mins).max() / 2.0 * 1.05

        ax.set_xlim(centers[0] - max_range, centers[0] + max_range)
        ax.set_ylim(centers[1] - max_range, centers[1] + max_range)
        ax.set_zlim(centers[2] - max_range, centers[2] + max_range)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=cfg["elev"], azim=cfg["azim"])
        ax.set_axis_off()
        ax.set_title(cfg["title"], fontsize=12.5, weight="bold", pad=12, color="#1A1A1A")
        ax.text2D(0.5, -0.04, cfg["desc"], transform=ax.transAxes, fontsize=9.2, ha="center", color="#555550")

    legend_items = [
        ("Base Chassis (Monolithic Unit with Curved Feet)", "#1B5E20"),
        ("Top Plate (Matching Enclosed Rim & Wire Clips)", "#81C784"),
        ("Spindle Carriage (Solid Foundation Bedrock)", "#512DA8"),
        ("Pusher Rack (Option 1: 180° Direct Stroke 6.68cm)", "#E65100"),
        ("34T Involute Pinion (D=42.5mm Direct Spline Drive)", "#FFD600"),
        ("Turret Drive Servo 2 (SG90/MG90S on Top Plate)", "#0288D1"),
        ("Target Wire Extended 5.14cm Beyond Rim", "#00E676"),
    ]
    for i, (label, color) in enumerate(legend_items):
        x = 0.04 + (i % 4) * 0.24
        y = 0.965 if i < 4 else 0.925
        fig.text(x, y, "■", fontsize=11, color=color)
        fig.text(x + 0.012, y, label, fontsize=8.8, color="#2D2D2A")

    preview_path = os.path.join(IMG_DIR, "assembly_preview.png")
    fig.savefig(preview_path, bbox_inches="tight", facecolor="#FFFFFF")
    print(f"✓ Saved assembly preview: {preview_path}")


def render_mounting_detail():
    fig = plt.figure(figsize=(26, 8.5), dpi=220)
    fig.patch.set_facecolor("#FFFFFF")

    LIGHT_DIR = np.array([0.45, -0.4, 0.78])
    LIGHT_DIR = LIGHT_DIR / np.linalg.norm(LIGHT_DIR)

    def shade(color_hex, normal, alpha=1.0):
        rgb = np.array([int(color_hex[i:i + 2], 16) for i in (1, 3, 5)]) / 255.0
        n = normal / (np.linalg.norm(normal) + 1e-9)
        diffuse = max(0.0, float(np.dot(n, LIGHT_DIR)))
        intensity = 0.38 + 0.62 * diffuse
        shaded = np.clip(rgb * intensity, 0, 1)
        return (shaded[0], shaded[1], shaded[2], alpha)

    chassis_mesh = trimesh.load_mesh(os.path.join(STL_DIR, "base_chassis.stl"))
    top_mesh = trimesh.load_mesh(os.path.join(STL_DIR, "top_plate.stl"))
    carriage_mesh = trimesh.load_mesh(os.path.join(STL_DIR, "spindle_carriage.stl"))
    pinion_mesh = trimesh.load_mesh(os.path.join(STL_DIR, "pinion_gear.stl"))
    rack_mesh = trimesh.load_mesh(os.path.join(STL_DIR, "pusher_rack.stl"))
    cover_mesh = trimesh.load_mesh(os.path.join(STL_DIR, "actuator_cover.stl"))
    mg996_mesh = to_trimesh(make_mg996_model())
    sg90_mesh = to_trimesh(make_sg90_model())

    axes = [
        fig.add_subplot(131, projection="3d"),
        fig.add_subplot(132, projection="3d"),
        fig.add_subplot(133, projection="3d"),
    ]

    configs = [
        {
            "ax": axes[0], "elev": 30, "azim": 55, "view_mode": "carriage_focus",
            "title": "(A) Solid Monolithic Carriage & Dual Servo Plate Screws",
            "desc": "Continuous 5mm base plate & solid bedrock (zero loose corners) | 4x M3 MG996 horn screws & top SG90 screws",
        },
        {
            "ax": axes[1], "elev": 38, "azim": -50, "view_mode": "groove_focus",
            "title": "(B) 25 Wire Grooves: Stable Straight Slots & Curved Top Section",
            "desc": "Straight clean slot (zero bottom scoops) | Top section of dividers rounded with smooth curve (green circle fix)",
        },
        {
            "ax": axes[2], "elev": 30, "azim": -52, "view_mode": "flush_base_assembly",
            "title": "(C) 100% Flush Base Mounting & Top-Mounted Turret Servo 2",
            "desc": "Base bottom is 100% flat (zero-gap wall/table mounting) | Curved feet with concave ski-jump gussets",
        },
    ]

    for cfg in configs:
        ax = cfg["ax"]
        ax.set_facecolor("#FFFFFF")
        all_pts = []

        def draw_m(m, color, offset=(0, 0, 0), rot_z=0.0, alpha=1.0, edge_color=None):
            vecs = m.vertices.copy()
            if rot_z != 0.0:
                rad = math.radians(rot_z)
                ca, sa = math.cos(rad), math.sin(rad)
                vx = vecs[:, 0] * ca - vecs[:, 1] * sa
                vy = vecs[:, 0] * sa + vecs[:, 1] * ca
                vecs[:, 0], vecs[:, 1] = vx, vy
            vecs += np.array(offset)

            normals = m.face_normals.copy()
            if rot_z != 0.0:
                rad = math.radians(rot_z)
                ca, sa = math.cos(rad), math.sin(rad)
                nx = normals[:, 0] * ca - normals[:, 1] * sa
                ny = normals[:, 0] * sa + normals[:, 1] * ca
                normals[:, 0], normals[:, 1] = nx, ny

            fc = [shade(color, n, alpha) for n in normals]
            lw = 0.2 if edge_color else 0.0
            coll = Poly3DCollection(vecs[m.faces], facecolor=fc, edgecolor=edge_color, linewidths=lw)
            ax.add_collection3d(coll)
            all_pts.append(vecs.reshape(-1, 3))

        center_z = PLATE_THICKNESS + 1.0
        rack_z = center_z + MG996_TOTAL_H + 4.0 + 1.0

        if cfg["view_mode"] == "carriage_focus":
            draw_m(carriage_mesh, "#512DA8", alpha=0.92)
            draw_m(mg996_mesh, "#212121", offset=(-10.35, PINION_PITCH_R, center_z + MG996_H / 2.0 + 4.0), alpha=0.88)
            draw_m(pinion_mesh, "#FFD600", offset=(0, PINION_PITCH_R, rack_z), edge_color="#B7950B")
            draw_m(rack_mesh, "#E65100", offset=(24.0, 0, rack_z), edge_color="#A04000")
            draw_m(cover_mesh, "#0288D1", offset=(-3.0, -8.0, rack_z + RACK_HEIGHT / 2.0 + 2.0))
            for x_p in [14.0, -34.0]:
                for y_off in [-5.0, 5.0]:
                    ax.plot([x_p, x_p], [PINION_PITCH_R + y_off, PINION_PITCH_R + y_off], [center_z + 24, center_z + 42], color="#B0BEC5", linewidth=3.5)

        elif cfg["view_mode"] == "groove_focus":
            # Isolate 4 grooves in the bottom plate to show top curved dividers and straight slots
            mask = (
                (chassis_mesh.vertices[chassis_mesh.faces][:, :, 0] > 15.0) &
                (chassis_mesh.vertices[chassis_mesh.faces][:, :, 0] < 38.0) &
                (chassis_mesh.vertices[chassis_mesh.faces][:, :, 1] > 38.0) &
                (chassis_mesh.vertices[chassis_mesh.faces][:, :, 1] < 58.0) &
                (chassis_mesh.vertices[chassis_mesh.faces][:, :, 2] <= 7.0)
            ).all(axis=1)
            f_sub = chassis_mesh.faces[mask]
            fc_sub = [shade("#1B5E20", n) for n in chassis_mesh.face_normals[mask]]
            coll = Poly3DCollection(chassis_mesh.vertices[f_sub], facecolor=fc_sub, edgecolor="#0D3B0E", linewidths=0.35)
            ax.add_collection3d(coll)
            ax.set_xlim(16, 36)
            ax.set_ylim(39, 56)
            ax.set_zlim(0, 6.5)
            ax.text(22.0, 52.0, 5.8, "Smooth Curved Dividers (R=0.8mm)", color="#00E676", fontsize=9.5, weight="bold")
            ax.text(26.0, 44.0, 0.4, "Stable Straight Base (0 Scoops)", color="#FF5252", fontsize=9.5, weight="bold")

        elif cfg["view_mode"] == "flush_base_assembly":
            draw_m(chassis_mesh, "#1B5E20", alpha=0.92)
            draw_m(top_mesh, "#81C784", alpha=0.50)
            draw_m(carriage_mesh, "#512DA8", rot_z=65.0)
            s2_cx = -(SG90_L / 2.0 - SG90_SPLINE_OFFSET)
            draw_m(sg90_mesh, "#0288D1", offset=(s2_cx, 0, DRUM_HEIGHT + 7.0))
            aim_rad = math.radians(65.0)
            px = -PINION_PITCH_R * math.sin(aim_rad)
            py = PINION_PITCH_R * math.cos(aim_rad)
            draw_m(pinion_mesh, "#FFD600", offset=(px, py, rack_z), rot_z=65.0, edge_color="#B7950B")
            rx = 42.0 * math.cos(aim_rad)
            ry = 42.0 * math.sin(aim_rad)
            draw_m(rack_mesh, "#E65100", offset=(rx, ry, rack_z), rot_z=65.0, edge_color="#A04000")

            surf_pts = np.array([[-90, -90, 0], [90, -90, 0], [90, 90, 0], [-90, 90, 0]])
            ax.add_collection3d(Poly3DCollection([surf_pts], facecolor="#CFD8DC", alpha=0.35, edgecolor="#90A4AE", linewidths=1.0))

        pts = np.concatenate(all_pts, axis=0) if len(all_pts) > 0 else np.array([[0, 0, 0]])
        mins = pts.min(axis=0)
        maxs = pts.max(axis=0)
        centers = (mins + maxs) / 2.0
        max_range = (maxs - mins).max() / 2.0 * 1.05

        if cfg["view_mode"] != "groove_focus":
            ax.set_xlim(centers[0] - max_range, centers[0] + max_range)
            ax.set_ylim(centers[1] - max_range, centers[1] + max_range)
            ax.set_zlim(centers[2] - max_range, centers[2] + max_range)

        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=cfg["elev"], azim=cfg["azim"])
        ax.set_axis_off()
        ax.set_title(cfg["title"], fontsize=12.0, weight="bold", pad=12, color="#1A1A1A")
        ax.text2D(0.5, -0.04, cfg["desc"], transform=ax.transAxes, fontsize=9.2, ha="center", color="#555550")

    legend_items = [
        ("Solid Carriage: Continuous 5mm base plate, 43mm solid bedrock, 4x flange screws", "#512DA8"),
        ("Clean Straight Grooves: Zero bottom scoops, smooth R=0.8mm top-curved lips", "#1B5E20"),
        ("100% Flush Base: Flat bottom plane (Z=0) bolts directly to flat wall/table with zero gap", "#CFD8DC"),
        ("Top-Mounted Turret Servo 2: Drives spindle from top plate; fully accessible", "#0288D1"),
    ]
    for i, (label, color) in enumerate(legend_items):
        x = 0.04 + (i % 2) * 0.48
        y = 0.965 if i < 2 else 0.925
        fig.text(x, y, "■", fontsize=11, color=color)
        fig.text(x + 0.012, y, label, fontsize=9.0, color="#2D2D2A")

    detail_path = os.path.join(IMG_DIR, "mounting_and_screw_detail.png")
    fig.savefig(detail_path, bbox_inches="tight", facecolor="#FFFFFF")
    print(f"✓ Saved mounting and screw detail preview: {detail_path}")


def render_spindle_mechanism_fit():
    fig = plt.figure(figsize=(26, 8.5), dpi=220)
    fig.patch.set_facecolor("#FFFFFF")

    LIGHT_DIR = np.array([0.45, -0.4, 0.78])
    LIGHT_DIR = LIGHT_DIR / np.linalg.norm(LIGHT_DIR)

    def shade(color_hex, normal, alpha=1.0):
        rgb = np.array([int(color_hex[i:i + 2], 16) for i in (1, 3, 5)]) / 255.0
        n = normal / (np.linalg.norm(normal) + 1e-9)
        diffuse = max(0.0, float(np.dot(n, LIGHT_DIR)))
        intensity = 0.38 + 0.62 * diffuse
        shaded = np.clip(rgb * intensity, 0, 1)
        return (shaded[0], shaded[1], shaded[2], alpha)

    carriage_mesh = trimesh.load_mesh(os.path.join(STL_DIR, "spindle_carriage.stl"))
    pinion_mesh = trimesh.load_mesh(os.path.join(STL_DIR, "pinion_gear.stl"))
    rack_mesh = trimesh.load_mesh(os.path.join(STL_DIR, "pusher_rack.stl"))
    cover_mesh = trimesh.load_mesh(os.path.join(STL_DIR, "actuator_cover.stl"))
    mg996_mesh = to_trimesh(make_mg996_model())

    axes = [
        fig.add_subplot(131, projection="3d"),
        fig.add_subplot(132, projection="3d"),
        fig.add_subplot(133, projection="3d"),
    ]

    configs = [
        {
            "ax": axes[0], "elev": 88, "azim": -90, "panel_mode": "overhead_mesh",
            "title": "(A) Overhead View: ISO 20° Involute Mesh & Guide Track Fit",
            "desc": "34T Involute Pinion meshes with 20° Trapezoidal Rack at Y=0 | Solid 8mm back wall supports rack",
        },
        {
            "ax": axes[1], "elev": 32, "azim": -46, "panel_mode": "iso_assembled",
            "title": "(B) Assembled Actuator Subassembly (MG996R + 34T Pinion + Rack + Cover)",
            "desc": "Solid continuous bedrock under rack | 4x M4 screws driven vertically | Retaining cover clamps rack securely",
        },
        {
            "ax": axes[2], "elev": 36, "azim": -55, "panel_mode": "kinematic_stroke",
            "title": "(C) 3-Stage Kinematic Stroke Motion: 0° -> 90° -> 180° Sweep",
            "desc": "Native 180° rotation delivers 6.68cm stroke | selected wire pushed 5.14cm beyond housing rim",
        },
    ]

    center_z = PLATE_THICKNESS + 1.0
    rack_z = center_z + MG996_TOTAL_H + 4.0 + 1.0
    s_cy = PINION_PITCH_R

    for cfg in configs:
        ax = cfg["ax"]
        ax.set_facecolor("#FFFFFF")
        all_pts = []

        def draw_m(m, color, offset=(0, 0, 0), rot_z=0.0, alpha=1.0, edge_color=None):
            vecs = m.vertices.copy()
            if rot_z != 0.0:
                rad = math.radians(rot_z)
                ca, sa = math.cos(rad), math.sin(rad)
                vx = vecs[:, 0] * ca - vecs[:, 1] * sa
                vy = vecs[:, 0] * sa + vecs[:, 1] * ca
                vecs[:, 0], vecs[:, 1] = vx, vy
            vecs += np.array(offset)

            normals = m.face_normals.copy()
            if rot_z != 0.0:
                rad = math.radians(rot_z)
                ca, sa = math.cos(rad), math.sin(rad)
                nx = normals[:, 0] * ca - normals[:, 1] * sa
                ny = normals[:, 0] * sa + normals[:, 1] * ca
                normals[:, 0], normals[:, 1] = nx, ny

            fc = [shade(color, n, alpha) for n in normals]
            lw = 0.25 if edge_color else 0.0
            coll = Poly3DCollection(vecs[m.faces], facecolor=fc, edgecolor=edge_color, linewidths=lw)
            ax.add_collection3d(coll)
            all_pts.append(vecs.reshape(-1, 3))

        if cfg["panel_mode"] == "overhead_mesh":
            draw_m(carriage_mesh, "#512DA8", alpha=0.30)
            draw_m(pinion_mesh, "#FFD600", offset=(0, s_cy, rack_z), alpha=0.95, edge_color="#8D6E63")
            draw_m(rack_mesh, "#E65100", offset=(10.0, 0, rack_z), alpha=0.95, edge_color="#4E342E")
            ax.plot([-40, 30], [0, 0], [rack_z + 6, rack_z + 6], color="#D50000", linestyle="--", linewidth=2.0)
            for xp in [14.0, -34.0]:
                for yo in [-5.0, 5.0]:
                    ax.plot([xp, xp], [s_cy + yo, s_cy + yo], [center_z + 20, center_z + 38], color="#37474F", linewidth=3.0)

        elif cfg["panel_mode"] == "iso_assembled":
            draw_m(carriage_mesh, "#512DA8", alpha=0.92)
            draw_m(mg996_mesh, "#212121", offset=(-10.35, s_cy, center_z + MG996_H / 2.0 + 4.0), alpha=0.90)
            draw_m(pinion_mesh, "#FFD600", offset=(0, s_cy, rack_z), alpha=0.95, edge_color="#B7950B")
            draw_m(rack_mesh, "#E65100", offset=(25.0, 0, rack_z), alpha=0.95, edge_color="#A04000")
            draw_m(cover_mesh, "#0288D1", offset=(-3.0, -8.0, rack_z + RACK_HEIGHT / 2.0 + 2.0), alpha=0.75)

        elif cfg["panel_mode"] == "kinematic_stroke":
            draw_m(carriage_mesh, "#512DA8", alpha=0.75)
            draw_m(pinion_mesh, "#FFD600", offset=(0, s_cy, rack_z), alpha=0.90, edge_color="#B7950B")
            draw_m(cover_mesh, "#0288D1", offset=(-3.0, -8.0, rack_z + RACK_HEIGHT / 2.0 + 2.0), alpha=0.60)
            # State 1: 0° Retracted
            draw_m(rack_mesh, "#FFB74D", offset=(-20.0, 0, rack_z), alpha=0.35)
            # State 2: 90° Mid-Stroke
            draw_m(rack_mesh, "#FF9800", offset=(13.4, 0, rack_z), alpha=0.55)
            # State 3: 180° Full Extension (+46.8mm, wire pushed 5.14cm beyond rim!)
            draw_m(rack_mesh, "#E65100", offset=(46.8, 0, rack_z), alpha=0.95, edge_color="#A04000")
            ax.plot([46.8 + RACK_LENGTH / 2.0 + 5.0, 46.8 + RACK_LENGTH / 2.0 + 56.4], [0, 0], [rack_z, rack_z], color="#00E676", linewidth=3.5)

        pts = np.concatenate(all_pts, axis=0)
        mins = pts.min(axis=0)
        maxs = pts.max(axis=0)
        centers = (mins + maxs) / 2.0
        max_range = (maxs - mins).max() / 2.0 * 1.05

        if cfg["panel_mode"] == "overhead_mesh":
            ax.set_xlim(-36, 26)
            ax.set_ylim(-16, 44)
            ax.set_zlim(center_z, rack_z + 12)
        else:
            ax.set_xlim(centers[0] - max_range, centers[0] + max_range)
            ax.set_ylim(centers[1] - max_range, centers[1] + max_range)
            ax.set_zlim(centers[2] - max_range, centers[2] + max_range)

        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=cfg["elev"], azim=cfg["azim"])
        ax.set_axis_off()
        ax.set_title(cfg["title"], fontsize=12.0, weight="bold", pad=12, color="#1A1A1A")
        ax.text2D(0.5, -0.04, cfg["desc"], transform=ax.transAxes, fontsize=9.2, ha="center", color="#555550")

    legend_items = [
        ("Spindle Carriage: Solid continuous floor under rack, solid 8mm back wall", "#512DA8"),
        ("34T Involute Pinion (D=42.5mm): Standard 20° pressure angle teeth", "#FFD600"),
        ("Pusher Rack: 20° trapezoidal teeth, radiused wire cradle, 6.68cm stroke", "#E65100"),
        ("Retainer Cover: Clamps rack securely against gear tooth climbing", "#0288D1"),
    ]
    for i, (label, color) in enumerate(legend_items):
        x = 0.04 + (i % 2) * 0.48
        y = 0.965 if i < 2 else 0.925
        fig.text(x, y, "■", fontsize=11, color=color)
        fig.text(x + 0.012, y, label, fontsize=9.0, color="#2D2D2A")

    fit_path = os.path.join(IMG_DIR, "spindle_mechanism_fit.png")
    fig.savefig(fit_path, bbox_inches="tight", facecolor="#FFFFFF")
    print(f"✓ Saved spindle mechanism fit preview: {fit_path}")


def render_actuator_cover_and_support_detail():
    """Dedicated explanatory visualizer explaining:
       (A) Spindle Carriage Support Bedrock: Solid floor underneath rack & solid back wall.
       (B) Mechanical Force & Constraint Diagram: Why the Actuator Cover is required (countering lifting reaction force).
       (C) Assembled 4-Sided Captive Tunnel: How the rack glides smoothly with zero play or derailment.
    """
    fig = plt.figure(figsize=(26, 8.5), dpi=220)
    fig.patch.set_facecolor("#FFFFFF")

    LIGHT_DIR = np.array([0.45, -0.4, 0.78])
    LIGHT_DIR = LIGHT_DIR / np.linalg.norm(LIGHT_DIR)

    def shade(color_hex, normal, alpha=1.0):
        rgb = np.array([int(color_hex[i:i + 2], 16) for i in (1, 3, 5)]) / 255.0
        n = normal / (np.linalg.norm(normal) + 1e-9)
        diffuse = max(0.0, float(np.dot(n, LIGHT_DIR)))
        intensity = 0.38 + 0.62 * diffuse
        shaded = np.clip(rgb * intensity, 0, 1)
        return (shaded[0], shaded[1], shaded[2], alpha)

    carriage_mesh = trimesh.load_mesh(os.path.join(STL_DIR, "spindle_carriage.stl"))
    pinion_mesh = trimesh.load_mesh(os.path.join(STL_DIR, "pinion_gear.stl"))
    rack_mesh = trimesh.load_mesh(os.path.join(STL_DIR, "pusher_rack.stl"))
    cover_mesh = trimesh.load_mesh(os.path.join(STL_DIR, "actuator_cover.stl"))
    mg996_mesh = to_trimesh(make_mg996_model())

    axes = [
        fig.add_subplot(131, projection="3d"),
        fig.add_subplot(132, projection="3d"),
        fig.add_subplot(133, projection="3d"),
    ]

    configs = [
        {
            "ax": axes[0], "elev": 30, "azim": 58, "mode": "foundation",
            "title": "(A) Solid Monolithic Carriage & Dual Servo Plate Screws",
            "desc": "Continuous 5mm base plate & solid bedrock (zero loose corners) | 4x M3 MG996 horn screws & top SG90 screws",
        },
        {
            "ax": axes[1], "elev": 32, "azim": -46, "mode": "cover_exploded",
            "title": "(B) Why Actuator Cover is Needed: 4-Way Mechanical Constraints",
            "desc": "Carriage supports downward & separating forces | Actuator Cover clamps top to prevent rack from lifting/jumping",
        },
        {
            "ax": axes[2], "elev": 32, "azim": -46, "mode": "tunnel_assembled",
            "title": "(C) Fully Enclosed 4-Sided Slide Tunnel in Action",
            "desc": "Rack is captive in all 3 axes (X sliding, Y & Z locked) | 0.4mm running clearance for silky smooth motion",
        },
    ]

    center_z = PLATE_THICKNESS + 1.0
    rack_z = center_z + MG996_TOTAL_H + 4.0 + 1.0
    s_cy = PINION_PITCH_R

    for cfg in configs:
        ax = cfg["ax"]
        ax.set_facecolor("#FFFFFF")
        all_pts = []

        def draw_m(m, color, offset=(0, 0, 0), rot_z=0.0, alpha=1.0, edge_color=None):
            vecs = m.vertices.copy()
            if rot_z != 0.0:
                rad = math.radians(rot_z)
                ca, sa = math.cos(rad), math.sin(rad)
                vx = vecs[:, 0] * ca - vecs[:, 1] * sa
                vy = vecs[:, 0] * sa + vecs[:, 1] * ca
                vecs[:, 0], vecs[:, 1] = vx, vy
            vecs += np.array(offset)

            normals = m.face_normals.copy()
            if rot_z != 0.0:
                rad = math.radians(rot_z)
                ca, sa = math.cos(rad), math.sin(rad)
                nx = normals[:, 0] * ca - normals[:, 1] * sa
                ny = normals[:, 0] * sa + normals[:, 1] * ca
                normals[:, 0], normals[:, 1] = nx, ny

            fc = [shade(color, n, alpha) for n in normals]
            lw = 0.25 if edge_color else 0.0
            coll = Poly3DCollection(vecs[m.faces], facecolor=fc, edgecolor=edge_color, linewidths=lw)
            ax.add_collection3d(coll)
            all_pts.append(vecs.reshape(-1, 3))

        if cfg["mode"] == "foundation":
            draw_m(carriage_mesh, "#512DA8", alpha=0.92)
            draw_m(mg996_mesh, "#212121", offset=(-10.35, s_cy, center_z + MG996_H / 2.0 + 4.0), alpha=0.88)
            for xp in [14.0, -34.0]:
                for yo in [-5.0, 5.0]:
                    ax.plot([xp, xp], [s_cy + yo, s_cy + yo], [center_z + 24, center_z + 44], color="#B0BEC5", linewidth=3.5)
            ax.plot([-45, 30], [-8, -8], [rack_z - 4.9, rack_z - 4.9], color="#00E676", linewidth=3.5, linestyle="-")
            ax.text(-25, 2, rack_z - 2.0, "RIGID RACK FLOOR", color="#00C853", fontsize=9.2, weight="bold")
            ax.text(-32, s_cy + 8, center_z + 46, "4x M3/M4 FLANGE SCREWS", color="#1565C0", fontsize=9.0, weight="bold")

        elif cfg["mode"] == "cover_exploded":
            draw_m(carriage_mesh, "#512DA8", alpha=0.88)
            draw_m(mg996_mesh, "#212121", offset=(-10.35, s_cy, center_z + MG996_H / 2.0 + 4.0), alpha=0.85)
            draw_m(pinion_mesh, "#FFD600", offset=(0, s_cy, rack_z), alpha=0.95, edge_color="#B7950B")
            draw_m(rack_mesh, "#E65100", offset=(15.0, 0, rack_z), alpha=0.95, edge_color="#A04000")
            draw_m(cover_mesh, "#0288D1", offset=(-3.0, -8.0, rack_z + RACK_HEIGHT / 2.0 + 18.0), alpha=0.85)
            ax.plot([-27.0, -27.0], [-19.0, -19.0], [rack_z + 4.0, rack_z + 24.0], color="#0288D1", linestyle="--", linewidth=2.0)
            ax.plot([15.0, 15.0], [-19.0, -19.0], [rack_z + 4.0, rack_z + 24.0], color="#0288D1", linestyle="--", linewidth=2.0)
            # Force annotations:
            ax.quiver(0, 0, rack_z + 7, 0, 0, -8, color="#00C853", linewidth=3.0, arrow_length_ratio=0.3)
            ax.text(2, 2, rack_z + 3, "F_down (Mesh Thrust) -> Center Pillar", color="#00C853", fontsize=8.2, weight="bold")
            ax.quiver(0, 0, rack_z, 0, -8, 0, color="#7B1FA2", linewidth=3.0, arrow_length_ratio=0.3)
            ax.text(2, -14, rack_z, "F_sep (20°) -> 8mm Back Wall", color="#7B1FA2", fontsize=8.2, weight="bold")
            ax.quiver(-3, -8, rack_z + 18, 0, 0, -8, color="#0288D1", linewidth=3.0, arrow_length_ratio=0.3)
            ax.text(-38, -12, rack_z + 21, "Cover Clamps Upward Lift & Tooth Jump!", color="#0288D1", fontsize=8.5, weight="bold")

        elif cfg["mode"] == "tunnel_assembled":
            draw_m(carriage_mesh, "#512DA8", alpha=0.90)
            draw_m(mg996_mesh, "#212121", offset=(-10.35, s_cy, center_z + MG996_H / 2.0 + 4.0), alpha=0.88)
            draw_m(pinion_mesh, "#FFD600", offset=(0, s_cy, rack_z), alpha=0.95, edge_color="#B7950B")
            draw_m(rack_mesh, "#E65100", offset=(35.0, 0, rack_z), alpha=0.95, edge_color="#A04000")
            draw_m(cover_mesh, "#0288D1", offset=(-3.0, -8.0, rack_z + RACK_HEIGHT / 2.0 + 2.0), alpha=0.80)
            ax.plot([35.0 + RACK_LENGTH / 2.0 + 5.0, 35.0 + RACK_LENGTH / 2.0 + 54.0], [0, 0], [rack_z, rack_z], color="#00E676", linewidth=3.5)

        pts = np.concatenate(all_pts, axis=0)
        mins = pts.min(axis=0)
        maxs = pts.max(axis=0)
        centers = (mins + maxs) / 2.0
        max_range = (maxs - mins).max() / 2.0 * 1.05

        ax.set_xlim(centers[0] - max_range, centers[0] + max_range)
        ax.set_ylim(centers[1] - max_range, centers[1] + max_range)
        ax.set_zlim(centers[2] - max_range, centers[2] + max_range)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=cfg["elev"], azim=cfg["azim"])
        ax.set_axis_off()
        ax.set_title(cfg["title"], fontsize=12.0, weight="bold", pad=12, color="#1A1A1A")
        ax.text2D(0.5, -0.04, cfg["desc"], transform=ax.transAxes, fontsize=9.2, ha="center", color="#555550")

    legend_items = [
        ("Solid Foundation Floor: Supports downward gear thrust (Z locked)", "#00C853"),
        ("Solid 8mm Back Wall: Resists 20° gear separating force (Y locked)", "#512DA8"),
        ("Actuator Cover Clip: Prevents upward rack lifting & tooth jump (Z+ locked)", "#0288D1"),
        ("Open Gear Mesh Window: Direct conjugate engagement with 34T Pinion", "#FFD600"),
    ]
    for i, (label, color) in enumerate(legend_items):
        x = 0.04 + (i % 2) * 0.48
        y = 0.965 if i < 2 else 0.925
        fig.text(x, y, "■", fontsize=11, color=color)
        fig.text(x + 0.012, y, label, fontsize=9.0, color="#2D2D2A")

    detail_path = os.path.join(IMG_DIR, "actuator_cover_and_support_detail.png")
    fig.savefig(detail_path, bbox_inches="tight", facecolor="#FFFFFF")
    print(f"✓ Saved actuator cover and support detail preview: {detail_path}")


def render_groove_detail_closeup():
    """Dedicated high-resolution visualizer directly addressing the user's groove & rim feedback:
       - User's New Request: Top outer rim corner curved with smooth R = 1.2mm circular fillet.
       - Green Circle Fix: Smooth circular curved fillet (R = 0.8mm) along top divider lips so wires glide effortlessly.
       - Red Circle Revert: Reverted back to stable straight rectangular slot (zero bottom circular scoops).
    """
    fig = plt.figure(figsize=(26, 8.5), dpi=220)
    fig.patch.set_facecolor("#FFFFFF")

    chassis_mesh = trimesh.load_mesh(os.path.join(STL_DIR, "base_chassis.stl"))
    mask = (
        (chassis_mesh.vertices[chassis_mesh.faces][:, :, 0] > 12.0) &
        (chassis_mesh.vertices[chassis_mesh.faces][:, :, 0] < 45.0) &
        (chassis_mesh.vertices[chassis_mesh.faces][:, :, 1] > 32.0) &
        (chassis_mesh.vertices[chassis_mesh.faces][:, :, 1] < 60.0) &
        (chassis_mesh.vertices[chassis_mesh.faces][:, :, 2] <= 6.5)
    ).all(axis=1)
    f_sub = chassis_mesh.faces[mask]

    LIGHT_DIR = np.array([0.45, -0.4, 0.78])
    LIGHT_DIR = LIGHT_DIR / np.linalg.norm(LIGHT_DIR)

    def shade(color_hex, normal, alpha=1.0):
        rgb = np.array([int(color_hex[i:i + 2], 16) for i in (1, 3, 5)]) / 255.0
        n = normal / (np.linalg.norm(normal) + 1e-9)
        diffuse = max(0.0, float(np.dot(n, LIGHT_DIR)))
        intensity = 0.38 + 0.62 * diffuse
        shaded = np.clip(rgb * intensity, 0, 1)
        return (shaded[0], shaded[1], shaded[2], alpha)

    fc_sub = [shade("#80DEEA", n) for n in chassis_mesh.face_normals[mask]]

    # Panel 1: User's Screenshot Perspective (elev=36, azim=52)
    ax1 = fig.add_subplot(131, projection="3d")
    ax1.set_facecolor("#FFFFFF")
    coll1 = Poly3DCollection(chassis_mesh.vertices[f_sub], facecolor=fc_sub, edgecolor="#00838F", linewidths=0.25)
    ax1.add_collection3d(coll1)
    ax1.set_xlim(15, 45)
    ax1.set_ylim(35, 60)
    ax1.set_zlim(0, 6)
    ax1.view_init(elev=36, azim=52)
    ax1.set_axis_off()
    ax1.set_title("(A) User Perspective: Curved Top Outer Rim Corner (R=1.2mm)", fontsize=11.5, weight="bold", pad=12, color="#1A1A1A")
    ax1.text2D(0.5, -0.04, "Matches your uploaded screenshot angle | Outer rim corner now smoothly curved with R=1.2mm", transform=ax1.transAxes, fontsize=8.8, ha="center", color="#555550")
    ax1.text(30.0, 50.0, 5.5, "CURVED RIM CORNER (R=1.2mm)", color="#00838F", fontsize=9.2, weight="bold")

    # Panel 2: Top Plate Perspective Showing Matching Curved Outer Rim
    ax2 = fig.add_subplot(132, projection="3d")
    ax2.set_facecolor("#FFFFFF")
    top_mesh = trimesh.load_mesh(os.path.join(STL_DIR, "top_plate.stl"))
    mask_top = (
        (top_mesh.vertices[top_mesh.faces][:, :, 0] > 12.0) &
        (top_mesh.vertices[top_mesh.faces][:, :, 0] < 45.0) &
        (top_mesh.vertices[top_mesh.faces][:, :, 1] > 32.0) &
        (top_mesh.vertices[top_mesh.faces][:, :, 1] < 60.0) &
        (top_mesh.vertices[top_mesh.faces][:, :, 2] >= 62.0)
    ).all(axis=1)
    f_sub_top = top_mesh.faces[mask_top]
    fc_sub_top = [shade("#81C784", n) for n in top_mesh.face_normals[mask_top]]
    coll2 = Poly3DCollection(top_mesh.vertices[f_sub_top], facecolor=fc_sub_top, edgecolor="#2E7D32", linewidths=0.25)
    ax2.add_collection3d(coll2)
    ax2.set_xlim(15, 45)
    ax2.set_ylim(35, 60)
    ax2.set_zlim(62, 69)
    ax2.view_init(elev=36, azim=52)
    ax2.set_axis_off()
    ax2.set_title("(B) Top Plate: Matching Curved Outer Rim (R=1.2mm)", fontsize=11.5, weight="bold", pad=12, color="#1A1A1A")
    ax2.text2D(0.5, -0.04, "Top plate outer rim corner is also curved with R=1.2mm | Underside clips match base grooves", transform=ax2.transAxes, fontsize=8.8, ha="center", color="#555550")
    ax2.text(30.0, 50.0, 68.2, "TOP PLATE CURVED RIM (R=1.2mm)", color="#1B5E20", fontsize=9.2, weight="bold")

    # Panel 3: 2D Engineering Cross-Section Profile
    ax3 = fig.add_subplot(133)
    ax3.set_facecolor("#FAFAFA")
    ax3.set_aspect("equal")

    slot_w = GROOVE_WIDTH
    r_fillet = 0.8
    h_plate = PLATE_THICKNESS

    for cy in [-5.0, 0.0, 5.0]:
        ax3.plot([cy - slot_w / 2.0, cy + slot_w / 2.0], [0, 0], color="#D32F2F", linewidth=3.5, label="Stable Flat Base (Red Revert)" if cy == 0 else "")
        ax3.plot([cy - slot_w / 2.0, cy - slot_w / 2.0], [0, h_plate - r_fillet], color="#2E7D32", linewidth=2.5)
        ax3.plot([cy + slot_w / 2.0, cy + slot_w / 2.0], [0, h_plate - r_fillet], color="#2E7D32", linewidth=2.5)
        th = np.linspace(0, np.pi / 2.0, 20)
        fx_l = (cy - slot_w / 2.0 - r_fillet) + r_fillet * np.cos(th)
        fz_l = (h_plate - r_fillet) + r_fillet * np.sin(th)
        ax3.plot(fx_l, fz_l, color="#00C853", linewidth=3.5, label="Curved Divider Lip (Green Fix)" if cy == 0 else "")
        fx_r = (cy + slot_w / 2.0 + r_fillet) - r_fillet * np.cos(th)
        fz_r = (h_plate - r_fillet) + r_fillet * np.sin(th)
        ax3.plot(fx_r, fz_r, color="#00C853", linewidth=3.5)

    # Top surface between slots
    ax3.plot([-8.0, -5.0 - slot_w / 2.0 - r_fillet], [h_plate, h_plate], color="#2E7D32", linewidth=2.5)
    ax3.plot([-5.0 + slot_w / 2.0 + r_fillet, 0.0 - slot_w / 2.0 - r_fillet], [h_plate, h_plate], color="#2E7D32", linewidth=2.5)
    ax3.plot([0.0 + slot_w / 2.0 + r_fillet, 5.0 - slot_w / 2.0 - r_fillet], [h_plate, h_plate], color="#2E7D32", linewidth=2.5)

    # Outer rim corner curves (BOTH outer and inside):
    r_rim_out = 1.2
    r_rim_in = 0.8
    # Top of outer rim has outer curve and inside curve
    rim_base_y = 6.2
    th_in = np.linspace(np.pi / 2.0, np.pi, 20)
    in_y = (rim_base_y + r_rim_in) + r_rim_in * np.cos(th_in)
    in_z = (h_plate - r_rim_in) + r_rim_in * np.sin(th_in)

    th_out = np.linspace(0, np.pi / 2.0, 25)
    out_y = (rim_base_y + 1.2) + r_rim_out * np.cos(th_out)
    out_z = (h_plate - r_rim_out) + r_rim_out * np.sin(th_out)

    ax3.plot([rim_base_y, rim_base_y], [0, h_plate - r_rim_in], color="#0288D1", linewidth=2.5)
    ax3.plot(in_y, in_z, color="#FF6F00", linewidth=3.5, label="Inside Rim Fillet (R=0.8mm)")
    ax3.plot([rim_base_y + r_rim_in, rim_base_y + 1.2], [h_plate, h_plate], color="#0288D1", linewidth=2.5)
    ax3.plot(out_y, out_z, color="#0288D1", linewidth=3.5, label="Outer Rim Fillet (R=1.2mm)")
    ax3.plot([rim_base_y + 1.2 + r_rim_out, rim_base_y + 1.2 + r_rim_out], [0, h_plate - r_rim_out], color="#0288D1", linewidth=2.5)
    ax3.plot([rim_base_y, rim_base_y + 1.2 + r_rim_out], [0, 0], color="#D32F2F", linewidth=2.5)

    ax3.annotate("INSIDE RIM CURVE:\nR=0.8mm Fillet\nSmooth wire exit!",
                 xy=(rim_base_y + 0.1, h_plate - 0.2), xytext=(3.0, h_plate + 2.0),
                 arrowprops=dict(facecolor="#FF6F00", shrink=0.08, width=1.5, headwidth=6),
                 fontsize=8.5, weight="bold", color="#E65100")

    ax3.annotate("OUTER RIM CURVE:\nR=1.2mm Fillet",
                 xy=(rim_base_y + 1.2 + r_rim_out * 0.7, h_plate - r_rim_out * 0.3), xytext=(6.5, h_plate + 1.5),
                 arrowprops=dict(facecolor="#0288D1", shrink=0.08, width=1.5, headwidth=6),
                 fontsize=8.5, weight="bold", color="#01579B")

    ax3.annotate("GREEN FIX:\nR=0.8mm Divider Fillet",
                 xy=(-slot_w / 2.0 - 0.2, h_plate - 0.1), xytext=(-5.8, h_plate + 1.8),
                 arrowprops=dict(facecolor="#00C853", shrink=0.08, width=1.5, headwidth=6),
                 fontsize=8.5, weight="bold", color="#1B5E20")

    ax3.annotate("RED REVERT:\nStable straight 90° corner",
                 xy=(0.0, 0.0), xytext=(-4.5, -2.2),
                 arrowprops=dict(facecolor="#D32F2F", shrink=0.08, width=1.5, headwidth=6),
                 fontsize=8.5, weight="bold", color="#B71C1C")

    ax3.set_xlim(-9, 10.5)
    ax3.set_ylim(-3.5, 8.0)
    ax3.set_xlabel("Y: Slot Width Offset (mm)", fontsize=9.5, weight="bold")
    ax3.set_ylabel("Z: Plate Height (mm)", fontsize=9.5, weight="bold")
    ax3.set_title("(C) 2D Profile: Rim Fillet vs Divider Lip vs Flat Base", fontsize=11.5, weight="bold", pad=12, color="#1A1A1A")
    ax3.grid(True, linestyle="--", alpha=0.5)
    ax3.legend(loc="upper left", fontsize=8.2)

    groove_path = os.path.join(IMG_DIR, "groove_detail_closeup.png")
    fig.savefig(groove_path, bbox_inches="tight", facecolor="#FFFFFF")
    print(f"✓ Saved groove detail closeup preview: {groove_path}")


def render_spindle_pivot_detail():
    """Dedicated explanatory visualizer explaining:
       (A) Spindle Carriage Underside: 100% Solid Continuous 5mm Base Plate & Integrated Lower Spindle Pin (Zero loose corners!).
       (B) Pinion Gear 4x M3 Servo Plate Screws: PCD=14mm bolt pattern to fasten 34T pinion directly to MG996 metal round horn.
       (C) Top Turret SG90 Servo Horn Mount: 4x M2 screw pilot tap holes to physically bolt the SG90 horn to the carriage.
    """
    fig = plt.figure(figsize=(26, 8.5), dpi=220)
    fig.patch.set_facecolor("#FFFFFF")

    LIGHT_DIR = np.array([0.45, -0.4, 0.78])
    LIGHT_DIR = LIGHT_DIR / np.linalg.norm(LIGHT_DIR)

    def shade(color_hex, normal, alpha=1.0):
        rgb = np.array([int(color_hex[i:i + 2], 16) for i in (1, 3, 5)]) / 255.0
        n = normal / (np.linalg.norm(normal) + 1e-9)
        diffuse = max(0.0, float(np.dot(n, LIGHT_DIR)))
        intensity = 0.38 + 0.62 * diffuse
        shaded = np.clip(rgb * intensity, 0, 1)
        return (shaded[0], shaded[1], shaded[2], alpha)

    carriage = trimesh.load_mesh(os.path.join(STL_DIR, "spindle_carriage.stl"))
    chassis = trimesh.load_mesh(os.path.join(STL_DIR, "base_chassis.stl"))
    bushing = trimesh.load_mesh(os.path.join(STL_DIR, "spindle_bushing.stl"))
    pinion = trimesh.load_mesh(os.path.join(STL_DIR, "pinion_gear.stl"))

    axes = [
        fig.add_subplot(131, projection="3d"),
        fig.add_subplot(132, projection="3d"),
        fig.add_subplot(133, projection="3d"),
    ]

    configs = [
        {
            "ax": axes[0], "elev": -28, "azim": 45, "mode": "underside_solid",
            "title": "(A) Spindle Carriage Underside: 100% Solid Base (Zero Loose Corners)",
            "desc": "Continuous 5mm base plate | Integrated Ø3.9mm pivot pin & Ø6.5mm thrust collar | Monolithic rigidity",
        },
        {
            "ax": axes[1], "elev": 55, "azim": -45, "mode": "pinion_screws",
            "title": "(B) 34T Pinion: 4x M3 Screws for MG996 Metal Servo Plate",
            "desc": "Standard PCD=14mm M3 screw holes bolt gear directly to MG996 metal plate | 4x outer web lightening holes",
        },
        {
            "ax": axes[2], "elev": 45, "azim": 50, "mode": "top_sg90_screws",
            "title": "(C) Carriage Top: 4x M2 Pilot Screws for SG90 Servo Horn",
            "desc": "SG90 horn bolts directly to carriage via 4x M2 screws | Prevents carriage from loosely hanging or wobbling",
        },
    ]

    for cfg in configs:
        ax = cfg["ax"]
        ax.set_facecolor("#FFFFFF")
        all_pts = []

        def draw_m(m, color, offset=(0, 0, 0), rot_z=0.0, alpha=1.0, edge_color=None):
            vecs = m.vertices.copy()
            if rot_z != 0.0:
                rad = math.radians(rot_z)
                ca, sa = math.cos(rad), math.sin(rad)
                vx = vecs[:, 0] * ca - vecs[:, 1] * sa
                vy = vecs[:, 0] * sa + vecs[:, 1] * ca
                vecs[:, 0], vecs[:, 1] = vx, vy
            vecs += np.array(offset)

            normals = m.face_normals.copy()
            if rot_z != 0.0:
                rad = math.radians(rot_z)
                ca, sa = math.cos(rad), math.sin(rad)
                nx = normals[:, 0] * ca - normals[:, 1] * sa
                ny = normals[:, 0] * sa + normals[:, 1] * ca
                normals[:, 0], normals[:, 1] = nx, ny

            fc = [shade(color, n, alpha) for n in normals]
            lw = 0.25 if edge_color else 0.0
            coll = Poly3DCollection(vecs[m.faces], facecolor=fc, edgecolor=edge_color, linewidths=lw)
            ax.add_collection3d(coll)
            all_pts.append(vecs.reshape(-1, 3))

        if cfg["mode"] == "underside_solid":
            draw_m(carriage, "#512DA8", alpha=0.92)
            ax.quiver(0, 0, -8, 0, 0, 7, color="#E91E63", linewidth=3.5, arrow_length_ratio=0.3)
            ax.text(2, -10, -6, "INTEGRATED SPINDLE PIN\n(Ø3.9mm x 3.8mm L)", color="#E91E63", fontsize=9.2, weight="bold")
            ax.text(-25, 18, 5, "SOLID 5mm CONTINUOUS FLOOR\n(Zero Cutouts / Zero Loose Corners)", color="#00C853", fontsize=8.8, weight="bold")
            ax.text(-38, -12, 12, "43mm Solid Bedrock", color="#1565C0", fontsize=8.8, weight="bold")

        elif cfg["mode"] == "pinion_screws":
            draw_m(pinion, "#FFD600", alpha=0.95, edge_color="#B7950B")
            # Draw circles and text pointing to 4x M3 screw holes
            for i in range(4):
                ang = i * (math.pi / 2.0)
                sx = 7.0 * math.cos(ang)
                sy = 7.0 * math.sin(ang)
                ax.plot([sx, sx], [sy, sy], [4.0, 10.0], color="#E91E63", linewidth=2.5)
            ax.text(0, -18, 8, "4x M3 SCREW HOLES\n(PCD = 14.0mm for Metal Horn)", color="#E91E63", fontsize=9.2, weight="bold", ha="center")
            ax.text(0, 18, 8, "M3 Center Spline Bolt", color="#1565C0", fontsize=8.5, weight="bold", ha="center")

        elif cfg["mode"] == "top_sg90_screws":
            draw_m(carriage, "#512DA8", alpha=0.90)
            center_z = PLATE_THICKNESS + 1.0
            top_z = center_z + CLEARANCE_H - 2.0
            for xp, yp in [(-7.5, 0.0), (7.5, 0.0), (0.0, -7.5), (0.0, 7.5)]:
                ax.plot([xp, xp], [yp, yp], [top_z - 2.0, top_z + 8.0], color="#00E676", linewidth=2.5)
            ax.text(0, 0, top_z + 10.0, "4x M2 PILOT SCREWS\nBolts SG90 Horn to Carriage!", color="#00E676", fontsize=9.2, weight="bold", ha="center")
            ax.text(0, -16, top_z - 4.0, "SG90 Horn Arm Pocket", color="#FFD600", fontsize=8.5, weight="bold")

        pts = np.concatenate(all_pts, axis=0)
        mins = pts.min(axis=0)
        maxs = pts.max(axis=0)
        centers = (mins + maxs) / 2.0
        max_range = (maxs - mins).max() / 2.0 * 1.05

        ax.set_xlim(centers[0] - max_range, centers[0] + max_range)
        ax.set_ylim(centers[1] - max_range, centers[1] + max_range)
        ax.set_zlim(centers[2] - max_range, centers[2] + max_range)
        ax.set_box_aspect((1, 1, 1))
        ax.view_init(elev=cfg["elev"], azim=cfg["azim"])
        ax.set_axis_off()
        ax.set_title(cfg["title"], fontsize=12.0, weight="bold", pad=12, color="#1A1A1A")
        ax.text2D(0.5, -0.04, cfg["desc"], transform=ax.transAxes, fontsize=9.2, ha="center", color="#555550")

    pivot_path = os.path.join(IMG_DIR, "spindle_pivot_detail.png")
    fig.savefig(pivot_path, bbox_inches="tight", facecolor="#FFFFFF")
    print(f"✓ Saved spindle pivot detail preview: {pivot_path}")


if __name__ == "__main__":
    build_all()

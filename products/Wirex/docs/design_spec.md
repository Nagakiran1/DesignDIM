# Wirex Design Specification (Mounting Base & 180° Direct Pinion Edition)

## 1. Geometric & Kinematic Architecture

Total 360° allocation:
$$\mathbf{130^\circ \text{ (Wires)}} + \mathbf{50^\circ \text{ (Left Wall)}} + \mathbf{130^\circ \text{ (Rear Open Swing)}} + \mathbf{50^\circ \text{ (Right Wall)}} = \mathbf{360^\circ}$$

| Parameter | Specification | Engineering Rationale |
| :--- | :--- | :--- |
| **Number of Channels** | **25 positions** (wires 0 to 24) | Accommodates 25 distinct wires |
| **Channel Spacing** | **$5.0\text{ mm}$ pitch** | Linear arc distance between adjacent wire centers |
| **Wire Arc Sector** | **$130.0^\circ$** ($2.2689\text{ rad}$) | $0^\circ \to 130^\circ$ front fan sector |
| **Derived Pitch Radius** | **$52.89\text{ mm}$** | $R = \frac{24 \times 5.0\text{ mm}}{130^\circ \times (\pi / 180)} \approx 52.89\text{ mm}$ |
| **Outer Plate Rim** | **$58.4\text{ mm}$** | Outer perimeter enclosing all wire pockets |
| **Outer Line Rim** | **$2.2\text{ mm}$ thickness** | Encloses the wire teeth into captive pockets |
| **Wire Pocket Geometry** | **Curved Funnel Pointers ($R=1.8\text{mm}$) & Rounded Lips** | Tapered bullet-nose separators with rounded entry lips eliminate wire pull/leave friction |
| **Flush Bottom Base** | **100% Flat ($Z=0$)** | Zero under-mount protrusion; sits completely flush against flat wall or table |
| **Servo 2 Mount** | **Top Plate Integrated Bracket** | Inverted SG90/MG90S coaxial with pivot $(0,0)$, easily accessible from above |
| **External Mounting Feet** | **4x Heavy-Duty Lugs @ $R = 74.0\text{ mm}$** | Angles: $138^\circ, 174^\circ, 316^\circ, 352^\circ$; M4 clearance bores ($4.5\text{ mm}$) with $8.5\text{ mm}$ counterbore ($2.8\text{ mm}$ depth) |
| **Mounting Gusset Ribs** | **$22.0\text{ mm}$ tall $\times 5.0\text{ mm}$ thick** | Directly fused to $50^\circ$ walls; eliminates bending moment under wire pull tension; zero disconnected parts |
| **Left Support Wall** | **$130^\circ \to 180^\circ$** ($50^\circ$ span) | $5.5\text{ mm}$ thick curved upright wall |
| **Left Support Link** | Positioned at **$155.0^\circ$**, $R=49.0\text{ mm}$ | Vertical pillar with M3 screw bore |
| **Rear Open Sector** | **$180^\circ \to 310^\circ$** ($130^\circ$ span) | 100% open clearance (recessed to $R=28\text{ mm}$); no 3rd support |
| **Right Support Wall** | **$310^\circ \to 360^\circ$** ($50^\circ$ span) | $5.5\text{ mm}$ thick curved upright wall |
| **Right Support Link** | Positioned at **$335.0^\circ$**, $R=49.0\text{ mm}$ | Vertical pillar with M3 screw bore |
| **Total Drum Height** | **$68.0\text{ mm}$** | Internal clearance of $58.0\text{ mm}$ for MG996R carriage |

---

## 2. Decoupled Servo Kinematics & 180° Direct Drive Calculations

### Servo 1 — Linear Pusher Actuator (MG996R with D=42.5mm Direct Pinion)
- Standard metal-gear servo ($180^\circ$ rotation).
- **Stall Torque:** $11.0\text{ kg}\cdot\text{cm}$ at 4.8V; $13.5\text{ kg}\cdot\text{cm}$ at 6.0V.
- **Pinion Gear:** 34-tooth module-1.25, pitch radius $R_p = 21.25\text{ mm} = 2.125\text{ cm}$ (Pitch diameter $D = 42.5\text{ mm}$).
- **Linear Stroke for 180° Sweep:**
  $$\Delta X = R_p \times \pi = 21.25\text{ mm} \times 3.14159 = \mathbf{66.76\text{ mm} \ (\approx 6.68\text{ cm})}$$
- **Linear Push Force:**
  $$F = \frac{\tau}{R_p} = \frac{11.0\text{ kg}\cdot\text{cm}}{2.125\text{ cm}} = \mathbf{5.18\text{ kgf} \ (\approx 50.8\text{ N})}$$
- **Linear Stroke Positions:**
  - **Rack Length:** **$125.0\text{ mm}$**.
  - **Retracted Position:** Tip at **$R = 43.0\text{ mm}$** ($5.2\text{ mm}$ safety clearance inside the wire pocket base at $R = 48.2\text{ mm}$).
  - **Extended Position:** Tip reaches **$R = 109.8\text{ mm}$** (**$5.14\text{ cm}$ push beyond the $R = 58.4\text{ mm}$ outer rim**).

### Mounting Stability & Rigidity
- The 4 external mounting tabs distribute clamping force over a broad base footprint ($144\text{ mm} \times 144\text{ mm}$ bolt circle).
- Vertical gussets lock the base chassis directly to the mounting surface, ensuring zero tilt or deformation even if an external puller exerts over $10\text{ kgf}$ on the wires.
- The rear swing sector ($Z > 6\text{ mm}$) remains completely open air.

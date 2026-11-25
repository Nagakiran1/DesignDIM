# Wirex Bill of Materials (BOM) — Mounting Base Edition

## 1. 3D Printed Components (Material: PLA or PETG)

| Part Name | File | Layer Height | Infill | Notes |
| :--- | :--- | :---: | :---: | :--- |
| **Base Chassis** | `base_chassis.stl` | $0.20\text{ mm}$ | 30% | Monolithic unit with 4 mounting feet; print flat on base |
| **Top Plate** | `top_plate.stl` | $0.20\text{ mm}$ | 25% | Print top face down |
| **Spindle Carriage** | `spindle_carriage.stl` | $0.16\text{ mm}$ | 35% | Lightweight skeletonized turret with integrated lower pivot pin |
| **Pusher Rack** | `pusher_rack.stl` | $0.16\text{ mm}$ | 100% | High strength for module-1.25 teeth & head |
| **Pinion Gear** | `pinion_gear.stl` | $0.12\text{ mm}$ | 100% | 34-tooth module-1.25 ($D=42.5\text{mm}$) with 4 lightening holes |
| **Actuator Cover** | `actuator_cover.stl` | $0.20\text{ mm}$ | 50% | Heavy-duty rack guide retainer clip |
| **Spindle Bushing** | `spindle_bushing.stl` | $0.16\text{ mm}$ | 100% | Snap-in bushing for Ø10mm pocket (if 624ZZ bearing is not used) |

---

## 2. Electro-Mechanical & Fasteners

| Item | Qty | Specifications | Approximate Cost |
| :--- | :---: | :--- | :--- |
| **Servo 1 (Linear Actuator)** | 1 | **MG996R** Standard Metal-Gear Servo ($180^\circ$ rotation, $11\text{ kg}\cdot\text{cm}$) | ~$8.00 |
| **Servo 2 (Turret Indexing)** | 1 | **SG90 or MG90S** 9g Micro Servo ($180^\circ$ rotation) | ~$2.50 |
| **Base Mounting Screws** | 4 | **$\text{M4} \times 16\text{–}25\text{ mm}$** machine screws or bolts with nuts/washers | ~$0.60 |
| **Top Plate Screws** | 6 | **$\text{M3} \times 12\text{ mm}$** buttonhead or socket head screws | ~$0.60 |
| **Pivot Dowel Pin** | 1 | $\varnothing 4.0\text{ mm} \times 66\text{ mm}$ precision steel pin (or M4 bolt) | ~$0.60 |
| **Ball Bearings** | 2 | 624ZZ ($\varnothing 4\text{mm ID} \times \varnothing 10\text{mm OD} \times 4\text{mm W}$) | ~$1.50 |
| **Servo Mounting Screws** | 8 | $\text{M3} \times 10\text{ mm}$ (for MG996R ears) and $\text{M2} \times 8\text{ mm}$ (for SG90 ears & cover) | ~$0.80 |
| **Servo Plate Screws** | 8 | 4x $\text{M3} \times 6\text{–}8\text{ mm}$ (pinion to MG996 metal plate) & 4x $\text{M2} \times 6\text{–}8\text{ mm}$ (carriage to SG90 horn) | ~$0.80 |
| **Total Estimated Cost** | | | **~$14.60** |

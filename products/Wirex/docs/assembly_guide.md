# Wirex Assembly Guide

## Part List (Printable STLs in `../stl/`)

| Part | File | Qty | Description |
| :--- | :--- | :---: | :--- |
| **Base Chassis** | `base_chassis.stl` | 1 | 100% Flat monolithic bottom plate with integrated walls, links, outer line, and 4x M4 counterbored mounting feet |
| **Top Plate** | `top_plate.stl` | 1 | Screw-down top plate with matching curved wire clips and integrated top-mount bracket for Servo 2 |
| **Spindle Carriage** | `spindle_carriage.stl` | 1 | Open-cradle rotating turret housing MG996R with 4x accessible screw stanchions and top horn socket |
| **Pusher Rack** | `pusher_rack.stl` | 1 | 125mm module-1.25 toothed rack with radiused wire cradle head |
| **Pinion Gear** | `pinion_gear.stl` | 1 | 34-tooth module-1.25 direct 180° pinion ($D=42.5\text{ mm}$) with 4 lightening holes |
| **Actuator Cover** | `actuator_cover.stl` | 1 | Guide retainer clip holding the rack in mesh with the pinion |
| **Spindle Bushing** | `spindle_bushing.stl` | 1 | Optional 3D printable snap-in pivot bushing (if 624ZZ bearing is not used) |

## Hardware Required

- **1x MG996R Standard Metal-Gear Servo** (Servo 1: Linear Actuator, $180^\circ$ rotation, $>5\text{ kgf}$ thrust)
- **1x SG90 or MG90S Micro Servo** (Servo 2: Turret Rotation across $130^\circ$ arc, mounted on Top Plate)
- **1x 4mm Steel Dowel Pin or M4 Bolt** (Length: $66\text{ mm}$, central spindle pivot shaft)
- **2x 624ZZ Miniature Ball Bearings** ($4\text{ mm ID} \times 10\text{ mm OD} \times 4\text{ mm W}$)
- **4x M4 x 16–25mm Socket Head Cap Screws (DIN 912) or Wood Screws** (for bolting flush base chassis to table/wall)
- **6x M3 x 12mm Flathead/Buttonhead Screws** (for securing the top plate to the chassis uprights and pillars)
- **4x M3 or M4 x 10–12mm Screws** (for mounting MG996R servo ears to spindle carriage stanchions)
- **2x M2 x 8mm Screws** (for mounting SG90 servo ears onto top plate bracket)
- **2x M3 x 8mm Screws** (for securing `actuator_cover.stl`)

---

## Step-by-Step Assembly Instructions

### Step 1: Base Chassis Mounting & Pivot Bearing/Bushing
1. Position `base_chassis.stl` directly against the mounting surface (wall, tabletop, or machine fixture).
2. Because the bottom plane is **100% flat and flush** (zero under-mount protrusion), it mates with zero gap.
3. Fasten using 4x M4 socket head cap screws through the counterbored mounting lugs at angles $138^\circ, 174^\circ, 316^\circ, 352^\circ$.
4. **Pivot Support Installation**:
   - If using ball bearings: Press a standard **624ZZ ball bearing** into the recessed $\varnothing 10\text{ mm}$ pocket on the upper face of the base pivot hub.
   - If 100% 3D printed: Press the printable **`spindle_bushing.stl`** into the pocket.

### Step 2: Open-Cradle Spindle Carriage Subassembly (MG996R)
1. Take `spindle_carriage.stl`. Notice the **open-cradle design**: the servo bay is completely open from the side and rear.
2. Drop the **MG996R servo** horizontally into the cradle, seating its base on the bottom support shelf.
3. Align the servo flange ears over the front and rear mounting stanchions.
4. Using a screwdriver or hex key with **100% direct line-of-sight access**, drive 4x M3 or M4 screws straight down into the pilot tap holes.
5. Route the 3-pin servo cable through the built-in cable relief notch at the bottom of the front stanchion.
6. **Fasten Pinion to MG996 Metal Horn Plate**:
   - Seat the MG996 circular metal servo horn plate into the recessed pocket on `pinion_gear.stl`.
   - Drive **4x M3 x 6–8mm screws** through the gear's clearance holes into the tapped holes of the metal horn plate (PCD = 14mm).
   - Fasten the center M3 bolt into the MG996 output spline. This forms a permanent, zero-slip mechanical drive connection.
7. Slide `pusher_rack.stl` into the radial guide channel, engaging the teeth with the pinion gear.
8. Place `actuator_cover.stl` over the channel and secure with 2x M3x8mm screws.

### Step 3: Central Spindle Installation
1. Lower the assembled `spindle_carriage.stl` into the base chassis:
   - The **integrated lower spindle pivot pin** ($\varnothing 3.9\text{ mm} \times 3.8\text{ mm}$ long) on the bottom of the carriage drops directly into the 624ZZ ball bearing (or `spindle_bushing.stl`).
   - The $\varnothing 6.5\text{ mm}$ integrated spacer collar seats onto the inner bearing ring, keeping the carriage body elevated $1.0\text{ mm}$ above the chassis floor so it spins with zero friction.
2. Optionally insert a $4\text{ mm}$ steel pivot dowel pin or long M4 bolt through the central pivot bore for maximum load rigidity.

### Step 4: Top Plate & Servo 2 Installation
1. Press the second **624ZZ ball bearing** into the underside pocket of `top_plate.stl`.
2. Lower `top_plate.stl` onto the chassis, engaging the central shaft into the bearing and resting on the side walls and pillars.
3. Fasten the top plate using 6x M3x12mm screws into the pilot tap holes of the upright side walls and support link pillars.
4. **Fasten SG90 Servo Horn to Carriage**:
   - Place the SG90 servo horn into the top socket of `spindle_carriage.stl`.
   - Drive **2x or 4x M2 x 6–8mm screws** through the horn arm holes directly into the pilot tap holes on top of the carriage. This physically locks the carriage to the servo horn, completely preventing the carriage from loosely hanging or wobbling.
5. Insert **Servo 2 (SG90/MG90S)** into the integrated top bracket, engaging its spline into the bolted horn. Fasten Servo 2 ears from above using 2x M2x8mm screws.

### Step 5: Wire Loading
1. Drop the 25 wires into the wire guide pockets along the $130^\circ$ front arc.
2. Thanks to the **curved bullet-nose pointers and rounded entry lips**, wires glide smoothly into each pocket without catching or friction.
3. Test linear stroke: when Servo 1 rotates $180^\circ$, the pusher rack travels $6.68\text{ cm}$, extending the selected wire $5.14\text{ cm}$ cleanly beyond the housing rim.

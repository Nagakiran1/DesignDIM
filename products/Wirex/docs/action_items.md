# Wirex Action Items

## Pending CAD & Hardware Updates

### 1. Push Rack Redesign (`pusher_rack.stl` in `cad/wire_selector.py`)
- [x] **Keep printed `spindle_carriage` fixed:** Preserve the existing rack bar cross-section (`8.0 mm` width / `8.75 mm` with `module = 1.25` teeth, `9.0 mm` height) so it fits the already-printed `9.8 mm` carriage channel and `D = 42.5 mm` pinion without reprinting the carriage.
- [x] **Tapered triangle pusher head & 5 cm neck (`50 mm` front section):**
  - Reduce pusher head tip from `14.0 mm × 13.0 mm` down to **`4.0 mm` wide × `4.0 mm` tall** with a shallow `R = 1.5 mm` wire cradle notch (clears `5.0 mm` groove pitch / `2.6 mm` groove width / `1.96 mm` min divider wall).
  - Taper smoothly over **`50.0 mm` (`5 cm`)** from **`4.0 mm` at the pointed tip → `6.2 mm` wide** (and `4.0 mm → 9.0 mm` tall in Z) back to the toothed rack bar (clears `-8.0 mm` carriage wall and `-1.25 mm` pinion tip on full retraction).
- [x] **Increase rack length for ≥ 10 cm actual push:**
  - Extend toothed bar to **`135.0 mm`** + **`50.0 mm`** tapered front = **`185.0 mm` total length**.
  - Target **`120.0 mm` total travel** (retracted tip at `R = 43.0 mm` → extended tip at `R = 163.0 mm`, giving **`10.46 cm` actual push** beyond the `R = 58.4 mm` outer rim; rear tail extends into the open `180°–310°` rear sector).
- [ ] **Servo drive for > 6.68 cm stroke:**
  - Swap the `180°` MG996R for a **360° continuous-rotation / multi-turn MG996R-sized servo** (same `40.7 × 20.0 × 36.5 mm` body, drops into the printed carriage) with home/end limits (~`324°` pinion rotation for `120 mm` travel).
- [x] **Regenerate & verify:** Exported `products/Wirex/stl/pusher_rack.stl` (`185.0 × 8.75 × 9.0 mm`, watertight single body, `0.0000` collision volume with `spindle_carriage` at both retracted and extended positions).


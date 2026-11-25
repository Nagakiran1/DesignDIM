# Calibration and control

## Position-to-angle mapping

25 positions, 0-indexed `i = 0..24` (wire number = `i + 1`), 7.5° spacing:

```
angle_deg(i) = i * 7.5
```

## SG90 pulse-width mapping (servo 1 / selection)

Typical SG90 pulse range is ~500–2400 µs over its ~180° of travel.
Calibrate your own servos (they vary), but as a starting point:

```
pulse_us(i) = 500 + i * (1900 / 24)   # ≈ 500 + i * 79.2
```

| Wire | i | angle | pulse (µs) |
|---|---|---|---|
| 1 | 0 | 0.0° | 500 |
| 7 | 6 | 45.0° | 975 |
| 13 | 12 | 90.0° | 1450 |
| 19 | 18 | 135.0° | 1926 |
| 25 | 24 | 180.0° | 2400 |

Measure your own servo's end-stop pulses before trusting the table —
SG90 units are not consistent unit-to-unit.

## Operating sequence

1. **Idle** — both servos at home position.
2. **Select** — controller receives a wire number (1–25); servo 1 moves
   to `pulse_us(wire_number - 1)` and holds.
3. **Align (settle)** — wait for servo 1's rated transit time
   (SG90 ≈ 0.1 s/60° → ≈ 0.3 s for the worst-case 180° sweep) before
   commanding servo 2. Don't start the pull on a servo that hasn't
   physically arrived.
4. **Grip** — servo 2 closes the gripper on the aligned wire.
5. **Pull** — servo 2 sweeps its 180° stroke, drawing the wire rearward.
6. **Hold** — the ratchet pawl (passive, mechanical) takes the holding
   load; no servo torque is relied on here.
7. **Release / reset** — servo 2 reverses, the gripper's return motion
   lifts the ratchet pawl and releases the wire, then servo 2 returns
   home. Servo 1 is now free to select a different wire.

## ESP32 control sketch (pseudocode)

```cpp
#include <ESP32Servo.h>

Servo servoSelect, servoPull;
const int PIN_SELECT = 18;
const int PIN_PULL   = 19;

int pulseForWire(int wireNumber) {           // wireNumber: 1..25
  int i = wireNumber - 1;
  return 500 + round(i * (1900.0 / 24));
}

void selectAndPull(int wireNumber) {
  servoSelect.writeMicroseconds(pulseForWire(wireNumber));
  delay(350);                                 // settle time, measure for your build

  servoPull.write(0);                         // grip
  delay(150);
  servoPull.write(180);                       // pull stroke
  delay(400);                                 // hold — ratchet takes the load here

  // ... wire is now held by the ratchet; release when commanded ...
  servoPull.write(0);                         // return stroke lifts ratchet pawl
  delay(300);
}

void setup() {
  servoSelect.attach(PIN_SELECT, 500, 2400);
  servoPull.attach(PIN_PULL, 500, 2400);
}
```

Treat the `delay()` calls as placeholders — replace with your own
timing once you've measured real transit and settle times on the
assembled prototype.

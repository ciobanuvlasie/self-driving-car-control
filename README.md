# 🏎️ self-driving-car-control

> **Bare-metal embedded C firmware for an autonomous race car — NXP Cup competition.**  
> Virtual-point lane detection · PID steering control · Real-time camera processing on NXP MCU.

---

## 🏁 About the Project

This repository contains the full firmware for an autonomous model car built for the **NXP Cup** — a worldwide student competition where teams design, build, and race a self-driving car around an unknown track as fast as possible.

The car perceives the track through a **line-scan camera**, computes a **virtual vanishing point** from detected lane lines, and drives a **PID steering controller** to keep the car centered and on-track — all running bare-metal on an NXP microcontroller.

---

## ⚙️ Hardware Stack

| Component | Details |
|---|---|
| **MCU** | NXP FRDM series FRDM-MCXN947 |
| **Camera** | Line-scan (Pixy2.1) via GPIO/ADC |
| **Actuators** | Servo + 2 x DC Motors + Drivers |
| **Debugger** | LinkServer (MCUXpresso IDE) |
| **Power** | LiPo battery |

---

## 🧠 Algorithm — Virtual Point Heading

The core navigation algorithm estimates the car's heading error using a **virtual vanishing point (VP)** — the projected intersection of detected left and right lane boundaries.

### 1. Camera Frame Analysis

Each frame from the line-scan camera is processed to extract lane edges. Lines are classified by slope sign into left/right boundaries and scored by:

- **Length** → longer lines carry more weight (main lane markings)
- **Vertical position** → lines from the **bottom of the frame** are prioritised (close range = stable, low noise)
- **Dominance** → strongest line per side is selected as the primary boundary

> Lines from the **top of the frame** have a longer prediction horizon but are noise-prone due to perspective compression near the horizon — calibration sets the VP vertical window to balance this trade-off.

```
Camera frame layout:
┌──────────────────────────────┐  ← Horizon zone: long look-ahead, high noise
│                              │
│    left line \   / right     │
│               \ /            │
│      VP ───────X─── heading  │  ← Virtual Point (calibrated zone)
│               / \            │
│              /   \           │
└──────────────────────────────┘  ← Near zone: short look-ahead, high stability
```

### 2. Line Scoring & Selection

```c
// Conceptual scoring — longer + lower in frame = higher priority
score = line_length * WEIGHT_LENGTH
      + (FRAME_HEIGHT - line_y) * WEIGHT_POSITION;
```

Short lines near frame edges are **excluded from VP calculation** but used separately for **curve anticipation** — they give early warning of upcoming turns before the dominant lines reflect the change.

### 3. Virtual Point Computation

The VP is computed as the **weighted intersection** of the best-ranked left/right line pair:

```c
// Intersection of two lines → virtual point
vp_x = (b2 - b1) / (m1 - m2);
vp_y = m1 * vp_x + b1;

// Heading error = horizontal offset from frame centre
error = vp_x - (FRAME_WIDTH / 2);
```

Edge cases handled:
- **Parallel lines** → no intersection, fallback to frame centre
- **Single side only** → mirror extrapolation from the detected side
- **VP out of calibrated window** → clamped to safe vertical bounds

### 4. PID Steering Control

The heading error drives a PID controller that outputs the servo angle:

```c
integral   += error * dt;
derivative  = (error - prev_error) / dt;

servo_output = Kp * error
             + Ki * integral
             + Kd * derivative;

prev_error = error;
```

**Tuning workflow:**
1. Mount camera at the final fixed position on the car
2. Place car at known angles relative to track (e.g. −20°, 0°, +20°)
3. Capture static frames → compute VP per angle → record expected vs actual heading
4. Adjust `Kp` until `servo_output = 0` when car is parallel to the lane group
5. Add `Kd` to reduce oscillation on straights; add `Ki` only if a persistent lateral offset remains
6. Validate **sensor repeatability before finalising gains** — sensor noise sets the tuning floor

---

## 📁 Repository Structure

```
self-driving-car-control/
├── source/                  # Application source — main loop, algorithm, control
├── include/                 # Header files
├── drivers/                 # NXP peripheral drivers (PWM, ADC, GPIO, UART)
├── board/                   # Board-specific pin configuration
├── device/                  # MCU device headers & startup
├── CMSIS/                   # ARM CMSIS core library
├── CMSIS_driver/Include/    # CMSIS driver interfaces
├── component/uart/          # UART component (serial debug / telemetry)
├── startup/                 # Reset handler & vector table
├── utilities/               # Debug utilities
├── doc/                     # Documentation & diagrams
├── Debug/                   # Build artefacts (MCUXpresso)
├── nxpcup.mex               # MCUXpresso Config Tools — pin & clock configuration
├── nxpcup LinkServer Debug.launch  # Debug launch configuration
├── .cproject / .project     # MCUXpresso IDE project files
└── README.md
```

---

## 🛠️ Build & Flash

### Requirements

- [MCUXpresso IDE](https://www.nxp.com/mcuxpresso) (v11+)
- NXP MCUXpresso SDK matching your board
- LinkServer debug probe (or J-Link / CMSIS-DAP)

### Steps

1. **Clone** the repository:
   ```bash
   git clone https://github.com/ciobanuvlasie/self-driving-car-control.git
   ```

2. **Import** into MCUXpresso IDE:  
   `File → Import → Existing Projects into Workspace` → select repo root

3. **Configure** pins & clocks if needed:  
   Open `nxpcup.mex` in MCUXpresso Config Tools → re-generate if modified

4. **Build:**  
   `Project → Build Project` (or `Ctrl+B`)

5. **Flash & Debug:**  
   Use `nxpcup LinkServer Debug.launch` → `Run → Debug` (or `F11`)

---

## 🔧 Key Tuning Parameters

| Parameter | Description |
|---|---|
| `Kp` | Proportional steering gain |
| `Ki` | Integral gain — corrects persistent lateral offset |
| `Kd` | Derivative gain — damps oscillation on straights |
| `VP_Y_MIN / VP_Y_MAX` | Calibrated vertical window for virtual point clamping |
| `WEIGHT_LENGTH` | Influence of line length on candidate scoring |
| `WEIGHT_POSITION` | Influence of vertical frame position on scoring |
| `FRAME_WIDTH / HEIGHT` | Camera resolution constants |

---

## 📐 Camera Calibration Procedure

1. Fix the camera at the exact mounting position (angle and height locked)
2. Place the car **perfectly straight** on the track → VP should land at `FRAME_WIDTH / 2`
3. Rotate the car ±15°, ±30° → record VP displacement per degree
4. Compute `pixels_per_degree` — this scale factor sets the baseline for `Kp`
5. Re-run with the motor active to detect vibration-induced pixel noise
6. Confirm VP stays within the calibrated vertical window across all expected speeds

> ⚠️ Always validate sensor repeatability **before** tuning control gains.  
> Noisy or inconsistent camera output makes gain tuning meaningless.

---

## 🏆 Competition Context — NXP Cup

The **NXP Cup** is a global student competition focused on autonomous model car racing. Teams design and program a car to complete laps on an unknown track as fast as possible. Disciplines include:

- **Sprint lap** — fastest single lap wins
- **Figure-8** — most laps completed within 60 seconds
- **Emergency braking** — car must stop before an obstacle without leaving the track

The track uses white lane markings on a dark surface. This firmware targets the lane-following core required across all disciplines.

---

## 📄 License

MIT License — see [`LICENSE`](LICENSE) for details.

---

*Built for speed. Tuned for the track. Racing the NXP Cup.*

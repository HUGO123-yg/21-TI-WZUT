# Mechanics Framework — Learnings

## 2026-05-09: Four-Bar Linkage IK Skeleton

### Files Modified
- `project/code/servo_kinematics.h`: Appended 5 mechanical parameter macros (all `(0)` with TODO) + `servo_angle_to_pwm` prototype
- `project/code/servo_kinematics.c`: Added `servo_angle_to_pwm()` helper; replaced `servo_control_table` body with cosine-law IK

### Key Decisions
- **DEG_TO_RAD dependency**: Added `#include "control.h"` to servo_kinematics.c for DEG_TO_RAD. Circular include safe (control.h also includes servo_kinematics.h, but include guards prevent recursion).
- **clip2 vs func_limit**: Used `func_limit()` (from zf_common_function.h) instead of `clip2()` (from control.h) in `servo_angle_to_pwm` since func_limit is available via zf_common_headfile.h regardless of include chain.
- **acosf domain clamping**: Manual if-checks clamp cos_theta to [-1, 1] before acosf to prevent NaN from floating-point rounding errors.
- **angle-to-PWM**: servo_angle_to_pwm applies `func_limit(pwm_f, 10000.0f)` — clamps PWM offset to ±10000 (matching servo 1000us–2000us range at ±90°).
- **Output mapping**: theta2 → out_ph1 (lower servo, PWM1), theta1 → out_ph4 (upper servo, PWM4).

### TODO Placeholders (all `(0)`)
| Macro | Meaning | Unit |
|-------|---------|------|
| L1_MM | Upper link length | mm |
| L2_MM | Lower link length | mm |
| D_MM | Servo axis spacing | mm |
| SERVO_ANGLE_MAX | Max mechanical angle (unidirectional) | deg |
| PWM_PER_DEG | PWM offset per degree | PWM/deg |

### LSP Notes
- clangd errors are pre-existing (no IAR include paths configured). All symbols resolve correctly in IAR build.

---

## 2026-05-09: F1 Review — Mechanical Parameter Framework

### Review Scope
- `project/code/servo_kinematics.h` (54 lines)
- `project/code/servo_kinematics.c` (87 lines)

### Checklist Results

| # | Check Item | Location | Result |
|---|-----------|----------|--------|
| 1 | L1_MM = (0) + TODO | .h:33 | ✅ PASS |
| 2 | L2_MM = (0) + TODO | .h:34 | ✅ PASS |
| 3 | D_MM = (0) + TODO | .h:35 | ✅ PASS |
| 4 | SERVO_ANGLE_MAX = (0) + TODO | .h:38 | ✅ PASS |
| 5 | PWM_PER_DEG = (0) + TODO | .h:39 | ✅ PASS |
| 6 | cos_theta1 clamped to [-1,1] before acosf | .c:72-73 | ✅ PASS |
| 7 | cos_theta2 clamped to [-1,1] before acosf | .c:74-75 | ✅ PASS |
| 8 | theta1_deg clamped via func_limit | .c:81 | ✅ PASS |
| 9 | theta2_deg clamped via func_limit | .c:82 | ✅ PASS |
| 10 | DEG_TO_RAD used for radian conversion | .c:59 | ✅ PASS |
| 11 | SERVO_1~4 aliases preserved | .h:12-15 | ✅ PASS |
| 12 | SERVO1_MID~SERVO4_MID aliases preserved | .h:17-20 | ✅ PASS |
| 13 | Dead zone macros preserved (×4) | .h:24-27 | ✅ PASS |
| 14 | mid_point extern preserved | .h:48 | ✅ PASS |
| 15 | No hardcoded mechanical params in servo_control_table | .c:37-87 | ✅ PASS |
| 16 | No new files created | — | ✅ PASS |

### Notes
- rad-to-deg constant (`57.29577951308232f` at .c:77-78) is a mathematical constant (180/π), not a mechanical parameter — acceptable
- `servo_angle_to_pwm` uses `10000.0f` as PWM full-scale hardware limit (not a configurable mechanical parameter) — acceptable
- LW servo output mapping: `theta2 → out_ph1` (lower), `theta1 → out_ph4` (upper) — correct per four-bar geometry
- All mechanical parameters in `servo_control_table` reference macros (`L1_MM`, `L2_MM`, `D_MM`, `SERVO_ANGLE_MAX`) — zero hardcoded values

### Verdict
```
Params [PASS] | Safety [PASS] | Regression [PASS] | VERDICT: APPROVE
```

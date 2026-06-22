# PATH PLANNING TECHNICAL DOCS

**Generated:** 2026-06-19
**Scope:** Technical design documents for adding path planning capabilities to the self-balancing smart car.

## DOCUMENTS

| # | File | Topic | Pages |
|---|------|-------|-------|
| 01 | `01_position_pid_calibration.md` | Position loop PID design, code insertion points, calibration procedure, fault modes | ~8 |
| 02 | `02_optical_flow_fusion.md` | PMW3901 interface + wheel odometry complementary filter, EKF (optional), outlier handling, calibration | ~8 |
| 03 | `03_algorithm_selection.md` | Algorithm comparison (A\*, Pure Pursuit, DWA, Bug2, VFH+, MPC), recommended 3-layer architecture, memory budget | ~8 |

## READING ORDER

1. **01_position_pid_calibration.md** — Prerequisite: understand the control cascade before planning
2. **02_optical_flow_fusion.md** — Prerequisite: robust positioning for path following
3. **03_algorithm_selection.md** — Integrates both into a complete path planning architecture

## KEY DECISIONS (from these docs)

| Module | Recommendation | Reason |
|--------|---------------|--------|
| Position PID | P=0.5-2.0, I=0.01-0.1, out_max=±5° | Safety tilt clamp is the red line |
| Sensor fusion | Complementary filter (start) → EKF (advanced) | Complementary covers 90% of cases at 30 ops/cycle |
| Global planning | A\* (100×100 @ 5cm, 2bit/cell) | ~25KB RAM, runs async in super-loop |
| Local following | Pure Pursuit + S-curve velocity profile | <0.5ms/cycle, 4-wheel steering advantage |
| Obstacle avoidance | Bug2 (simple) → VFH+ (complex) | Depends on sensor count |
| NOT feasible | MPC — solver alone needs 300+ KB RAM | Exceeds 100KB total budget |

## NOTES

- All docs assume the existing control cascade (angular_speed → angle → speed PID in pit_call_back)
- Position PID output is a tilt angle offset, not a direct speed command
- PMW3901 is NOT currently initialized in main() — docs explain how to add it
- Path planning subsystem estimated at ~17KB RAM (fits in 100KB budget with 50-70KB remaining)
- All algorithms designed for float32-only, no external math libraries

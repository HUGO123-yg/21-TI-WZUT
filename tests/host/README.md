# Host vision regression

The host test exercises the shared camera front-end, six synthetic terrain
classes, path-error signs, cone-gap geometry, minefield white-line rejection,
mission navigation gates, route-to-subject lifecycle, per-minefield rotation
re-arming, two-turn yaw unwrapping, and IMU/vision freshness behavior.

Run it from the repository root:

```sh
clang -std=c99 -Wall -Wextra -Werror \
  -Itests/host/stubs -Iproject/code \
  project/code/Vision_frame.c \
  project/code/Vision_pipeline.c \
  project/code/Terrain_vision.c \
  project/code/Cone_vision.c \
  project/code/Minefield_vision.c \
  project/code/Mission_perception.c \
  project/code/Perception_fusion.c \
  tests/host/terrain_perception_test.c \
  -lm -o /tmp/terrain_perception_test
/tmp/terrain_perception_test
```

Passing this test proves deterministic host-side geometry only. It does not
replace an IAR rebuild or validation with recorded and live MT9V03X frames.

The control-module regression checks route/subject action restrictions, a
two-turn yaw unwrap across +/-180 degrees, and forced bridge/bumpy completion:

```sh
clang -std=c99 -Wall -Wextra -Werror \
  -Itests/host/stubs -Iproject/code \
  project/code/Pid.c \
  project/code/Route_plan.c \
  project/code/Rotation_ctrl.c \
  project/code/Bridge_ctrl.c \
  project/code/Bumpy_ctrl.c \
  tests/host/control_modules_test.c \
  -lm -o /tmp/control_modules_test
/tmp/control_modules_test
```

## Replay recorded frames

`vision_replay.c` runs the exact firmware observers over one or more 8-bit
binary PGM (`P5`) frames and prints tab-separated observations. Files are
processed in argument order so the terrain confidence state can stabilize over
a recorded approach sequence.

```sh
clang -std=c99 -Wall -Wextra -Werror \
  -Itests/host/stubs -Iproject/code \
  project/code/Vision_frame.c \
  project/code/Vision_pipeline.c \
  project/code/Terrain_vision.c \
  project/code/Cone_vision.c \
  project/code/Minefield_vision.c \
  tests/host/vision_replay.c \
  -o /tmp/vision_replay
/tmp/vision_replay recordings/*.pgm
```

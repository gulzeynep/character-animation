# N8RO Character Motion Final Project

This is a CLion/CMake C++ project for a simplified N8RO/Arkheon character animation final: a kinematic, CoM-aware controller that outputs 10 joint overrides through the `ICharacterController.h` SDK ABI. It does not compute forces, torques, rigid-body dynamics, or contact physics.

## What It Builds

- `n8ro_motion_core`: reusable C++ controller library
- `n8ro_motion`: shared library / DLL adapter with Arkheon SDK lifecycle exports
- `n8ro_motion_sim_plugin`: N8RO sim animation plugin for `AnimationModelNathanHuman`
- `n8ro_motion_demo`: console demo that prints sample joint angles
- `controller_safety_tests`: finite/range/smoothness checks

## Controller Joint Order

The reusable motion core outputs radians in this internal order:

1. `left_hip_pitch`
2. `right_hip_pitch`
3. `left_knee_pitch`
4. `right_knee_pitch`
5. `left_ankle_pitch`
6. `right_ankle_pitch`
7. `left_shoulder_pitch`
8. `right_shoulder_pitch`
9. `left_elbow_pitch`
10. `right_elbow_pitch`

The DLL adapter maps those angles to the SDK order:

1. `ARK_JOINT_UPPERARM_L`
2. `ARK_JOINT_UPPERARM_R`
3. `ARK_JOINT_LOWERARM_L`
4. `ARK_JOINT_LOWERARM_R`
5. `ARK_JOINT_THIGH_L`
6. `ARK_JOINT_THIGH_R`
7. `ARK_JOINT_CALF_L`
8. `ARK_JOINT_CALF_R`
9. `ARK_JOINT_FOOT_L`
10. `ARK_JOINT_FOOT_R`

## Build From Terminal

```powershell
cmake -S . -B build
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

## Use In CLion

Open this folder:

```text
C:\Users\acarz\Documents\Codex\2026-05-26\karakter-animasyonu-dersim-i-in-final
```

Then build `n8ro_motion` or `n8ro_motion_demo`. If CLion asks to reload CMake, accept it.

## N8RO Integration Status

The project now includes the SDK header at:

```text
include/arkheon/character/ICharacterController.h
```

The DLL exports all required lifecycle functions:

- `arkheon_character_sdk_version`
- `arkheon_character_plugin_name`
- `arkheon_character_get_motion_clips`
- `arkheon_character_create`
- `arkheon_character_destroy`
- `arkheon_character_tick`

The N8RO-specific boundary is isolated in `src/n8ro_adapter.cpp`.

The visible Scenario Editor animation flow uses sim plugins from:

```text
C:\N8RO\userPlugins\sim
```

For that flow, use:

```text
build\Release\character_plugin_220201014.dll
```

The plugin registers two custom motion states for `animationModelNathanHuman`:

- `Zeynep Walk`: phase-based 10-joint walking motion.
- `Zeynep Squat`: 10-joint squat / arm-balance motion.

The mission override in `n8ro_overrides/human_animation_loop.lua` switches between these two custom animation codes every 4 seconds.

See `docs/n8ro_integration.md` for the current exported DLL functions.

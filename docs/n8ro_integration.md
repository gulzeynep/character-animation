# N8RO Integration Notes

This project is ready as a C++ motion core plus a Windows shared library target using the Arkheon `ICharacterController.h` SDK ABI. The N8RO-specific boundary is isolated in `src/n8ro_adapter.cpp`.

The Scenario Editor animation flow also supports ASTSIM sim plugins. Those are not selected from a field in the UI; N8RO scans them from `C:\N8RO\userPlugins\sim` at startup.

## Exported Functions

The generated DLL exports the required SDK lifecycle functions:

```cpp
uint32_t arkheon_character_sdk_version(void);
const char* arkheon_character_plugin_name(void);
void arkheon_character_get_motion_clips(void* handle, int32_t out_clip_ids[3]);
void* arkheon_character_create(const float segment_lengths_m[10]);
void arkheon_character_destroy(void* handle);
int32_t arkheon_character_tick(
    void* handle,
    const arkheon_frame* frame,
    const arkheon_bone_state in_bones[66],
    arkheon_bone_override out_overrides[10],
    arkheon_vec3* out_root_translation_delta,
    arkheon_quat* out_root_rotation_delta,
    const arkheon_input_state* input,
    const arkheon_mission_goal* current_goal,
    const arkheon_env_api* env);
```

## Runtime Behavior

- `tick` writes all 10 major joint overrides every frame.
- Joint angles are converted to local pitch quaternions in glTF order.
- Foot contact is inferred through `env->raycast` when available.
- `ARK_GOAL_GOTO` produces root translation toward the target and reports completion.
- Hotkey A selects walk, B selects idle, and C selects faster gait.

## Joint Mapping

- `left_shoulder_pitch` -> `ARK_JOINT_UPPERARM_L`
- `right_shoulder_pitch` -> `ARK_JOINT_UPPERARM_R`
- `left_elbow_pitch` -> `ARK_JOINT_LOWERARM_L`
- `right_elbow_pitch` -> `ARK_JOINT_LOWERARM_R`
- `left_hip_pitch` -> `ARK_JOINT_THIGH_L`
- `right_hip_pitch` -> `ARK_JOINT_THIGH_R`
- `left_knee_pitch` -> `ARK_JOINT_CALF_L`
- `right_knee_pitch` -> `ARK_JOINT_CALF_R`
- `left_ankle_pitch` -> `ARK_JOINT_FOOT_L`
- `right_ankle_pitch` -> `ARK_JOINT_FOOT_R`

## If Bones Bend Around the Wrong Axis

The adapter currently encodes pitch as local X-axis rotation. If the GLB skeleton uses a different local pitch axis, update `quatFromPitch` in `src/n8ro_adapter.cpp` only.

## Scenario Editor Plugin Flow

Use this DLL when working with the `Animation` tab shown in Scenario Editor:

```text
build\Release\character_plugin_220201014.dll
```

Copy it to:

```text
C:\N8RO\userPlugins\sim
```

This plugin exports the ASTSIM plugin functions:

```cpp
create_plugin();
destroy_plugin(...);
get_plugin_signature(); // "ARKHEON_PLUGIN_V1"
```

At runtime it registers an animation model extension for:

- model type: `animationModelNathanHuman`
- animation code: `Zeynep Walk`
- animation code: `Zeynep Squat`

If the custom codes do not appear in the Scenario Editor, add `Zeynep Walk` and `Zeynep Squat` to the platform's Animation Component list, then assign one of them to the human entity.

The provided mission override switches between both custom states:

```text
n8ro_overrides\human_animation_loop.lua
```

## CLion

Open this folder in CLion:

```text
C:\Users\acarz\Documents\Codex\2026-05-26\karakter-animasyonu-dersim-i-in-final
```

Build target:

```text
n8ro_motion_sim_plugin
```

The Windows DLL name is:

```text
character_plugin_220201014.dll
```

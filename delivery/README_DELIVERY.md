# N8RO Character Animation Final Delivery

## Primary DLL

Submit this DLL as the main N8RO integration artifact:

```text
build\Release\character_plugin_220201014.dll
```

This is the N8RO sim-plugin DLL. It exports the plugin entry points expected by the N8RO sample plugin architecture:

```text
create_plugin
destroy_plugin
get_plugin_signature
```

The plugin registers a kinematic animation model for:

```text
animationModelNathanHuman
```

It registers these custom animation codes:

```text
Zeynep Walk
Zeynep Squat
```

These codes should be added to the platform's Animation Component if they do not already appear in the Scenario Editor.

## What The Model Does

The project implements a closed-library C++ character animation model that computes 10 joint-angle values every simulation tick:

```text
left hip pitch
right hip pitch
left knee pitch
right knee pitch
left ankle pitch
right ankle pitch
left shoulder pitch
right shoulder pitch
left elbow pitch
right elbow pitch
```

The model is kinematic only. It does not compute forces, torques, rigid-body dynamics, or contact physics.

The model implements two motion states:

```text
Zeynep Walk  -> phase-based walking motion
Zeynep Squat -> squat / arm-balance motion
```

The walking motion is generated from a phase-based controller:

```text
time + gait phase + CoM/support correction
    -> 10 joint angles
    -> N8RO NathanHuman joint overrides
```

## N8RO Local Integration Path

During local testing the DLL was placed here:

```text
C:\N8RO\userPlugins\sim\character_plugin_220201014.dll
```

N8RO reads this plugin folder at application startup. There is no visible upload field in the Scenario Editor for this DLL.

## Secondary ABI DLL

The project also builds this secondary DLL:

```text
build\Release\n8ro_character_motion.dll
```

That DLL implements the `ICharacterController.h` ABI that was shared as the canonical character-controller header:

```text
arkheon_character_sdk_version
arkheon_character_plugin_name
arkheon_character_get_motion_clips
arkheon_character_create
arkheon_character_destroy
arkheon_character_tick
```

The primary file to use with the visible N8RO sample-plugin flow is still:

```text
character_plugin_220201014.dll
```

## Build And Test

Build:

```powershell
cmake -S . -B build
cmake --build build --config Release
```

Test:

```powershell
ctest --test-dir build -C Release --output-on-failure
```

Latest verification:

```text
controller_safety_tests: passed
arkheon_abi_tests: passed
```

## Important Note About DLL Contents

Do not open or submit `*.recipe`, `*.tlog`, `*.vcxproj`, or XML build-output files as the DLL.

The XML file that lists paths such as:

```text
build\Release\character_plugin_220201014.dll
```

is only a Visual Studio/CMake build recipe/output manifest. It is not the real DLL.

The real DLL is the binary file under:

```text
build\Release\character_plugin_220201014.dll
```

Opening a real DLL in a text editor will not show readable C++ source code because it is compiled binary code.

## Suggested Submission Package

Recommended package contents:

```text
character_plugin_220201014.dll
n8ro_character_motion.dll
source\
  CMakeLists.txt
  include\
  src\
  tests\
  docs\
README_DELIVERY.md
N8RO_BUG_REPORTS.md
```

If only one DLL is accepted, submit:

```text
character_plugin_220201014.dll
```

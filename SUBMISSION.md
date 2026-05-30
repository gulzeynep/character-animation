# Character Animation Final Submission

Student: Zeynep Gul Acar  
Student number: 220201014  
Project: N8RO character animation plugin

## Implemented Motion States

The plugin registers two custom animation states for `animationModelNathanHuman`:

- `Zeynep Walk`: a phase-based walking motion.
- `Zeynep Squat`: a squat / arm-balance motion.

For compatibility with the current N8RO scenario UI, the plugin also registers the built-in animation codes as aliases. If the scenario stays on `Idle Neutral`, the plugin still drives the same 10-joint walking model instead of leaving the character static.

The mission override in `n8ro_overrides/human_animation_loop.lua` switches between these two states every 4 seconds.

## Controlled Joints

The model controls these 10 NathanHuman joints:

1. `leftHip`
2. `rightHip`
3. `leftKnee`
4. `rightKnee`
5. `leftAnkle`
6. `rightAnkle`
7. `leftShoulder`
8. `rightShoulder`
9. `leftElbow`
10. `rightElbow`

The model is kinematic only. It does not compute forces, torques, rigid-body dynamics, or contact physics.

## Build

```powershell
cmake -S . -B build
cmake --build build --config Release --target n8ro_motion_sim_plugin
```

The main output is:

```text
build\Release\character_plugin_220201014.dll
```

A prebuilt copy is included under:

```text
release\character_plugin_220201014.dll
```

## N8RO Deployment

Close N8RO before compiling or replacing the DLL.

Copy the plugin DLL to:

```text
C:\N8RO\userPlugins\sim\character_plugin_220201014.dll
```

Copy the mission override to:

```text
C:\N8RO\data\resources\missions\human_animation_loop.lua
```

If the animation codes do not appear in Scenario Editor, add `Zeynep Walk` and `Zeynep Squat` to the platform's Animation Component list, then assign one of them to the human entity.

## Runtime Verification

The deployed DLL writes a short verification log at:

```text
C:\N8RO\userPlugins\sim\character_plugin_220201014_runtime.log
```

During the final N8RO run, the plugin was loaded and both custom states were evaluated by the simulation host:

```text
registered animationCode="Zeynep Walk"
registered animationCode="Zeynep Squat"
evaluate activeAnimationCode="Zeynep Walk" t=0.05 overrides=10
evaluate activeAnimationCode="Zeynep Squat" t=5.05 overrides=10
```

This confirms that the custom plugin is active and returns 10 joint overrides for both implemented motion states.

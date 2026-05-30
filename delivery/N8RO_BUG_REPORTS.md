# N8RO Bug Reports

These are concise reports prepared from observed issues during the GenericCivillianPresence / GLB viewer workflow.

## 1. Minimized Windows Restore To Incorrect Screen Position

Severity: Medium

Steps to reproduce:

1. Open N8RO.
2. Open Scenario Editor / GLB World / related floating panels.
3. Minimize one or more panels.
4. Restore them from the taskbar or UI.

Actual result:

The restored tabs/windows appear at an unrelated position near the lower-right area of the desktop instead of their previous location.

Expected result:

Minimized panels should restore to their previous screen position and size.

## 2. GLB Viewer Captures Mouse And Prevents Normal Cursor/Scroll Use

Severity: High

Steps to reproduce:

1. Load `GenericCivillianPresence`.
2. Run the simulation.
3. Press `G` and open the GLB viewer.
4. Move the cursor inside the GLB viewer and try to scroll or interact with the left-side panel.

Actual result:

The GLB viewer captures the cursor/camera controls in a way that makes normal mouse movement or scrolling difficult or impossible.

Expected result:

The GLB viewer should allow reliable release of mouse capture and normal scrolling/interacting with the control panel.

## 3. Human Simulation Opens With Broken Pose / Unexpected Object State

Severity: High

Steps to reproduce:

1. Load `GenericCivillianPresence`.
2. Run the simulation.
3. Open the GLB viewer and select the human entity.

Actual result:

The human model can appear in a broken pose. In one observed run, the displayed state later changed unexpectedly and showed a ball/object-like state instead of a stable human motion preview.

Expected result:

The human model should load into a stable, valid animation pose and remain consistent across refreshes.

## 4. Joint Angle Editing Requires Toggling GLB Fallback Repeatedly

Severity: Medium

Steps to reproduce:

1. Open GLB World.
2. Scroll to Joint editor.
3. Change X/Y/Z joint angle values.
4. Press Apply.

Actual result:

Angle changes do not consistently appear unless the GLB fallback option is toggled off/on repeatedly.

Expected result:

Joint angle edits should be applied immediately after pressing Apply, without requiring fallback toggling.

## 5. Download Button On Side Toolbar Crashes The App

Severity: Critical

Steps to reproduce:

1. Open N8RO.
2. Click the side toolbar download button.

Actual result:

The application crashes and closes.

Expected result:

The download button should either complete the action or show a recoverable error message without closing the application.

## 6. Console Window Appears Before The Main App Opens

Severity: Low

Steps to reproduce:

1. Launch N8RO normally from Windows.

Actual result:

A command prompt / console window appears before the main application opens.

Expected result:

The production app should launch directly without showing an implementation console window unless debug mode is enabled.

## 7. New Release Requires Full Installer Instead Of In-App Update

Severity: Low / UX

Steps to reproduce:

1. Use an older N8RO release.
2. Attempt to update to the latest release.

Actual result:

The app requires downloading and running a full installer again instead of offering an app update flow.

Expected result:

The app should provide a clear update mechanism or explain why a full reinstall is required.

## 8. Close Buttons Do Not Always Work

Severity: Medium

Steps to reproduce:

1. Open N8RO floating panels/windows.
2. Press the `X` close button on different panels.

Actual result:

Some close buttons do not respond reliably.

Expected result:

Every visible close button should close its corresponding panel/window or show why it cannot be closed.

## 9. Random Button Clicks Can Freeze Or Close The Application

Severity: Critical

Steps to reproduce:

1. Open N8RO and load a scenario.
2. Click UI buttons during scenario editing / GLB viewing.
3. Repeat with buttons that do not immediately respond.

Actual result:

When a button does not respond, the app can freeze and sometimes closes unexpectedly.

Expected result:

Unresponsive actions should fail safely with no freeze or crash.

## 10. No Clear UI For Loading A Character Animation DLL

Severity: High / Documentation

Steps to reproduce:

1. Build a sim-plugin DLL following the sample plugin structure.
2. Open Scenario Editor.
3. Open the Animation tab for the human entity.
4. Look for a DLL/plugin upload or selection field.

Actual result:

No clear plugin/DLL loading field is visible. The available UI shows animation profile fields and animation-code fields, but not the closed-library integration path.

Expected result:

The application or documentation should clearly state where student-built DLLs must be placed, how they are selected, and how to verify that the model is currently active.

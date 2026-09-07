# AI Disclosure

This mod was made with AI, I am not a coder, I've only done very simple things in the past. Using AI, specifically claude in this case has allowed me to act on ideas I have but can't realistically make on my own. I don't think AI mods should be private, closed-source or paid, so along with a forum release(soon) I am releasing this on github so others can contribute or build off of this, it's open source with a GNU General Public License v3.0

# nodeCam

A BeamNG.drive camera mode that welds the camera to nodes you pick by hand, so it follows specific body parts — the logs in the back of a truck, a door, a bumper, an axle. More dynamic than the stock relative camera, because the view shakes, flexes and crumples with whatever it is attached to.

Point at nodes with the picker, toggle them in, and once four or more are attached the camera is driven by a rigid fit to those nodes. Nothing is picked automatically: the set is yours and it stays put until you change it.

## Features

- **Hand-picked node attachment.** Aim at a node, toggle it in. The camera fits a rigid frame to the set every frame.
- **Survives crashes.** Nodes torn off or dragged far out of place are dropped from the fit automatically, so the view stays usable through a wreck instead of flying off.
- **Survives resets.** A vehicle reset rebuilds the internal node map but keeps your nodes, anchor and view direction.
- **Detects vehicle changes.** Swapping or replacing a vehicle clears the set instead of pointing old node indices at a new body.
- **Steady mode.** Turn node following off for a rigid camera on the vehicle axes. The picker and every control keep working, so you can set a camera up from steady mode.
- **Saved cameras.** Up to six slots per vehicle, each holding its own anchor, node set and view direction.
- **Settings menu.** Pause → Mods → nodeCam. Around two dozen settings with live status and hover hints.
- **On-screen app.** Optional control panel with the five main actions, a status readout, and FOV and camera speed sliders. Collapses to a single button.
- **Softness.** Optional virtual springs for a looser, floatier mount, leashed so it can never drift far from the rigid answer.
- **Free anchor movement.** Move the camera anywhere with the normal camera movement keys, with an adjustable leash you can switch off entirely.

## Install

Drop `nodeCam.zip` into:

```
/BeamNG.drive/<version>/mods/
```

Then press **C** in game until you reach nodeCam.

To install from source instead, copy the `lua/`, `scripts/` and `ui/` folders into `mods/unpacked/nodeCam/`.

## Getting started

1. Press **C** until you reach nodeCam. You start in steady view.
2. Turn on the node picker. Nearby nodes appear as spheres; the one under your crosshair turns white.
3. Aim at a node and toggle it in. It turns green. Repeat until you have at least four.
4. Pick nodes **spread out in depth**, not all on one flat panel. A flat or collinear set has no third axis to solve for, and the camera will say so and stay steady.
5. Turn the picker off and drive.

Move the camera with the normal camera movement keys, hold the fast modifier to move quicker, and look around with the mouse or the right stick.

## Keybinds

Bind these under **Options → Controls → Camera**. None are bound by default. You can skip them entirely and use the on-screen app instead.

| Action | What it does |
| --- | --- |
| nodeCam: toggle node picker | Show nearby nodes and the crosshair, so you can pick what the camera welds to |
| nodeCam: toggle node under crosshair | Add or remove the node you are aiming at. Needs the picker on |
| nodeCam: clear nodes (steady view) | Drop every attached node and go back to a steady view on the vehicle axes |
| nodeCam: next camera slot | Cycle the saved cameras for this vehicle |
| nodeCam: toggle node following | Switch between following the picked nodes and a steady view |

## Picker colours

| Colour | Meaning |
| --- | --- |
| White | Under your crosshair, ready to toggle |
| Green | Attached and contributing to the fit |
| Orange | Attached but currently dropped as an outlier, usually torn off or badly deformed |
| Red | Not attached |

## Settings

Everything lives in **Pause → Mods → nodeCam**, and every setting persists to `settings/nodeCam.json`.

| Setting | Notes |
| --- | --- |
| Field of view | nodeCam sets FOV itself every frame, so the game's zoom keys do not apply in this camera |
| Camera move speed | How fast the movement keys push the anchor |
| Saved cameras per vehicle | 1 to 6 |
| Mouse / pad look inversion | Separate toggles: the mouse and the pad arrive on different input fields and need their own signs |
| Pick radius, crosshair tolerance, max drawn nodes | Tune the picker. It reads every node per frame while on, so leave it off while driving |
| Leash camera to vehicle, bounds margin | How far past the body the camera may travel, or off for unlimited |
| Softness, stiffness, damping, max sag | The optional spring mount |
| Outlier floor, median scale, max dropped | How aggressively torn-off nodes are discarded during a crash |
| Quiet logging | Silences nodeCam's info lines. Warnings and errors always get through |

## Console commands

Settings and actions are all on `nodeCamCore`:

```lua
nodeCamCore.diag()                 -- dump what the mod can see, paste this into bug reports
nodeCamCore.status()               -- one-line summary
nodeCamCore.set('fov', 75)         -- any setting by name, clamped and saved
nodeCamCore.get('moveSpeed')
nodeCamCore.toggle('picker')       -- any boolean setting
nodeCamCore.resetSettings()        -- back to defaults

nodeCamCore.togglePicker()
nodeCamCore.toggleNode()           -- toggle whatever is under the crosshair
nodeCamCore.clearNodes()
nodeCamCore.cycleSlot()
nodeCamCore.toggleEnabled()        -- node following on/off

nodeCamCore.nudge('forward', 0.1)  -- move the anchor, in metres
nodeCamCore.look(15, -5)           -- turn the view, in degrees
nodeCamCore.resetLook()
```

`set()` accepts any key in the settings table, so anything in the menu can be driven from the console. The console allows a wider range than the sliders, for example `moveSpeed` up to 20.

## Reporting bugs

Open an issue and include the output of `nodeCamCore.diag()`, the vehicle you were driving, and what you expected to happen. `diag()` reports whether the node map was built, where the vehicle orientation came from, which input path your look controls are actually using, how many nodes are attached, and whether the anchor is hitting its leash — which covers most of what is needed to reproduce a problem.

## License

GNU General Public License v3.0. See [LICENSE](LICENSE).

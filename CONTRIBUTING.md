# Contributing

Help is welcome and encouraged. If you feel like a feature can be swapped for something better or added, go right on ahead.

## Getting set up

1. Fork and clone the repo.
2. Symlink or copy `lua/` and `scripts/` into your USERFOLDER which can be found from the launcher or open mod files in the mod menu in beam. Should look like this and your unzipped copy can go here `BeamNG.drive/<version>/mods/unpacked/nodeCam/`.
3. Launch the game and it should just work. Reload Lua with `Ctrl+L` after an edit, no game restart needed most of the time, sometimes it might need a reboot..
4. `nodeCamCore.diag()` in the console tells you what the mod can currently see.

## Layout

| Path | What's in it |
| --- | --- |
| `lua/ge/extensions/core/cameraModes/nodeCam.lua` | The camera mode: node picking, the fit, movement, look, debug drawing |
| `lua/ge/extensions/nodeCamCore.lua` | Settings, saved anchors and camera slots, console commands, diagnostics |
| `lua/ge/extensions/core/input/actions/nodeCam.json` | Keybind definitions |
| `scripts/nodeCam/modScript.lua` | Loads `nodeCamCore` when a level starts |

## The solver

The maths is fenced off between the `NODECAM_SOLVER_BEGIN` and `NODECAM_SOLVER_END` markers in `nodeCam.lua`. Everything in that block is pure Lua with no game dependencies — no `vec3`, no `veh:`, no globals — so you can paste it into a standalone script and test it with plain `lua`. If you're touching the fit, the outlier rejection or the spring step, that's the easiest place to work and the easiest place to prove a change is right.

The block covers the polar decomposition, the weighted Kabsch fit, strain and outlier rejection, strike accounting, ray picking, and the soft-mode spring integrator.

Please keep it dependency-free. Anything that needs the game belongs outside the markers.

## Style

- Two-space indent, matching what's already there.
- The update loop runs every frame — reuse the scratch tables on `self` rather than allocating new ones.
- Wrap anything that touches the vehicle object or game globals in `pcall`. Builds differ in what they expose.
- Don't swallow errors silently. If a `pcall` fails in a way that would make a feature quietly do nothing, log it once. Mouse look died this way before.

## Pull requests

Keep changes focused, and say in the description which vehicles and game version you tested on. Crash behaviour is the whole point of this mod, so if you touched the fit, please mention what happens when you actually hit something.

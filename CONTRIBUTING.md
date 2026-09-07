# Contributing

Help is welcome, on anything from a one-line fix to a new feature.

## Getting set up

1. Fork and clone the repo.
2. Symlink or copy `lua/`, `scripts/` and `ui/` into `mods/unpacked/nodeCam/`.
3. Launch with the console open. Reload Lua with `Ctrl+L` after an edit — no game restart needed.
4. `nodeCamCore.diag()` in the console tells you what the mod can currently see.

For UI work, the Vue debug overlay has a Mods panel that reloads a single mod, and file changes are picked up automatically.

## Layout

| Path | What's in it |
| --- | --- |
| `lua/ge/extensions/core/cameraModes/nodeCam.lua` | The camera mode: the solver, node picking, movement, look, debug drawing |
| `lua/ge/extensions/nodeCamCore.lua` | Settings, persistence, saved cameras, pending actions, console commands, diagnostics |
| `lua/ge/extensions/core/input/actions/nodeCam.json` | Keybind definitions |
| `scripts/nodeCam/modScript.lua` | Loads `nodeCamCore` when a level starts |
| `ui/ui-vue/mods/nodeCam/` | Pause menu settings card |
| `ui/modules/apps/nodeCam/` | On-screen control app |

## How it fits together

The camera mode only runs inside `core_camera`'s update. Anything triggered from a keybind, the console or the UI is queued in `nodeCamCore` and consumed on the next frame. If you add an action, queue it there and consume it in `update()` **before** any early return, or it will silently never fire — that exact mistake made several keybinds dead in steady view for a while.

Settings all go through `nodeCamCore.set()`, which clamps, saves and is the single source of truth. The UI holds no rules of its own. Add a key to `M.defaults`, give it a range in `LIMITS` if it is numeric, and it is immediately reachable from the console; add a row to the settings card to expose it in the menu.

## The solver

The maths is fenced between the `NODECAM_SOLVER_BEGIN` and `NODECAM_SOLVER_END` markers in `nodeCam.lua`. Everything in that block is pure Lua with no game dependencies — no `vec3`, no `veh:`, no globals — so you can extract it and test it with plain `lua`. If you are touching the fit, the outlier rejection or the spring step, that is the easiest place to work and the easiest place to prove a change is right.

The block covers the polar decomposition, the weighted Kabsch fit, outlier rejection, ray picking, look rotation and the spring integrator.

Please keep it dependency-free. Anything that needs the game belongs outside the markers.

## Things worth knowing

- **Live versus rest positions.** The rest shape is only for the rigid fit. Anything spatial in the present tense — picking, drawing, distance tests — must use live node positions. Filtering on rest positions is what made open doors select the wrong nodes.
- **Vehicle IDs get reused** when a vehicle is replaced, so identity is checked with a signature (node count plus jbeam name), not the ID.
- **A reset is not a vehicle change.** The rest shape is rebuilt but the picked nodes stay valid.
- **Input arrives on two paths** with different sign conventions: mouse deltas and analog pairs. They are signed independently. `diag()` prints which path is carrying input.

## Style

- Two-space indent, matching what is already there.
- The update loop runs every frame — reuse the scratch tables on `self` rather than allocating new ones.
- Wrap anything that touches the vehicle object or game globals in `pcall`. Builds differ in what they expose.
- Don't swallow errors silently. If a `pcall` fails in a way that would make a feature quietly do nothing, log it once. Mouse look died this way before.

## Pull requests

Keep changes focused, and say in the description which vehicles and game version you tested on. Crash behaviour is the whole point of this mod, so if you touched the fit, please mention what happens when you actually hit something.

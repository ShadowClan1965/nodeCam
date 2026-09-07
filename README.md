# AI Disclosure

This mod was made with AI, I am not a coder, I've only done very simple things in the past. Using AI, specifically claude in this case has allowed me to act on ideas I have but can't realistically make on my own. I don't agree AI mods should be private, closed-source or paid, or paid, so along with a forum release(soon) I am releasing this on github so others can contribute or build off of this, it's open source with a GNU General Public License v3.0

# nodeCam

A BeamNG.drive camera mode that attaches a camera to the nearest nodes, so the camera follows specific body parts like the logs in the back of a truck, more dynamic than normal relative camera.

The camera picks a set of nodes around an anchor point, creates a rigid connection to the camera so its locked in. You can also manually select nodes to follow, lock it so when you move the camera, the used nodes dont change.

## Install

Drop `nodeCam.zip` into:

```
/BeamNG.drive/<version>/mods/
```

Then press **C** in game until you reach nodeCam.

To install from source instead, copy the `lua/` and `scripts/` folders into the same `mods/unpackeds/nodeCam/` directory.

## Keybinds

Bind these under **Options → Controls → Camera**. None are bound by default.

| Action | What it does |
| --- | --- |
| nodeCam: next anchor preset | Cycles dash, hood, bumper, roof, tail, wheel arches |
| nodeCam: toggle beam debug | Draws nearby nodes and the beams to the attached set |
| nodeCam: recentre view | Snaps the look direction back to straight ahead |
| nodeCam: lock node set | Freezes the attached nodes so moving the camera keeps them |
| nodeCam: toggle node under crosshair | Adds or removes the node you're aiming at (needs debug on) |
| nodeCam: ignore nodes (steady view) | Holds a steady view off the vehicle axes instead |
| nodeCam: clear all nodes | Empties the set and locks it, so you can pick your own |
| nodeCam: next camera slot | Cycles the three saved camera and node sets per vehicle |

Standard camera movement keys move the anchor. Hold the fast modifier to move quicker.

## Console commands

All settings live on `nodeCamCore` and can be changed live from the console:

```lua
nodeCamCore.diag()              -- dump what the mod can see, paste this into bug reports
nodeCamCore.preset('hood')      -- dash, hood, bumper, roof, tail, wheelLeft, wheelRight
nodeCamCore.nudge('forward', 0.1)  -- move the anchor, in metres
nodeCamCore.look(15, -5)        -- turn the view, in degrees
nodeCamCore.setNodes(14)        -- how many nodes to attach to (4-32)
nodeCamCore.setSoftness(0.3)    -- 0 is rigid, above 0 blends in virtual springs
nodeCamCore.setFov(70)
nodeCamCore.forceFov(true)      -- stop the zoom filter overriding fov
nodeCamCore.setMoveSpeed(1.5)
nodeCamCore.setLookSensitivity(0.3)
nodeCamCore.toggleInvertYaw()
nodeCamCore.toggleInvertPitch()
nodeCamCore.flipForward()       -- if front and back came out reversed
nodeCamCore.clearBans()         -- forget every node the mod struck off
nodeCamCore.toggleDebug()
nodeCamCore.status()
```

## Reporting bugs

Open an issue and include the output of `nodeCamCore.diag()`, the vehicle you were driving, and what you expected to happen. `diag()` reports whether the node map was built, where the vehicle orientation came from, which input fields are actually moving, and how many nodes are attached, which covers most of what's needed to reproduce a problem.

## License

See [LICENSE](LICENSE).

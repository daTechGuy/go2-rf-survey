# Field guide — running the Go2 by hand, step by step

Printable, do-this-then-this instructions for a session with the robot.
They assume nothing is set up and nothing is remembered from last time.

| Guide | When | Time |
|---|---|---|
| **[01 — Small-room test](01-small-room-test.md)** | **First.** Unpack to pack-up: power, connect, check every safety piece, ending with a 1 m autonomous walk and a floorplan of the room. | ~90 min |
| [02 — Classroom demo](02-classroom-demo.md) | Only after 01 has worked once. The 3 m (10 ft) route from the classroom door, mapping the room, with people watching. | ~45 min before the audience, ~5 min demo |

The engineering detail behind these steps is in
[../autonomous-loop.md](../autonomous-loop.md) and [../live-view.md](../live-view.md).

---

## Kit list

- [ ] Robot, battery **charged** (all LEDs lit)
- [ ] Handheld remote, **charged**
- [ ] Laptop, charged, on Wi-Fi with internet
- [ ] USB Ethernet adapter (the ASIX one, Windows calls it **"Ethernet 4"**) + Ethernet cable, 3 m or longer
- [ ] Painter's tape (to mark the start spot and the turn-round spot)
- [ ] Tape measure
- [ ] A cardboard box, about knee height (for the obstacle checks)

The extra Mid-360 LiDAR and the D435i depth camera can stay mounted and
plugged in. Nothing here uses them yet. The mapping uses the robot's own
built-in head LiDAR.

---

## The remote: controls card

| Buttons | What it does | |
|---|---|---|
| **L2 + START** | **Damping: the hard stop.** Legs go soft and the robot sinks down. Use it whenever anything looks wrong. | ✔ verified on this robot |
| **L2 + A** | Stand up, **joint-locked** (stiff, will NOT walk) | ✔ verified |
| **START** (one single press, after L2 + A) | **Walking mode**: the body gently sways, "breathing". The robot only obeys walk commands in this mode. A double press does **not** work. | ✔ verified |
| Sticks | Walk / turn by hand (in walking mode) | standard |
| **SELECT + UP**, held 1.5 s | Our software: **start** the route (or resume after a pause) | ⚠ check in guide 01, step G |
| **SELECT + DOWN** | Our software: **abort** the route | ⚠ check in guide 01, step G |
| Any stick pushed past halfway *during a route* | Our software: **abort** the route | ⚠ check in guide 01, step G |
| Lie down from standing | Unitree's manual: **L2 + A** again from the locked stand. **L2 + START** also works (it just sinks less gracefully). | check on your remote |

**Power button** (robot battery and remote): **press once briefly, then press
and hold about 2 seconds.** The same for on and for off.

## Safety rules (every session)

1. **Remote on and in your hand before the robot is powered.** L2 + START is
   the stop that always works, cable or no cable.
2. **Nobody within 1.5 m of the robot** while it can walk. Kids, chairs, bags
   and cables out of the route.
3. **Never let it walk with the Ethernet cable plugged in.** When it turns round,
   it wraps the cable around itself. The software is built so the cable is
   only needed while you set up.
4. **One step at a time.** Each step proves what the next relies on. If a
   step fails, stop there and note what it printed (see "Record sheet" at
   the end of guide 01). Don't skip ahead.
5. The software also refuses by itself: there's no live walk without a passed
   turn test today, and no route longer than 1 m without a completed 1 m route
   today.

## Where you type

Two different terminals. Always look at the prompt first:

| Prompt looks like | It is | Used for |
|---|---|---|
| `PS C:\Users\Professor>` | **Laptop**, PowerShell | `ssh go2`, `scp` (copy files off the robot), `ping` |
| `unitree@ubuntu:~/go2-rf-survey$` | **Robot**, over SSH | every `bash tools/...` command |

Get from the laptop to the robot with `ssh go2`, then `cd ~/go2-rf-survey`.
`exit` goes back to the laptop.

**Passwords:** the robot's `sudo` password is `123` (it's a demo unit; the
default was left on purpose). `ssh go2` needs no password because the
laptop's key is installed on the robot.

---

## When something goes wrong

| You see | It means | Do |
|---|---|---|
| `ping 192.168.123.18` times out | Cable, adapter or laptop IP | Guide 01, step D3 |
| `ssh: connect ... timed out` | Same as above | Guide 01, step D3 |
| preflight `WARN no internet` | The robot's internet route is gone (it resets at every boot) | `sudo ip route replace default via 192.168.123.222 dev eth0` |
| preflight `FAIL image is older than docker/` | Code changed since the last build | `bash tools/go2loop.sh build` (needs internet) |
| preflight `motion: NOT READY` ... `not standing` | Robot lying down | Remote: L2 + A, then a single START |
| `motion mode '...', not 'normal'` | Robot in a mode that may ignore walk commands | Stop. Note the mode name and ask before going on. |
| `REFUSED: ... Stage 1` | No turn test passed today | Guide 01, step J |
| `start refused` / `path blocked (front=..., near=...)` | Something within ~0.55 m of the robot | Move it, or move the robot to open floor |
| `PAUSED` during a route | Something stepped in front, or a sensor went quiet | Clear the way, hold SELECT + UP to resume |
| The robot does anything you didn't expect | — | **L2 + START.** Then note what happened. |

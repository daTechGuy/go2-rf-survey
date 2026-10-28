# 02 — Classroom demo: map the room from the door

**What happens:** the robot starts just inside the classroom door, walks
**3 m (10 ft)** down the aisle by itself, turns round, walks back, and leaves
a floorplan of the room. You start it and stop it from the handheld remote;
no cable is attached while it walks.

**Only after [01 — Small-room test](01-small-room-test.md) has worked at least
once** (steps G, J, K and L all passed).

**Time:** about 45 minutes of setup **before the audience arrives**, then
about 5 minutes for the demo itself.

**Why setup takes 45 minutes:** the software only allows a 3 m walk after a
passed turn test **and** a completed 1 m walk **within the last 12 hours**. So
you re-run both, quickly, before people arrive.

Controls card and safety rules: [README](README.md). Detailed versions of the
setup steps (what to do when one fails): guide 01, same letters.

---

## A. The day before

- [ ] Robot battery and remote charged.
- [ ] Laptop charged, Foxglove installed, `git pull` in `$HOME\go2-rf-survey`.
- [ ] Look at the classroom: the aisle from the door must have **more than
  1.1 m (3 ft 8 in) of clear width** for its whole 3 m length, **and** room to
  turn round at the far end (about 1.2 m × 1.2 m clear). Chairs pushed in,
  bags off the floor. The robot stops if anything is within 0.55 m of its centre.
- [ ] Find a **separate open patch** (2.5 m × 2 m) for the warm-up 1 m walk:
  the front of the room, a hallway, the small room from guide 01.

## B. Unpack and set down (as guide 01 B)

1. [ ] Tape the start: **just inside the door, facing down the aisle**. Tape a
   second strip **3 m (10 ft)** ahead: the turn-round spot.
2. [ ] Robot out of its case, lying on its belly, legs folded, on the warm-up
   patch (not the aisle yet).
3. [ ] Check: LiDAR dome clear, nothing loose, no cables near the legs.

## C. Power up (as guide 01 C)

1. [ ] **Remote on:** press briefly, then hold ~2 s.
2. [ ] **Robot on:** battery button, press briefly, then hold ~2 s. Wait ~1 minute,
   stay clear (it may stand up on its own).

## D. Connect (as guide 01 D)

1. [ ] Cable: laptop adapter ↔ robot. Laptop Wi-Fi on.
2. [ ] Laptop PowerShell: `ping 192.168.123.18` → replies.
3. [ ] `ssh go2`, then `cd ~/go2-rf-survey`.

## E. Internet + check (as guide 01 E)

1. [ ] On the robot: `sudo ip route replace default via 192.168.123.222 dev eth0` (password `123`)
2. [ ] `bash tools/go2loop.sh preflight`. If it says `FAIL image is older`:
   `bash tools/go2loop.sh build`, then preflight again.

## F. Live view (as guide 01 H)

1. [ ] Robot: `bash tools/go2loop.sh viz-up`
2. [ ] Laptop: Foxglove → Open connection → `ws://192.168.123.18:8765`, layout "Go2 map".

## G. Warm-up 1: the turn test (as guide 01 I–J)

On the warm-up patch, 1.5 m clear around.

1. [ ] Remote: **L2 + A**, then **one single START**. It sways ("breathing").
2. [ ] Robot:
   ```bash
   bash tools/go2loop.sh preflight          # last line: motion: READY
   bash tools/go2loop.sh mode               # name='mcf' (or 'normal')
   bash tools/go2loop.sh bridge-up
   bash tools/go2loop.sh turn-test --i-confirm-breathing
   bash tools/go2loop.sh bridge-down
   ```
   The turn test must print **PASS**. If it keeps turning: **L2 + START**.
   **FAIL → no demo walk today.** (Plan B, at the end of this guide.)

## H. Warm-up 2: the 1 m walk (as guide 01 L)

Same patch, robot facing 2 m of clear floor.

1. [ ] Still breathing (else L2 + A, single START).
2. [ ] Robot:
   ```bash
   bash tools/demo.sh --length 1.0 --i-confirm-breathing --name warmup --remote-start --detach
   ```
   Wait for **`>> SAFE TO UNPLUG.`**
3. [ ] Unplug the cable at the robot, gently from the side. Stand 1.5 m to the side, thumbs off the sticks.
4. [ ] Hold **SELECT + UP** 2 seconds. It walks 1 m out, turns round, walks back, stops.
5. [ ] Wait a minute. Lie it down, plug in, `ssh go2`, `cd ~/go2-rf-survey`,
   `bash tools/go2loop.sh results` → **`demo finished: done`**.
   The 3 m walk is now unlocked for 12 hours.

## I. Move to the doorway and check the aisle (nothing moves)

1. [ ] Unplug. Walk the robot to the start tape **with the remote sticks** (or
   lie it down and carry it). It should stand **at the start tape, facing the
   turn-round tape**. Plug the cable back in and `ssh go2` again if it dropped.
2. [ ] Check the clearance with the robot standing still. It starts the
   route's software in **dry-run** mode, which can't move anything:
   ```bash
   bash tools/go2loop.sh up -p shape:=out_and_back -p length:=3.0
   bash tools/go2loop.sh status
   ```
   The line ends with `front N, near N`. **Both must be 0.**
   - `near` above 0 → something (door frame, desk, chair) is within 0.55 m.
     Move it, or move the start a little further in.
   - `front` above 0 → something is in the path ahead.
3. [ ] Optional but recommended: walk it on the sticks to the **turn-round tape**,
   run `bash tools/go2loop.sh status` again (want `near 0` there too), and
   walk it back to the start.
4. [ ] `bash tools/go2loop.sh down`
5. [ ] In Foxglove, the room's map has been building during all of this. To
   start the audience on an empty screen: `bash tools/go2loop.sh viz-reset`.

Setup done. The robot can wait, lying down, until the audience is ready.

---

## J. The demo

While the cable is in, the laptop shows the robot's live LiDAR view of the
room: a good thing to put on the projector while you explain.

1. [ ] Audience **behind the robot or to the sides, 1.5 m back**, not in the aisle.
2. [ ] Remote: **L2 + A**, then **one single START** → breathing.
3. [ ] Robot:
   ```bash
   bash tools/demo.sh --length 3.0 --i-confirm-breathing --name classroom --remote-start --detach
   ```
   Wait for **`>> SAFE TO UNPLUG.`**
4. [ ] Unplug the cable at the robot. Stand to the side, thumbs off the sticks.
5. [ ] Hold **SELECT + UP** for 2 seconds.
6. [ ] It walks 3 m down the aisle, turns round at the far tape, and walks back
   to the door: about a minute.
   - Someone steps in front → it **stops and waits**. That's the obstacle stop, a
     good thing to point out. Clear the way, hold **SELECT + UP**: it carries on.
   - To end it early: **SELECT + DOWN**.
   - Anything unexpected: **L2 + START.**
7. [ ] When it's back and stopped, wait a minute (it builds the floorplan),
   lie it down, plug the cable back in.

## K. Show the map

1. [ ] Foxglove: reconnect (`ws://192.168.123.18:8765`). The whole room it
   scanned, and the path it walked, are there. Rotate it in 3D for the audience.
2. [ ] For the flat floorplan, robot: `bash tools/go2loop.sh results`. Then
   laptop (new PowerShell window):
   ```powershell
   cd $HOME\Desktop
   scp go2:surveys/classroom_<date_time>_map-topdown.png .
   ```
   (copy the exact name from `results`), and open it.

## L. Shut down and pack (as guide 01 O)

1. [ ] Robot:
   ```bash
   bash tools/go2loop.sh demo-stop
   bash tools/go2loop.sh viz-down
   bash tools/go2loop.sh bridge-down
   bash tools/go2loop.sh down
   exit
   ```
2. [ ] Robot lying down → battery: press briefly, then hold ~2 s → LEDs off.
3. [ ] Remote off the same way. Unplug the cable. Pack. Put the batteries on charge.

---

## Plan B: if the walk isn't allowed today

If the turn test fails, or the 1 m warm-up doesn't finish `done`, the
software won't allow the 3 m walk, and that's correct. You can still show:

- **The live scan:** robot lying or standing at the door, cable in, Foxglove on the projector.
  Walk around it: people appear as moving dots. (`viz-up`, nothing moves.)
- **A map you drive by hand:** the software only records, and you walk the
  robot down the aisle and back on the remote sticks, as normal remote driving.
  The software commands nothing.
  1. Robot at the door, cable in. Robot shell:
     ```bash
     bash tools/go2loop.sh record-start classroom-manual
     ```
  2. Unplug the cable at the robot. Remote: L2 + A, a single START, then
     drive slowly down the aisle, turn round, and come back.
  3. Lie it down, plug in, `ssh go2`, `cd ~/go2-rf-survey`:
     ```bash
     bash tools/go2loop.sh record-stop
     ```
     It closes the recording, builds the floorplan, and prints the `scp`
     line. Foxglove shows the map too (step K).

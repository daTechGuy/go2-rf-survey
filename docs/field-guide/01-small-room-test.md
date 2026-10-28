# 01 — Small-room test (do this first)

**What it proves:** every piece, in order: power, network, software, the
remote buttons, the safety stops, a turn on the spot, and finally a short
**1 m out-and-back walk with no cable attached**, which leaves a floorplan of
the room on the robot.

**Time:** about 90 minutes. **People:** you, plus a helper if possible.

**Room:** any small room with a clear patch of floor about **2.5 m long and
2 m wide** (8 ft × 6½ ft). Flat, hard floor; no rugs with loose edges.

Read [the controls card and safety rules](README.md) once before starting.
Tick each box as you go. **If a step fails, stop and fill in the record sheet
at the end.** Don't skip ahead.

---

## A. The day before

- [ ] Charge the robot battery fully.
- [ ] Charge the handheld remote.
- [ ] Laptop: the **Foxglove** desktop app is installed (<https://foxglove.dev/download>).
- [ ] Laptop: the code is current. In PowerShell: `cd $HOME\go2-rf-survey; git pull`

## B. Unpack and set the robot down

1. [ ] Clear the floor patch. Put a strip of tape on the floor where the robot's
   **front feet** will start, and a second strip **1 m (3 ft 3 in) straight ahead** of it.
   The robot walks out to the second strip, turns round and comes back.
2. [ ] Lift the robot out of its case: two hands under the body, not by the legs
   or the LiDAR.
3. [ ] Set it down **lying flat on its belly, legs folded underneath**, at the
   first tape strip, facing the second. This is the position it must be in
   when it is switched on.
4. [ ] If the battery was taken out for transport, slide it in until it clicks.
5. [ ] Look it over:
   - the built-in LiDAR (the dome on the head) is clean and nothing is covering it;
   - the Mid-360 on top and the D435i are fastened and their cables are seated
     (they aren't used today, but nothing should be loose);
   - no cables hang where a leg can catch them.

## C. Power up: remote first, then the robot

The remote goes on first so the hard stop is in your hand from the moment
the robot is live.

1. [ ] **Remote on:** press the power button once briefly, then press and hold about
   2 seconds. Its lights come on.
2. [ ] **Robot on:** same on the battery: press once briefly, then press and hold about 2 seconds.
   The battery LEDs light up and the robot starts its self-check: you'll hear
   the motors click and twitch.
3. [ ] **Wait about 1 minute** and keep hands and feet clear. On Unitree's default
   settings the robot may **stand up on its own** after the self-check. Either
   way is fine:
   - still lying down: good, leave it;
   - stood up: you can leave it standing, or lie it down (controls card).
4. [ ] Hold the remote from now on. **L2 + B** is your stop.

## D. Connect the laptop

1. [ ] Plug the USB Ethernet adapter into the laptop. Plug the cable into it, and
   into the robot's Ethernet port (the same port used in earlier sessions).
   Lay the cable **behind** the robot, out of the route.
2. [ ] Laptop Wi-Fi **stays on**. The robot borrows the laptop's internet through
   the cable.
3. [ ] In laptop **PowerShell**:
   ```powershell
   ping 192.168.123.18
   ```
   You want `Reply from 192.168.123.18`. **If it times out:**
   - Is the adapter showing as "Ethernet 4"? Check with
     `Get-NetIPAddress -InterfaceAlias "Ethernet 4" -AddressFamily IPv4`.
     You want `192.168.123.222`.
   - If the address is missing, open **PowerShell as Administrator** and run:
     ```powershell
     New-NetIPAddress -InterfaceAlias "Ethernet 4" -IPAddress 192.168.123.222 -PrefixLength 24
     ```
   - If Windows now names the adapter differently (e.g. after a different USB
     port or adapter), use that name in the command above, and in E2 below.
4. [ ] Log in to the robot:
   ```powershell
   ssh go2
   ```
   The prompt changes to `unitree@ubuntu:~$`. Then:
   ```bash
   cd ~/go2-rf-survey
   ```

## E. Give the robot internet, then run the start-of-session check

1. [ ] On the **robot**: this resets every time the robot boots, so it's needed every session:
   ```bash
   sudo ip route replace default via 192.168.123.222 dev eth0
   ```
   Password: `123`.
2. [ ] Only if the next step says `no internet`: check the laptop's sharing in
   laptop PowerShell: `Get-NetNat` should list **Go2Nat**. If it doesn't, open
   **PowerShell as Administrator** and run:
   ```powershell
   Set-NetIPInterface -InterfaceAlias "Ethernet 4" -Forwarding Enabled
   Set-NetIPInterface -InterfaceAlias "Wi-Fi" -Forwarding Enabled
   New-NetNat -Name Go2Nat -InternalIPInterfaceAddressPrefix 192.168.123.0/24
   ```
3. [ ] On the **robot**: run the check. It updates the code from GitHub, then only reads.
   ```bash
   bash tools/go2loop.sh preflight
   ```
   Read each section:
   - `== network` → `OK internet via the laptop`
   - `== code` → `OK code folders at origin/main ...`
   - `== image` → `OK`, **or** `FAIL image is older`: then do step F.
   - `== robot` → the sensors are listed `OK`. The last line says
     `sensing: READY` and `motion: NOT READY` while the robot lies down.
     **That is expected for now.**

## F. Rebuild the software image (only if E3 said so)

1. [ ] On the **robot**:
   ```bash
   bash tools/go2loop.sh build
   ```
   This takes several minutes and needs the internet. It ends with
   `Successfully tagged go2-survey:humble`.
2. [ ] Run `bash tools/go2loop.sh preflight` again. `== image` must now be `OK`.

## G. Check the remote buttons (read-only, robot lying down)

The new start and abort combos have never been tried on this remote. This
step shows exactly what the robot receives. **Nothing in this step can make it walk.**

1. [ ] Robot **lying down**. If it is standing, lie it down first (controls card).
2. [ ] On the **robot**:
   ```bash
   bash tools/go2loop.sh remote-keys
   ```
   It prints `probe: READ-ONLY ...`, and then a line every time a button or
   stick changes, plus a rate line every 5 seconds.
3. [ ] Press and hold each of these, one at a time, and check the line it prints:

   | Press | Should print | Tick |
   |---|---|---|
   | SELECT + UP (D-pad up) | `keys=0x1008 SELECT+UP` | [ ] |
   | SELECT + DOWN | `keys=0x4008 DOWN+SELECT` | [ ] |
   | L2 + B (robot lying down: it just stays down) | `keys=0x0220 L2+B` | [ ] |
   | Push the left stick forward | `sticks lx,ly,rx,ry=(0.0, 0.x, ...)` | [ ] |
   | (rate line) | `(NNN msg/s)`, a steady number, not 0 | [ ] |

4. [ ] **Watch the robot while pressing SELECT + UP and SELECT + DOWN.** It
   must do **nothing**. If it reacts (lights, sounds, moves), the robot's own
   firmware uses that combo: write it down.
5. [ ] `Ctrl-C` to quit.

**If any row is wrong:** write down exactly what it printed. Do steps H–J,
then **skip K–M** (they need these buttons) and finish with N–O. The combos
can be changed in the software once we know what your remote sends.

## H. Live view on the laptop

1. [ ] On the **robot**:
   ```bash
   bash tools/go2loop.sh viz-up
   ```
   It ends with `Foxglove app -> Open connection -> ... ws://192.168.123.18:8765`.
2. [ ] On the **laptop**: open **Foxglove** → **Open connection** → **Foxglove WebSocket**
   → `ws://192.168.123.18:8765` → **Open**.
3. [ ] First time only: add a **3D** panel and set its **Display frame** to
   `odom`, then turn on `/go2/map_cloud`, `/go2/scan_lite`, `/go2/path` and `/go2/pose`.
   Add an **Image** panel on `/go2/camera/compressed`: the robot's front camera.
   Save the layout (**Layouts → Save as** "Go2 map").
   Details: [../live-view.md](../live-view.md).
4. [ ] You see the room as dots, building up. Walk around the robot: you
   appear in the scan.
5. [ ] The Image panel shows the view from the robot's front camera. If it
   stays blank, run `bash tools/go2loop.sh camera-check` and write down what
   it prints. Nothing else depends on the camera, so carry on.

## I. Stand it up and check it's ready to walk

1. [ ] Floor around the robot clear for 1.5 m. Everyone back.
2. [ ] Remote: **L2 + A**. The robot stands up stiff.
3. [ ] Remote: **one single press of START**. The body starts to sway gently
   ("breathing"). **Look for the sway; it's the only sign of walking mode.**
4. [ ] On the **robot**:
   ```bash
   bash tools/go2loop.sh preflight
   ```
   The last line must now say `motion: READY (after you confirm breathing)`.
5. [ ] Which mode owns the legs (read-only):
   ```bash
   bash tools/go2loop.sh mode
   ```
   You want `name='mcf'` (this robot's firmware) or `name='normal'` (older
   firmware's name for the same mode). **Anything else: stop here** and note it.

## J. Stage 1: the turn test (the robot turns about 20° on the spot)

The first time the software moves the robot. It turns slowly in place for
2 seconds, then must stop by itself. The cable can stay plugged in; a 20°
turn won't wrap it.

1. [ ] Still breathing (the sway from I3). If it stopped, L2 + A then a single START again.
2. [ ] Remote in hand, thumb near **L2 + B**. Everyone 1.5 m back.
3. [ ] On the **robot**:
   ```bash
   bash tools/go2loop.sh bridge-up
   ```
   About 1 s after it starts, the robot may shift its posture slightly
   (a "balance stand" command). That is expected, and it doesn't walk.
4. [ ] Then:
   ```bash
   bash tools/go2loop.sh turn-test --i-confirm-breathing
   ```
   **Watch:** it turns slowly to the left about 20°, then stops and stays stopped.
   **If it keeps turning or swings after stopping: L2 + B.**
5. [ ] Read the report it prints. It ends **PASS** or **FAIL** with the reasons.
6. [ ] Always, pass or fail:
   ```bash
   bash tools/go2loop.sh bridge-down
   ```

**PASS:** go on to K.

**FAIL:** copy the whole report into the record sheet, then try J2 once. If
J2 isn't possible or also fails, go to N–O. (The software won't allow any
walking today without a PASS anyway.)

## J2. Only if J failed: the turn test with "walk prep"

Another Go2 project found that, after the remote has been used, the robot
needs two extra commands before it follows software turns properly. Without
them it "jerks", which matches what ours did. The catch: those commands
switch the remote's **sticks** off while the software drives. So first prove,
with nothing moving, that your hard stop still works.

1. [ ] Robot standing and breathing (I2–I3). Everyone 1.5 m back.
2. [ ] Switch the sticks off. It's a setting, and nothing moves:
   ```bash
   bash tools/go2loop.sh joystick hold
   ```
3. [ ] Push a stick gently. The robot should **not** walk. Then, in a
   second SSH window (`ssh go2`, `cd ~/go2-rf-survey`), run
   `bash tools/go2loop.sh remote-keys` and press SELECT + UP: it must still
   print `SELECT+UP` (the remote start depends on it). Ctrl-C.
4. [ ] **Press L2 + B.** It must sink down (damp) as usual. **Write down
   yes/no.**
5. [ ] Always, whatever happened:
   ```bash
   bash tools/go2loop.sh joystick release
   ```
   Stand it up again (L2 + A, single START) and check that the sticks walk it.
6. [ ] **Only if step 4 was yes:** repeat J3–J6 with the new option:
   ```bash
   bash tools/go2loop.sh bridge-up -p backend:=sport_prep
   bash tools/go2loop.sh turn-test --i-confirm-breathing
   bash tools/go2loop.sh bridge-down
   ```
   **PASS:** in steps L and M, add `--backend sport_prep` to every `demo.sh`
   command. While those walks run, the sticks are off: the stop is
   **L2 + B** or **SELECT + DOWN**.

## K. Rehearse the untethered run: nothing moves

This practises the exact sequence of the real walk, with the robot unable to
move: start it from the laptop, unplug, start it from the remote, plug back
in, check the result.

1. [ ] Lie the robot down (controls card). It stays down for all of K.
2. [ ] On the **robot**:
   ```bash
   bash tools/demo.sh --length 1.0 --rehearse --remote-start --detach
   ```
   It runs the checks, then prints the log so far, ending with
   **`>> SAFE TO UNPLUG.`** (allow up to a minute).
3. [ ] **Unplug the Ethernet cable at the robot.** The laptop's SSH window
   freezes or drops. That's expected.
4. [ ] Hold **SELECT + UP** for 2 full seconds. Nothing moves; it's a rehearsal.
5. [ ] Wait 30 seconds. Plug the cable back in. On the **laptop**:
   ```powershell
   ssh go2
   ```
   then on the **robot**:
   ```bash
   cd ~/go2-rf-survey
   bash tools/go2loop.sh results
   ```
   The top run must say **`demo finished: rehearsal OK`**.
6. [ ] Look at the log to see that the remote press was received:
   ```bash
   bash tools/go2loop.sh demo-log
   ```
   Look for `remote start (loop idle) -> start` and `success=True`.
   `Ctrl-C` to stop viewing.

   *Not there?* If it shows `no start from the remote` or a `WARNING: no
   /wirelesscontroller messages`, the press didn't arrive. Note it and skip L–M.
7. [ ] **Abort rehearsal:** repeat 2–5, but in step 4, about 5 seconds after
   SELECT + UP, press **SELECT + DOWN**. `results` must now say
   **`demo finished: aborted`**.
8. [ ] **Stick-abort rehearsal:** repeat 2–5, but in step 4, after SELECT + UP,
   push the left stick fully forward within 10 seconds. Lying down, it won't walk. `results`
   must say **`demo finished: aborted`**.

## L. Stage 2: the 1 m walk, no cable

The real thing: the robot walks 1 m out, turns round on the spot, walks back
and stops. **It walks on its own; your job is to watch and be ready.**

1. [ ] Robot lying at the first tape strip, facing the second. 1.5 m clear on
   both sides, 2 m ahead. Box and bags out of the way.
2. [ ] Remote: **L2 + A**, then **one single START**. Check that it's breathing (sway).
3. [ ] On the **robot**:
   ```bash
   bash tools/demo.sh --length 1.0 --i-confirm-breathing --name smallroom --remote-start --detach
   ```
   It refuses by itself if the turn test hasn't passed today, if the robot
   isn't standing, or if preflight finds a problem. Otherwise it sets
   everything up (the robot may shift its posture once when the bridge
   starts) and prints **`>> SAFE TO UNPLUG.`**
4. [ ] **Unplug the cable at the robot.** Approach from the side, gently; it's
   standing. Move the cable well out of the route.
5. [ ] Stand **1.5 m to the side** of the start, remote in hand, thumbs **off the sticks**.
6. [ ] Hold **SELECT + UP** for 2 seconds.
7. [ ] **Watch.** It walks slowly to the far strip (~0.3 m/s), stops, turns
   round on the spot in **one** direction, walks back, and stops near the
   start. 20–40 seconds in all. Then it stands breathing.
   - Anything unexpected → **L2 + B.**
   - It stops and waits mid-route → it saw something within half a metre.
     Clear it, then hold SELECT + UP to resume.
8. [ ] It keeps working for about a minute after stopping: it closes the
   recording and builds the floorplan. Then lie it down (controls card) and
   plug the cable back in.
9. [ ] On the **laptop** `ssh go2`, then on the **robot**:
   ```bash
   cd ~/go2-rf-survey
   bash tools/go2loop.sh results
   ```
   Want: **`demo finished: done`**. A completed 1 m route counts as
   **Stage 2 PASS**, which unlocks the 3 m route for the next 12 hours.

## M. Optional: the obstacle stop

Proves it stops for something in its path, and waits.

1. [ ] Repeat L1–L6 (use `--name smallroom2`).
2. [ ] While it walks out, a helper sets the **box** down in its path about
   1 m ahead of its nose, from the side, and steps back.
3. [ ] It must **stop** about half a metre short of the box, and stay stopped.
4. [ ] Take the box away. Hold **SELECT + UP**. It continues the route.
5. [ ] L8–L9 as before.

## N. Look at the map and copy it to the laptop

1. [ ] **Foxglove:** reconnect (`ws://192.168.123.18:8765`). The map it built
   during the walk is still there, plus the path it walked. The live map kept
   building on the robot while the cable was out.
2. [ ] On the **robot**, `bash tools/go2loop.sh results` prints `scp` lines for each run.
3. [ ] On the **laptop** (a new PowerShell window, not the SSH one):
   ```powershell
   cd $HOME\Desktop
   scp go2:surveys/smallroom_<date_time>_map-topdown.png .
   ```
   Copy the exact name from the `results` output. Open the PNG: it's the room
   seen from above.
4. [ ] To keep the whole run, including the raw recording: the `whole run:` line
   from `results` (it can be a few hundred MB).

## O. Shut down and pack

1. [ ] On the **robot**: stop everything (each is harmless if already stopped):
   ```bash
   bash tools/go2loop.sh demo-stop
   bash tools/go2loop.sh viz-down
   bash tools/go2loop.sh bridge-down
   bash tools/go2loop.sh down
   exit
   ```
2. [ ] Robot **lying down** (controls card).
3. [ ] **Robot off:** on the battery, press once briefly, then hold about 2 seconds. Wait
   for the LEDs to go out.
4. [ ] **Remote off:** same: press briefly, then hold.
5. [ ] Unplug the cable from the robot and from the adapter.
6. [ ] Lift the robot with two hands under the body into its case. Put the
   batteries on charge before the next session.
7. [ ] Fill in the record sheet below.

---

## Record sheet

Copy this into a message (or `docs/`) after the session. It's what's needed
to fix anything that went wrong.

```
Date:
Step reached (A–O):
E3 preflight last line:
G  remote-keys:   SELECT+UP -> ______   SELECT+DOWN -> ______
                  L2+B  -> ______   rate ____ msg/s
                  robot reacted to SELECT+UP/DOWN? yes / no
I5 mode:          name=______
J  turn-test:     PASS / FAIL   (paste the report)
J2 sticks held:   stick walked it? yes / no   L2+B damped? yes / no
   turn-test sport_prep: PASS / FAIL / not run
K  rehearsal:     results line: ______   abort: ______   stick: ______
L  1 m walk:      results line: ______
                  what it did (turned which way, stopped where):
M  obstacle stop: stopped? ____  resumed? ____
Anything odd:
```

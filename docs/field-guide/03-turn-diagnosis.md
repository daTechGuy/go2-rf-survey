# 03 — Turn diagnosis (DONE 2026-10-28: no need to repeat)

> **Solved the same day.** The Go2 only *sways* in place when asked for a
> gentle turn — from its own remote too. Tries A and B below made no
> difference; `state-watch` showed mode/gait/foot fields are always 0 on this
> firmware. The turn test now uses a brisk 1.0 rad/s turn, which passes, and
> the first autonomous 1 m walk followed. Kept as a record of how it was found;
> details in [../autonomous-loop.md](../autonomous-loop.md), Stage 1.

**Why:** on 2026-10-28 the turn test (guide 01, step J) failed three times
the same way. The robot reacted to our turn command within 0.1 s and stopped
cleanly afterwards, but turned only 5–14° of the 25–38° asked. You saw a
rear foot shuffle and the body twist: **it never started walking.** Its own
status said `mode=0 gait=0` the whole time.

This session finds out why, in three short tries. **Only steps 3 and 4 move
the robot from software, and Claude asks you before each.**

**Time:** about 30 minutes. **Stop:** **L2 + B** (hold L2, press B): it goes
limp onto its belly. L2 + START is **not** a stop.

---

## 1. Set up (guide 01, steps B–E)

- [ ] Power on (remote first), plug in the cable, `ssh go2`, `cd ~/go2-rf-survey`.
- [ ] `sudo ip route replace default via 192.168.123.222 dev eth0` (password `123`).
- [ ] Tell Claude. Claude runs `preflight` (this pulls the new code).

## 2. Watch what "walking" looks like: you drive, the software only listens

The software has never seen this firmware walk. Here you walk it with the
sticks while a recorder notes what the robot reports, foot by foot.
**The software sends nothing in this step.**

1. [ ] Floor clear 1.5 m around. Remote: **L2 + A**, then **one single START**
   (breathing sway).
2. [ ] In the SSH window:
   ```bash
   bash tools/go2loop.sh state-watch
   ```
   Wait for `state watch: READ-ONLY`. **Keep the robot still for 3 seconds.**
3. [ ] Left stick **halfway forward for 2 seconds**, then let go.
4. [ ] Wait 3 seconds. Right stick **halfway left for 2 seconds** (turn on the spot), then let go.
5. [ ] Wait 3 seconds, then **Ctrl-C**. Paste everything it printed to Claude.

What it shows: a `mode=… gait=…` line each time the robot changes state, and
every second the speed, the stick and `STEPPED` / `NO STEPS` for the feet.

## 3. Try A: without our "balance stand" command (Claude asks first)

Our bridge sends a *balance stand* command when it starts. On this firmware
that may be switching the robot **out** of walking mode, the breathing you set
with START.

- [ ] Robot standing and breathing. Everyone back, thumb near **L2 + B**.
- [ ] Claude runs:
  `bridge-up -p auto_balance_stand:=false`, then the turn test, then `bridge-down`.
- [ ] Watch: **do the feet step?** The report now ends with a `feet …` line
  that says `STEPPED` or `NO STEPS`.

## 4. Try B: "walk prep" (guide 01, step J2) (Claude asks first)

Sends the robot's **FreeWalk** ("start walking") command first. It switches
the sticks off while the software drives, so you check the stop first:

- [ ] Claude switches the sticks off (`joystick hold`, a setting; nothing moves).
- [ ] Push a stick gently: it should **not** walk.
- [ ] Press **L2 + B**: it must go limp and lie down. **Yes / no?**
- [ ] Claude switches the sticks back on (`joystick release`). Stand it up again.
- [ ] Only if L2 + B worked: Claude runs the turn test with `backend:=sport_prep`.

## 5. Then

Whichever try **PASSES with `STEPPED`** becomes the setting for guide 01
steps K–M (the rehearsals and the 1 m walk). If neither does, the
`state-watch` log from step 2 shows what the firmware does differently when
*you* walk it, and that is the next thing to copy.

## Record sheet

```
Date:
2  state-watch: walking forward -> mode=__ gait=__  STEPPED? __
               turning         -> mode=__ gait=__  yaw=____
3  try A (no balance stand): turned __ of __ deg   feet: STEPPED / NO STEPS   PASS / FAIL
4  try B (sport_prep):  stick walked it while held? __   L2+B damped? __
                        turned __ of __ deg   feet: STEPPED / NO STEPS   PASS / FAIL
Anything odd:
```

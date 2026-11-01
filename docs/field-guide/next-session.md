# Next session — one page

About **45 minutes**. Copy-paste each command; send Claude what the
**Send** line says. Claude asks before anything moves the robot.

**Stop at any time: L2 + B** (hold L2, press B). L2 + A, SELECT + DOWN or a
stick also end a run.

---

## 1. Power and connect (5 min)

1. Remote **on** first, then the robot (battery button: press once, then hold
   2 s). Leave it lying down.
2. Cable into the robot's Ethernet port. In a **normal** PowerShell (not
   "Run as administrator"):
   ```powershell
   ssh go2
   ```
   Prompt shows `unitree@ubuntu:~$`. Then:
   ```bash
   sudo ip route replace default via 192.168.123.222 dev eth0
   cd ~/go2-rf-survey
   bash tools/go2loop.sh preflight
   ```
   Password `123`.

**Send:** the last 5 lines of preflight.

## 2. Mid-360 check — nothing moves (5 min)

```bash
bash tools/go2loop.sh mid360-check
```
It prints a summary at the end, and one `sudo tcpdump ...` line. Run that
line too (password `123`).

**Send:** the whole output of both.

> If it says **`WORKS with our config`**, the Mid-360 problem is solved, and
> it can be put to work the same day (prepared, switched off until you say so):
>
> 1. Stand the robot up (L2 + A, single START) on open floor, a few metres
>    from walls, and keep it **still**. Then (nothing moves):
>    ```bash
>    bash tools/go2loop.sh mid360-calibrate
>    ```
>    It measures the Mid-360's tilt, roll, height and direction from the floor
>    and walls. **Send:** its output.
> 2. In step 4 below, Claude then adds `--mid360` to the explore command: the
>    Mid-360 also feeds the obstacle stop, the turn check and the map.

## 3. Today's safety gate — the robot moves (10 min)

Clear 1.5 m all round. Remote: **L2 + A**, then **one single START** (body
sways = walking mode).

1. **Turn test.** Tell Claude **"go"**. It steps round briskly ~20°, then
   stops. Must say **PASS**.
2. **1 m walk.** Robot facing 2.5 m of clear floor. Tell Claude **"ready"**.
   At **`SAFE TO UNPLUG`**: unplug, stand aside, hold **SELECT + UP** 2 s.
   It walks 1 m, turns round, comes back. Plug back in.

**Send:** "turn test PASS / FAIL", "1 m done / what happened".

## 4. Explore the room — the robot moves (10 min)

Start spot **1.5 m inside the room** (not in the doorway), clear all round.
L2 + A, single START. Tell Claude **"ready"**. At `SAFE TO UNPLUG`: unplug,
stand at the **edge** of the room, hold **SELECT + UP**.

It walks to places it can see unmapped floor from, then comes home by a direct
route. **Watch the podium and the desks** — it should keep well clear now.

**Send:** what it did, and **anything it touched or came close to**.

## 5. Copy everything to the laptop, then pack up (10 min)

On the **laptop**, a new PowerShell window:
```powershell
cd $HOME\go2-rf-survey
python tools\get_results.py
```
Cut off half-way? Plug back in and run it again — it carries on.

Then: L2 + B (lies down), robot off, remote off, unplug, batteries on charge.

**Send:** the last line of `get_results` (how many files copied).

---

Each run also leaves a one-page report:
`C:\Users\<you>\go2-surveys\<run>_report.html` — double-click to open.

# 04 — Explore a room on its own

**What happens:** no route, no tape. The robot looks around, walks up to
1.5 m into open floor it hasn't seen, and repeats, up to 8 legs within 4 m of
where it started. Then it walks home along its own path and leaves a
floorplan. For rooms whose furniture moves between visits.

**Only after** guide 01 has passed its 1 m walk once. Each session still
starts with a turn test and a 1 m walk (the safety gate needs both within 12
hours).

**Time:** 10 minutes of setup, then 2–4 minutes of exploring.

**Stop:** **L2 + B** (it goes limp). **L2 + A**, **SELECT + DOWN** or a stick
also end the run. If the robot stands still while the software expects it to
move, the run pauses itself within 5 seconds.

---

## A. Before (as guide 01 B–I)

1. [ ] Robot charged, remote on first, robot on. Cable in, `ssh go2`.
2. [ ] `sudo ip route replace default via 192.168.123.222 dev eth0` (password `123`).
3. [ ] Tell Claude. Claude runs `preflight`.
4. [ ] Start spot: open floor, **1.5 m clear all round**. Bags, cables and
   anything knee-high or lower that it can't see well (thin chair legs are
   fine; flat cables on the floor are invisible to it) out of its way.

## B. Today's safety gate (as guide 01 J and L)

1. [ ] Turn test: L2 + A, a single START, then **"go"** to Claude. Must PASS.
2. [ ] 1 m walk: Claude starts it, you unplug and hold SELECT + UP. Must end `done`.

## C. Explore

1. [ ] Robot standing and breathing at the start spot, cable in.
2. [ ] Tell Claude **"ready"**. Claude runs:
   ```bash
   bash tools/demo.sh --explore --i-confirm-breathing --name room --remote-start --detach
   ```
   and waits for **`>> SAFE TO UNPLUG.`**
3. [ ] Unplug the cable, move it well away. Stand at the side of the room, not
   in the middle: it will walk around you, but it explores open floor.
4. [ ] Hold **SELECT + UP** 2 seconds.
5. [ ] **Watch.** Short walks, a brisk turn between each. If someone stands in
   its way it waits a second, then picks another way. On the way home it
   waits for you if blocked: clear it, hold SELECT + UP.
6. [ ] When it stands still back near the start (or after 5 minutes, the time
   limit): lie it down, plug in, tell Claude. Claude shows the floorplan.

## D. Copy the results to the laptop (before unplugging for the day)

In a **normal** laptop PowerShell window:
```powershell
cd $HOME\go2-rf-survey
python tools\get_results.py
```
Floorplans and logs first, the big recordings last, into
`C:\Users\<you>\go2-surveys`. Cut off half-way? Plug back in and run it again:
it carries on. `--no-recordings` for just the maps (seconds, not minutes).

Options (Claude adds them): `--legs N` (default 8), `--radius R` metres from
the start (default 4).

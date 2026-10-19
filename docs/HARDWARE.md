# Hardware

What to buy, what to avoid, and why. Each item below removes a specific
documented risk.

| Item | Removes | Needed by | Approx |
|---|---|---|---|
| [USB WiFi adapter — MT7612U](#wifi-scanning-adapter) | R2, R8 | part 6 | $25–40 |
| [USB-Ethernet adapter](#usb-ethernet-adapter) | — | part 1 | $15 |
| [LiDAR mount, ~22° pitch](#lidar-mount) | R3 | part 4 | print |
| [Second Go2 battery](#second-battery) | R6 | part 7 | $200+ |
| [Short USB 2.0 extension](#usb-20-extension) | R8 | part 6 | $8 |
| [Antenna mast](#optional-antenna-mast) *(optional)* | R7 | — | $20 |

---

## WiFi scanning adapter

**The single most important purchase decision in this project**, and the one
where the obvious choice is wrong.

### What actually matters

Not throughput. **The adapter never transfers data — it only ever scans.** A
$25 adapter is exactly as good as a $90 one here. What matters is:

1. **Dual-band** — it must see 5 GHz, or half the survey is missing
2. **A driver on the Jetson's kernel** — JetPack 5.1.1 is **kernel 5.10**
3. **Stability under sustained scanning** — 90 minutes of repeated sweeps
4. **A fixed antenna** — see [antenna orientation](#antenna-orientation)

Point 2 is the one that catches people. **Docker shares the host kernel**, so
running Humble in a container does not rescue a driver that needs a newer one.

### Driver support by chipset

| Chipset | Driver | Mainline since | On kernel 5.10 |
|---|---|---|---|
| **MT7612U** | `mt76x2u` | 4.19 | ✅ **plug and play** |
| MT7610U | `mt76x0u` | 4.19 | ✅ but 1×1, weaker |
| MT7921AU | `mt7921u` | 5.18 | ❌ too new |
| RTL8812BU / 8822BU | `rtw88` | 6.12 | ❌ far too new |
| RTL8812AU / 8814AU | out-of-tree only | **never** | ❌ DKMS |
| AR9271 | `ath9k_htc` | ancient | ⚠️ 2.4 GHz only — useless here |

This inverts the usual advice. MT7921AU is the newer, better adapter and would
normally win — on a 5.10 kernel it is useless without backporting.

### Recommended

| Model | Chipset | USB ID | Notes |
|---|---|---|---|
| **Netgear A6210** | MT7612U | `0e8d:7612` | AC1200. Compact, widely available used. **First choice.** |
| **Alfa AWUS036ACM** | MT7612U | `0e8d:7612` | AC1200, external RP-SMA. Better range; zip-tie the antennas. |

Any MT7612U adapter works. **Verify the USB ID before buying** — vendors
silently change chipsets between hardware revisions under the same model name
(the TP-Link Archer T4U is the classic trap: v1, v2 and v3 are three different
chipsets). If a listing does not state the chipset, assume it is Realtek.

```bash
lsusb        # want 0e8d:7612
             # 0e8d: = MediaTek, good
             # 0bda: = Realtek, probably trouble
```

### Avoid

Anything Realtek RTL88xx, unless you enjoy DKMS against a vendor kernel.
Specifically avoid adapters marketed as **AC1750 / AC1900** — those classes
are almost exclusively RTL8814AU, which has never been mainlined.

### Tested on hand: Edimax EW-7833UAC — not recommended

Tested 2026-10-19 on Windows to read its USB ID.

```
Description      : Edimax AC1750 Wi-Fi USB Adapter
USB ID           : 7392:a833
Chipset          : RTL8814AU
Radio status     : Hardware On / Software On   (hardware is healthy)
```

The hardware is fine. The chipset is the problem:

- never merged to mainline
- Edimax's own driver covers kernel 2.6–**4.4** and will not compile past 4.10
- the only route on 5.10 is out-of-tree DKMS
  ([`morrownr/8814au`](https://github.com/morrownr/8814au), covers 5.4–6.18)
  against a Jetson **vendor** kernel — needs L4T headers and rebuilds on every
  kernel bump
- RTL8814AU has a poor record under exactly this project's load: sustained
  repeated scanning
- it is **USB 3.0**, which buys nothing (scanning only) and worsens **R8** by
  putting a 2.4 GHz noise source beside its own antenna
- its antenna is **180° adjustable** — the R5 problem below

If you already own one, it is worth a time-boxed attempt:

```bash
sudo apt install -y dkms git linux-headers-$(uname -r)
git clone https://github.com/morrownr/8814au && cd 8814au
sudo ./install-driver.sh
iw dev && sudo iw dev wlan1 scan | grep -c '^BSS'
```

Stop after an hour if it is not scanning. If it does work, **soak-test it** —
scan in a loop for 30 minutes and confirm the interface does not wedge —
before trusting it to a survey.

### Antenna orientation

Prefer a **fixed, non-articulating** antenna. **R5** already has RSSI varying
5–10 dB with robot heading; an antenna that flops as the robot walks adds a
second uncontrolled variable to the measurement.

If your adapter has hinged antennas, zip-tie them in one position and leave
them there for the entire survey campaign. Changing antenna geometry between
runs makes those runs incomparable.

### If you need 6 GHz

Surveying WiFi 6E means MT7921AU or MT7922, which need kernel ≥ 5.18 — i.e.
upgrading the robot to JetPack 6. That is a larger decision than an adapter
purchase; see [D1](plan.md).

---

## USB-Ethernet adapter

For your laptop, if it has no wired port. Needed in part 1 and for every
LiDAR debugging session afterwards.

No special requirements — any gigabit USB 3.0 adapter. The robot's internal
network is 100 Mbit anyway.

---

## LiDAR mount

A bracket holding the Mid-360 **~22° nose-down**. Printed or shimmed; the
angle matters, the material does not.

Mounted level, the Mid-360 cannot see the floor within 3.3 m — its vertical
FOV is −7° to +52°, biased upward. **This fails silently:** the map looks
excellent and the robot walks into things.

```bash
python tools/fov_geometry.py --height 0.42   # your measured height
```

| Pitch | Blind radius | Upper FOV kept |
|---|---|---|
| 0° (level) | 3.26 m | 52° |
| 15° | 0.99 m | 37° |
| **22°** | **0.72 m** | **30°** |
| 30° | 0.53 m | 22° |

Requirements:

- **rigid** — a mount that flexes while walking injects noise straight into
  the SLAM extrinsic
- **repeatable** — if you ever remove it, it must go back at the same angle,
  or the URDF is wrong and every map after that is subtly off
- keep the LiDAR's 360° horizontal view clear of the robot's own structure

The Mid-360 needs **9–27 V**, not USB. Power it from the Go2's expansion port.

---

## Second battery

1–1.5 hours of walking, less with LiDAR, depth camera and full compute
running. Any sizeable floor needs multiple runs.

Pair this with resumable route segments (**R6**) so a battery swap resumes
rather than restarts.

---

## USB 2.0 extension

A short USB 2.0 extension cable, to move the WiFi adapter away from the
RealSense and its cable.

USB 3.0 raises the 2.4 GHz noise floor by ~20 dB
([Intel whitepaper](https://www.usb.org/sites/default/files/327216.pdf)). The
D435i is USB 3.0 and runs throughout a survey. Without separation, every
2.4 GHz reading is depressed while 5 GHz looks fine, and the heatmap shows a
dead zone that is actually the robot (**R8**).

Deliberately **USB 2.0**, not 3.0 — the adapter only scans, so there is no
throughput to lose, and a USB 2.0 link is not itself a broadband noise source.

Characterise it once, in part 6 of the [runbook](RUNBOOK.md#characterise-usb-30-interference--once).

---

## Optional: antenna mast

Only if you need **absolute** dBm at human height.

Sampling happens at ~0.40 m; phones and laptops live at 1–1.5 m. Relative
coverage and dead zones transfer accurately between those heights; absolute
values do not (**R7**).

A light mast carrying **only the scanning antenna** to ~1 m fixes it. It does
not need to be near the LiDAR, and should not be — keep it clear of the
sensor's horizontal view.

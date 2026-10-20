# Manual motion test — "step forward" over SSH

**First verified: 2026-10-20** (demo robot, LiDAR/depth not yet mounted).
Robot walked ~0.9 m forward on command and stopped cleanly.

> ⚠️ **This bypasses the safety bridge.** It talks straight to the vendor
> `SportClient` — **no watchdog, no velocity clamp, no slew limit, no e-stop**
> (those live in `go2_survey_bridge`, phase 6). Use this only as a **manual,
> attended bring-up test**: a human on the remote, the robot standing, ≥3 m
> clear ahead, and a self-terminating command. For anything autonomous, use the
> bridge, not this.

## What we learned about this robot (phase-0 facts)

- **No Python SDK** (`unitree_sdk2py` not installed) and **no internet** on the
  robot (direct wired link only) — so we can't `pip install`. Use the C++ SDK
  that's already on disk.
- **Vendor C++ SDK:** `/unitree/lib/unitree_go2_sdk/unitree_go2_sdk/`
  - headers: `/usr/local/include/unitree` (+ `ddscxx`, `iceoryx/v2.0.2`)
  - libs in `/usr/local/lib`: `unitree_go2_sdk unitree_go2_idl_cpp
    unitree_ros2_idl_cpp ddsc ddscxx rt pthread`
  - `g++` is present.
- **API** (from `example/go2/sport_client_test.cpp`):
  `unitree::robot::ChannelFactory::Instance()->Init(0, "<iface>")`, then
  `unitree::robot::SportClient sc; sc.SetTimeout(10); sc.Init();`, then
  `sc.Move(vx, vy, vyaw)` (returns `0` on accept).
- **Interface is `eth0`** on this robot. The stock example hardcodes
  `enx0826ae3e0542`, which does **not** exist here (it's the SDK author's USB
  dongle). A wrong interface is fail-safe: `Move` just returns non-zero and
  **nothing moves**.
- **Do NOT run the stock `sport_client_test` binary** — it is an infinite
  `while(true)` loop sending `Move(0.5, 0.0, 0.8)` (forward *and* turning)
  with no stop. It will drive the robot in circles until killed.

## The program — `~/step_forward.cpp`

Forward at 0.3 m/s for 3 s (~0.9 m), then a guaranteed zero-velocity stop even
on Ctrl-C. Interface is `argv[1]`, default `eth0`.

```cpp
#include <unitree/robot/client/client.hpp>
#include <unitree/robot/go2/sport/sport_client.hpp>
#include <iostream>
#include <chrono>
#include <thread>
#include <csignal>

using namespace std::chrono;
static volatile std::sig_atomic_t g_stop = 0;
static void on_signal(int) { g_stop = 1; }

int main(int argc, char** argv)
{
  const char* iface = (argc > 1) ? argv[1] : "eth0";
  const double VX = 0.3;       // m/s forward (gentle)
  const double SECONDS = 3.0;  // ~0.9 m  ≈ 3 steps

  std::signal(SIGINT,  on_signal);
  std::signal(SIGTERM, on_signal);

  unitree::robot::ChannelFactory::Instance()->Init(0, iface);
  unitree::robot::SportClient sc;
  sc.SetTimeout(10.0f);
  sc.Init();

  std::cout << "[step_forward] iface=" << iface << "  vx=" << VX
            << " m/s for " << SECONDS << " s. Robot must be STANDING.\n"
            << "Forward begins in 2 s. Ctrl-C here, or remote L2+START, to stop.\n";
  std::this_thread::sleep_for(seconds(2));

  auto t0 = steady_clock::now();
  int n = 0;
  while (!g_stop &&
         duration_cast<milliseconds>(steady_clock::now() - t0).count() < (long)(SECONDS * 1000)) {
    int32_t ret = sc.Move(VX, 0.0, 0.0);
    if (ret != 0 || n % 5 == 0) std::cout << "  Move ret=" << ret << "\n";
    ++n;
    std::this_thread::sleep_for(milliseconds(100));
  }

  for (int i = 0; i < 10; ++i) {        // guaranteed stop
    sc.Move(0.0, 0.0, 0.0);
    std::this_thread::sleep_for(milliseconds(50));
  }
  std::cout << "[step_forward] stopped.\n";
  return 0;
}
```

## Build

```bash
cd ~
g++ -std=c++17 -O2 step_forward.cpp -o step_forward \
  -I/usr/local/include/ddscxx \
  -I/usr/local/include/iceoryx/v2.0.2 \
  -I/usr/local/include/unitree \
  -L/usr/local/lib \
  -lunitree_go2_sdk -lunitree_go2_idl_cpp -lunitree_ros2_idl_cpp \
  -lddsc -lddscxx -lrt -lpthread
```

## Run

1. ≥3 m clear ahead; stay clear of leg range.
2. **Remote:** L2 + A to stand, then double-click START to enter walking mode.
   Leave the sticks centered.
3. `./step_forward eth0`
4. It waits 2 s, creeps forward ~0.9 m over 3 s, stops, and stays standing.

**Stops:** Ctrl-C in SSH (sends zero-velocity on exit) → or **L2 + START
(damping)** on the remote, which overrides everything instantly.

**Tuning:** `VX` (speed, m/s) and `SECONDS` (duration) are the two constants at
the top. Distance ≈ `VX * SECONDS`. Keep both small for indoor tests.

## Connecting from the laptop (recap)

Direct wired link, laptop on the robot's subnet:

```powershell
# Windows, admin PowerShell — adapter was "Ethernet 3" (TP-LINK USB)
New-NetIPAddress -InterfaceAlias "Ethernet 3" -IPAddress 192.168.123.222 -PrefixLength 24
ssh unitree@192.168.123.18      # Jetson Orin
```

Keep Wi-Fi on (you need internet); the static IP adds an on-link route to
`192.168.123.0/24` out the wired adapter, so robot traffic goes there anyway.

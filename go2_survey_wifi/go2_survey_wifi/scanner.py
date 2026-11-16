"""WiFi scanning via nl80211, and parsing of ``iw`` output.

The parser is deliberately a pure function over text. It is the most bug-prone
part of this package -- ``iw`` output is loosely structured, varies between
versions, and has several fields that are *usually* present -- so it is written
to be testable against captured fixtures with no radio, no root and no ROS.
See ``test/test_scanner.py``.

Nothing in this module imports rclpy. It runs standalone:

    python -m go2_survey_wifi.scanner wlan1
"""

from __future__ import annotations

import re
import shutil
import subprocess
from dataclasses import dataclass, asdict
from typing import Iterable, Iterator


# ``iw scan`` needs CAP_NET_ADMIN. We shell out rather than bind nl80211
# directly: the dependency surface is one binary that is present on every
# Linux with a working wireless stack, and the output is stable enough to
# parse once it is pinned by tests.
IW_BINARY = "iw"

# A scan sweeps every channel in every supported band. 2.4 GHz alone is fast;
# dual-band with DFS channels is where the 2-5 s figure comes from. This is the
# reason the survey stops to scan rather than scanning while walking.
DEFAULT_SCAN_TIMEOUT_S = 15.0


@dataclass(frozen=True)
class Observation:
    """One access point, as seen in one scan sweep."""

    bssid: str
    ssid: str
    freq_mhz: int
    rssi_dbm: float

    @property
    def band(self) -> str:
        """Coarse band label. Useful for splitting heatmaps by band."""
        if 2400 <= self.freq_mhz < 2500:
            return "2.4"
        if 5000 <= self.freq_mhz < 5900:
            return "5"
        if 5925 <= self.freq_mhz <= 7125:
            return "6"
        return "?"

    def as_dict(self) -> dict:
        return asdict(self)


class ScanError(RuntimeError):
    """Raised when a scan could not be performed at all.

    Distinct from a scan that succeeded and simply saw nothing -- an empty
    result is valid data (it means no coverage, which is exactly what a survey
    is looking for) and must never be conflated with a failure.
    """


# ``BSS aa:bb:cc:dd:ee:ff(on wlan1)`` -- optionally followed by ``-- associated``
_RE_BSS = re.compile(r"^BSS\s+([0-9a-fA-F:]{17})\b")
_RE_CONNECTED = re.compile(r"^Connected to\s+([0-9a-fA-F:]{17})\b")
_RE_IFACE = re.compile(r"^\s*Interface\s+(\S+)\s*$")
_RE_FREQ = re.compile(r"^\s*freq:\s*(\d+)(?:\.\d+)?\s*$")
_RE_SIGNAL = re.compile(r"^\s*signal:\s*(-?\d+(?:\.\d+)?)\s*dBm")
# SSID may be empty (hidden network) or absent entirely.
_RE_SSID = re.compile(r"^\s*SSID:\s?(.*)$")
# Some iw versions report the frequency inside the DS Parameter set instead.
_RE_DS_CHAN = re.compile(r"^\s*DS Parameter set:\s*channel\s*(\d+)")


def parse_iw_dev(text: str) -> list[str]:
    """Extract wireless interface names from ``iw dev`` output."""
    out: list[str] = []
    for line in text.splitlines():
        m = _RE_IFACE.match(line)
        if m:
            name = m.group(1)
            if name not in out:
                out.append(name)
    return out


def choose_interface(interfaces: Iterable[str]) -> str | None:
    """Pick the best default survey interface from ``iw dev`` names.

    Prefers a dedicated USB adapter (``wlan1``, ``wlx*``) over the Jetson's
    onboard ``wlan0`` when both exist, ignoring ``p2p-dev-*`` pseudo-interfaces.
    """
    names = [n for n in interfaces if not n.startswith("p2p-dev-")]
    if not names:
        return None
    for n in names:
        if n != "wlan0" and (n.startswith("wlan") or n.startswith("wlx") or n.startswith("wl")):
            return n
    return names[0]


def resolve_interface(interface: str = "auto") -> str:
    """Resolve ``'auto'`` (or empty) to a real wireless interface via ``iw dev``."""
    if interface and interface != "auto":
        return interface
    if shutil.which(IW_BINARY) is None:
        raise ScanError(
            f"{IW_BINARY!r} not found on PATH. Install it with: "
            "sudo apt install iw"
        )
    proc = subprocess.run(
        [IW_BINARY, "dev"], capture_output=True, text=True, timeout=5.0, check=False
    )
    chosen = choose_interface(parse_iw_dev(proc.stdout or ""))
    if not chosen:
        raise ScanError("no wireless interfaces found by 'iw dev'")
    return chosen


def _channel_to_freq(channel: int) -> int | None:
    """Fallback for output that gives a channel but no ``freq:`` line."""
    if 1 <= channel <= 13:
        return 2407 + 5 * channel
    if channel == 14:
        return 2484
    if 32 <= channel <= 177:
        return 5000 + 5 * channel
    return None


def parse_iw_link(text: str) -> list[Observation]:
    """Parse ``iw dev <iface> link`` output for the currently associated AP."""
    bssid: str | None = None
    ssid: str | None = None
    freq: int | None = None
    rssi: float | None = None
    for line in text.splitlines():
        m = _RE_CONNECTED.match(line)
        if m:
            bssid = m.group(1).lower()
            continue
        if bssid is None:
            continue
        m = _RE_FREQ.match(line)
        if m:
            freq = int(m.group(1))
            continue
        m = _RE_SIGNAL.match(line)
        if m:
            rssi = float(m.group(1))
            continue
        m = _RE_SSID.match(line)
        if m:
            ssid = m.group(1).rstrip("\n")
            continue
    if bssid is not None and freq is not None and rssi is not None:
        return [Observation(bssid=bssid, ssid=ssid or "", freq_mhz=freq, rssi_dbm=rssi)]
    return []


def parse_iw_scan(text: str) -> list[Observation]:
    """Parse ``iw dev <iface> scan`` output into observations.

    Tolerant by design: a BSS block missing a frequency or a signal level is
    dropped rather than raising, because one malformed block in a sweep must
    not discard the other twenty. Blocks with no ``SSID:`` line, or an empty
    one, are kept with ``ssid=""`` -- a hidden network still contributes a
    real signal measurement and still belongs in the survey.
    """
    observations: list[Observation] = []

    bssid: str | None = None
    ssid: str | None = None
    freq: int | None = None
    rssi: float | None = None
    channel: int | None = None

    def flush() -> None:
        nonlocal bssid, ssid, freq, rssi, channel
        if bssid is not None:
            f = freq if freq is not None else (
                _channel_to_freq(channel) if channel is not None else None
            )
            if f is not None and rssi is not None:
                observations.append(
                    Observation(
                        bssid=bssid.lower(),
                        ssid=ssid if ssid is not None else "",
                        freq_mhz=f,
                        rssi_dbm=float(rssi),
                    )
                )
        bssid = ssid = freq = rssi = channel = None

    for line in text.splitlines():
        m = _RE_BSS.match(line)
        if m:
            flush()           # close out the previous block
            bssid = m.group(1)
            continue

        if bssid is None:
            continue          # preamble before the first BSS block

        m = _RE_FREQ.match(line)
        if m:
            freq = int(m.group(1))
            continue

        m = _RE_SIGNAL.match(line)
        if m:
            rssi = float(m.group(1))
            continue

        m = _RE_DS_CHAN.match(line)
        if m:
            channel = int(m.group(1))
            continue

        m = _RE_SSID.match(line)
        if m:
            ssid = m.group(1).rstrip("\n")
            continue

    flush()                   # final block has no sentinel after it
    return observations


def dedupe_strongest(observations: Iterable[Observation]) -> list[Observation]:
    """Collapse duplicate BSSIDs, keeping the strongest reading.

    A single sweep can report the same BSSID more than once when an AP is
    heard on multiple channels. Taking the strongest is the conservative
    choice for coverage mapping: it answers "could a client here associate",
    which is the question a site survey is actually asked.
    """
    best: dict[str, Observation] = {}
    for obs in observations:
        prev = best.get(obs.bssid)
        if prev is None or obs.rssi_dbm > prev.rssi_dbm:
            best[obs.bssid] = obs
    return sorted(best.values(), key=lambda o: o.rssi_dbm, reverse=True)


def scan(
    interface: str,
    timeout_s: float = DEFAULT_SCAN_TIMEOUT_S,
    use_sudo: bool = False,
) -> list[Observation]:
    """Run one scan sweep and return what was heard.

    Raises ``ScanError`` if the scan could not run. Returns an empty list if
    it ran and heard nothing -- that is a measurement, not an error.
    """
    if shutil.which(IW_BINARY) is None:
        raise ScanError(
            f"{IW_BINARY!r} not found on PATH. Install it with: "
            "sudo apt install iw"
        )
    interface = resolve_interface(interface)

    def _run_iw(*args: str) -> subprocess.CompletedProcess:
        cmd = [IW_BINARY, "dev", interface, *args]
        if use_sudo:
            cmd.insert(0, "sudo")
        return subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )

    try:
        proc = _run_iw("scan")
    except subprocess.TimeoutExpired as exc:
        raise ScanError(
            f"scan on {interface} exceeded {timeout_s}s. A dual-band sweep "
            "with DFS channels can be slow; raise scan_timeout_s if this "
            "happens consistently."
        ) from exc

    observations: list[Observation] = []
    if proc.returncode != 0:
        err = (proc.stderr or "").strip()
        # When the adapter is associated to the hotspot (go2w) or
        # NetworkManager is mid-scan, active trigger returns -16 (busy) or -1
        # (unprivileged). Fall back to cached scan dump + live link RSSI so
        # the survey keeps recording without dropping the SSH link.
        if "Device or resource busy" in err or "(-16)" in err or "Operation not permitted" in err:
            dump = _run_iw("scan", "dump")
            link = _run_iw("link")
            observations = parse_iw_scan(dump.stdout or "") + parse_iw_link(link.stdout or "")
            if observations or dump.returncode == 0:
                return dedupe_strongest(observations)
        if "Operation not permitted" in err:
            raise ScanError(
                f"scan on {interface} needs CAP_NET_ADMIN. Either run with "
                "use_sudo, or grant the capability once:\n"
                "  sudo setcap cap_net_admin+ep $(which iw)"
            )
        if "No such device" in err:
            raise ScanError(
                f"no interface named {interface!r}. List what exists with: "
                "iw dev"
            )
        raise ScanError(f"iw scan on {interface} failed (rc={proc.returncode}): {err}")

    observations = parse_iw_scan(proc.stdout)
    try:
        link = _run_iw("link")
        if link.returncode == 0:
            observations.extend(parse_iw_link(link.stdout or ""))
    except subprocess.TimeoutExpired:
        pass
    return dedupe_strongest(observations)


def _main(argv: list[str] | None = None) -> int:
    """Standalone entry point, for checking an adapter works before ROS does."""
    import argparse

    ap = argparse.ArgumentParser(
        description="Run one WiFi scan sweep and print what was heard."
    )
    ap.add_argument(
        "interface", nargs="?", default="auto",
        help="scanning interface, e.g. wlan1 (default: auto-detect via iw dev)"
    )
    ap.add_argument("--sudo", action="store_true", help="invoke iw via sudo")
    ap.add_argument(
        "--timeout", type=float, default=DEFAULT_SCAN_TIMEOUT_S, metavar="S"
    )
    args = ap.parse_args(argv)

    try:
        iface = resolve_interface(args.interface)
        results = scan(iface, timeout_s=args.timeout, use_sudo=args.sudo)
    except ScanError as exc:
        print(f"scan failed: {exc}")
        return 1

    if not results:
        print(f"interface {iface}: scan succeeded but heard nothing (valid result: no coverage)")
        return 0

    print(f"interface: {iface}")
    print(f"{'RSSI':>7}  {'BAND':>4}  {'FREQ':>5}  {'BSSID':<17}  SSID")
    for o in results:
        label = o.ssid or "<hidden>"
        print(
            f"{o.rssi_dbm:>7.1f}  {o.band:>4}  {o.freq_mhz:>5}  "
            f"{o.bssid:<17}  {label}"
        )
    print(f"\n{len(results)} access points")
    return 0


if __name__ == "__main__":
    raise SystemExit(_main())

"""Parser tests against captured ``iw scan`` output.

These run with plain stdlib unittest -- no radio, no root, no ROS, no numpy.
That is the point: parsing is the part of this package most likely to be
quietly wrong, so it is pinned before the hardware exists.

    python -m unittest discover -s go2_survey_wifi/test -v
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from go2_survey_wifi.scanner import (  # noqa: E402
    Observation,
    dedupe_strongest,
    parse_iw_scan,
)


# Captured from `iw dev wlan0 scan` on iw 5.16. Trimmed to the fields the
# parser reads, with the surrounding noise kept so the tests exercise the
# real shape of the input rather than an idealised version of it.
FIXTURE_TYPICAL = """\
BSS 3c:37:86:a1:b2:c3(on wlan0)
	TSF: 1203948571 usec (0d, 00:20:03)
	freq: 5180
	beacon interval: 100 TUs
	capability: ESS Privacy ShortSlotTime (0x0411)
	signal: -67.00 dBm
	last seen: 1240 ms ago
	SSID: CorpNet
	Supported rates: 6.0* 9.0 12.0* 18.0 24.0* 36.0 48.0 54.0
	HT capabilities:
		Capabilities: 0x19ef
BSS 3c:37:86:a1:b2:c4(on wlan0)
	freq: 2437
	signal: -45.50 dBm
	SSID: CorpNet
	DS Parameter set: channel 6
BSS 00:11:22:33:44:55(on wlan0)
	freq: 2412
	signal: -89.00 dBm
	SSID: Guest WiFi 2.4
BSS de:ad:be:ef:00:01(on wlan0) -- associated
	freq: 5745
	signal: -52.00 dBm
	SSID: CorpNet-5G
"""

# A hidden network reports an empty SSID; some APs omit the line entirely.
FIXTURE_HIDDEN = """\
BSS aa:bb:cc:dd:ee:f0(on wlan1)
	freq: 5500
	signal: -71.00 dBm
	SSID:
BSS aa:bb:cc:dd:ee:f1(on wlan1)
	freq: 5520
	signal: -73.00 dBm
"""

# Older output with no `freq:` line -- only the DS Parameter set channel.
FIXTURE_CHANNEL_ONLY = """\
BSS 11:22:33:44:55:66(on wlan0)
	signal: -60.00 dBm
	SSID: LegacyAP
	DS Parameter set: channel 11
"""

# Blocks that cannot yield a measurement must be dropped without taking the
# good ones with them.
FIXTURE_MALFORMED = """\
BSS 99:99:99:99:99:99(on wlan0)
	freq: 5180
	SSID: NoSignalReported
BSS 88:88:88:88:88:88(on wlan0)
	signal: -55.00 dBm
	SSID: NoFreqReported
BSS 77:77:77:77:77:77(on wlan0)
	freq: 2462
	signal: -61.00 dBm
	SSID: PerfectlyFine
"""


class TestParseTypical(unittest.TestCase):
    def setUp(self) -> None:
        self.obs = parse_iw_scan(FIXTURE_TYPICAL)

    def test_finds_every_bss(self) -> None:
        self.assertEqual(len(self.obs), 4)

    def test_fields_of_first_block(self) -> None:
        o = self.obs[0]
        self.assertEqual(o.bssid, "3c:37:86:a1:b2:c3")
        self.assertEqual(o.ssid, "CorpNet")
        self.assertEqual(o.freq_mhz, 5180)
        self.assertEqual(o.rssi_dbm, -67.0)

    def test_bssid_normalised_to_lowercase(self) -> None:
        # Mixed case in, lowercase out -- otherwise the same AP appears twice
        # in the survey depending on which iw version produced the line.
        obs = parse_iw_scan("BSS 3C:37:86:AA:BB:CC(on wlan0)\n\tfreq: 2412\n\tsignal: -50.00 dBm\n")
        self.assertEqual(obs[0].bssid, "3c:37:86:aa:bb:cc")

    def test_fractional_rssi_preserved(self) -> None:
        self.assertEqual(self.obs[1].rssi_dbm, -45.5)

    def test_ssid_with_spaces(self) -> None:
        self.assertEqual(self.obs[2].ssid, "Guest WiFi 2.4")

    def test_associated_suffix_does_not_break_bssid(self) -> None:
        # `-- associated` trails the BSSID on the interface's own network.
        self.assertEqual(self.obs[3].bssid, "de:ad:be:ef:00:01")
        self.assertEqual(self.obs[3].ssid, "CorpNet-5G")

    def test_same_ssid_on_distinct_bssids_kept_separate(self) -> None:
        # Two radios of one AP, or two APs of one ESS: both are real and must
        # not be merged, because AP placement is the question being answered.
        corpnet = [o for o in self.obs if o.ssid == "CorpNet"]
        self.assertEqual(len(corpnet), 2)
        self.assertNotEqual(corpnet[0].bssid, corpnet[1].bssid)


class TestBandClassification(unittest.TestCase):
    def test_bands(self) -> None:
        cases = [(2412, "2.4"), (2484, "2.4"), (5180, "5"), (5745, "5"), (6195, "6")]
        for freq, expected in cases:
            with self.subTest(freq=freq):
                o = Observation("aa:bb:cc:dd:ee:ff", "x", freq, -50.0)
                self.assertEqual(o.band, expected)

    def test_unknown_frequency_is_marked_not_guessed(self) -> None:
        o = Observation("aa:bb:cc:dd:ee:ff", "x", 999, -50.0)
        self.assertEqual(o.band, "?")


class TestHiddenNetworks(unittest.TestCase):
    def test_empty_and_absent_ssid_both_retained(self) -> None:
        # A hidden network still carries a real signal measurement, so it
        # belongs in a coverage survey even though it has no name.
        obs = parse_iw_scan(FIXTURE_HIDDEN)
        self.assertEqual(len(obs), 2)
        self.assertEqual(obs[0].ssid, "")
        self.assertEqual(obs[1].ssid, "")
        self.assertEqual(obs[1].rssi_dbm, -73.0)


class TestChannelFallback(unittest.TestCase):
    def test_channel_used_when_freq_line_absent(self) -> None:
        obs = parse_iw_scan(FIXTURE_CHANNEL_ONLY)
        self.assertEqual(len(obs), 1)
        self.assertEqual(obs[0].freq_mhz, 2462)   # channel 11 -> 2462 MHz

    def test_explicit_freq_wins_over_channel(self) -> None:
        # The second fixture block carries both; freq: is authoritative.
        obs = parse_iw_scan(FIXTURE_TYPICAL)
        self.assertEqual(obs[1].freq_mhz, 2437)


class TestMalformedBlocks(unittest.TestCase):
    def test_incomplete_blocks_dropped_good_block_survives(self) -> None:
        obs = parse_iw_scan(FIXTURE_MALFORMED)
        self.assertEqual(len(obs), 1)
        self.assertEqual(obs[0].ssid, "PerfectlyFine")

    def test_empty_input(self) -> None:
        self.assertEqual(parse_iw_scan(""), [])

    def test_preamble_without_bss_block(self) -> None:
        self.assertEqual(parse_iw_scan("command failed: Network is down (-100)\n"), [])


class TestDedupe(unittest.TestCase):
    def test_keeps_strongest_reading_per_bssid(self) -> None:
        obs = [
            Observation("aa:bb:cc:dd:ee:ff", "A", 2412, -70.0),
            Observation("aa:bb:cc:dd:ee:ff", "A", 2412, -55.0),
            Observation("aa:bb:cc:dd:ee:ff", "A", 2412, -80.0),
        ]
        out = dedupe_strongest(obs)
        self.assertEqual(len(out), 1)
        self.assertEqual(out[0].rssi_dbm, -55.0)

    def test_sorted_strongest_first(self) -> None:
        obs = [
            Observation("11:11:11:11:11:11", "weak", 2412, -88.0),
            Observation("22:22:22:22:22:22", "strong", 2412, -40.0),
            Observation("33:33:33:33:33:33", "mid", 2412, -65.0),
        ]
        out = dedupe_strongest(obs)
        self.assertEqual([o.ssid for o in out], ["strong", "mid", "weak"])

    def test_empty_in_empty_out(self) -> None:
        self.assertEqual(dedupe_strongest([]), [])


if __name__ == "__main__":
    unittest.main(verbosity=2)

"""Battery charge estimation."""

import unittest

from fossbot_bridge.battery import Smoother, liion_fraction, rail_fraction


class BatteryTests(unittest.TestCase):
    def test_liion_curve_endpoints_and_clamping(self):
        self.assertEqual(liion_fraction(4.20), 1.0)
        self.assertEqual(liion_fraction(4.35), 1.0)
        self.assertEqual(liion_fraction(3.30), 0.0)
        self.assertEqual(liion_fraction(3.00), 0.0)
        self.assertAlmostEqual(liion_fraction(3.80), 0.50)

    def test_liion_curve_is_monotonic(self):
        values = [liion_fraction(3.2 + i * 0.01) for i in range(110)]
        self.assertEqual(values, sorted(values))

    def test_rail_estimate_matches_observed_readings(self):
        # 4.90 V was a healthy, charged robot; the Pi flags undervoltage at 4.63.
        self.assertEqual(rail_fraction(4.90, 4.65, 4.90), 1.0)
        self.assertEqual(rail_fraction(4.60, 4.65, 4.90), 0.0)
        self.assertAlmostEqual(rail_fraction(4.775, 4.65, 4.90), 0.5)

    def test_rail_rejects_inverted_thresholds(self):
        with self.assertRaises(ValueError):
            rail_fraction(4.8, 4.9, 4.6)

    def test_smoother_tracks_slowly_and_converges(self):
        s = Smoother(10.0)
        self.assertEqual(s.update(4.90, 0.0), 4.90)
        dipped = s.update(4.50, 1.0)          # one-second sag under load
        self.assertGreater(dipped, 4.80)      # a brief dip barely moves it
        for t in range(2, 200):
            v = s.update(4.70, float(t))
        self.assertAlmostEqual(v, 4.70, places=3)


if __name__ == "__main__":
    unittest.main()

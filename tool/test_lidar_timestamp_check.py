#!/usr/bin/env python3
import unittest
from lidar_timestamp_check import analyze


def rows(front, back):
    return [dict(source=s, stamp_ns=t, receive_monotonic_ns=i*10_000_000)
            for s, values in [('front', front), ('back', back)] for i, t in enumerate(values)]


class TimestampTest(unittest.TestCase):
    def setUp(self):
        self.front = [10**18+i*10_000_000 for i in range(200)]

    def test_good(self):
        r = analyze(rows(self.front, [t+3_000_000 for t in self.front]))
        self.assertEqual(r['window_pairs'], 200)
        self.assertEqual(r['nearest_abs_delta_ms']['p95'], 3)
        self.assertEqual(r['streams']['front']['stamp_interval_ms']['mean'], 10)

    def test_outside_window(self):
        r = analyze(rows(self.front, [t+5_000_000 for t in self.front]), tolerance_ms=1)
        self.assertEqual(r['window_pairs'], 0)
        self.assertEqual(r['nearest_abs_delta_ms']['p95'], 5)

    def test_missing_frames(self):
        r = analyze(rows(self.front, self.front[::2]))
        self.assertEqual(r['coverage']['front'], .5)
        self.assertEqual(r['verdict'], '时间戳检查未达标')

    def test_backwards(self):
        bad = self.front.copy()
        bad[50] = bad[49]-1
        r = analyze(rows(self.front, bad))
        self.assertEqual(r['streams']['back']['non_increasing'], 1)
        self.assertEqual(r['verdict'], '时间戳检查未达标')

    def test_no_data(self):
        r = analyze([])
        self.assertEqual(r['verdict'], '样本不足')
        self.assertIsNone(r['nearest_abs_delta_ms'])


if __name__ == '__main__':
    unittest.main()

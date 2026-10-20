import unittest
from types import SimpleNamespace

import obstacle_sensing as osn


class StaticObstacleIdTests(unittest.TestCase):
    def test_distinct_objects_have_distinct_ids(self):
        a = SimpleNamespace(id=101)
        b = SimpleNamespace(id=102)

        first = osn.static_obstacle_id(a, label_id=11)
        second = osn.static_obstacle_id(b, label_id=11)

        self.assertIsInstance(first, int)
        self.assertIsInstance(second, int)
        self.assertGreaterEqual(first, 0)
        self.assertGreaterEqual(second, 0)
        self.assertNotEqual(first, second)
        self.assertEqual(first, osn.static_obstacle_id(a, label_id=11))
        self.assertNotIn(first, (900001, 900002))
        self.assertNotIn(second, (900001, 900002))


if __name__ == "__main__":
    unittest.main()

import unittest
from OCC.Core.gp import gp_Pnt

from src.line.spline import Spline

class TestSpline(unittest.TestCase):
    def setUp(self):
        path_pnts = [
            gp_Pnt(0, 0, 0),
            gp_Pnt(1, 1, 1),
            gp_Pnt(2, 2, 2),
            gp_Pnt(3, 3, 3)
        ]
        diameter = 1.0
        self.spline = Spline(path_pnts, diameter)

    def test_set_intermediate_pnts(self):
        intermediate_pnts = [
            gp_Pnt(1, 1, 1),
            gp_Pnt(2, 2, 2)
        ]
        self.spline.set_intermediate_pnts(intermediate_pnts)
        self.assertEqual(self.spline.intermediate_pnts, intermediate_pnts)

if __name__ == '__main__':
    unittest.main()
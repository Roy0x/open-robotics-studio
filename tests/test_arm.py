import math
import unittest
from robotics_studio.arm import Arm


class ReachTests(unittest.TestCase):
    def test_two_solutions_and_forward_round_trip(self):
        result = Arm().solve(0.6, 0.3)
        self.assertEqual(result['status'], 'reachable')
        self.assertEqual(len(result['solutions']), 2)
        for solution in result['solutions']:
            self.assertLess(solution['position_error_m'], 1e-12)

    def test_workspace_hole_and_outer_boundary(self):
        self.assertEqual(Arm().solve(0,0)['status'], 'unreachable')
        self.assertEqual(Arm().solve(1.1,0)['status'], 'unreachable')
        self.assertEqual(len(Arm().solve(1,0)['solutions']), 1)
        self.assertEqual(Arm().solve(0.2,0)['status'], 'reachable')

    def test_joint_limits(self):
        arm = Arm(limits=((-math.pi, math.pi), (0, math.pi)))
        self.assertEqual(len(arm.solve(0.6,0.3)['solutions']), 1)
        self.assertEqual(Arm(limits=((0,0),(0,0))).solve(0.6,0.3)['status'], 'joint_limit_blocked')

    def test_negative_x_boundary(self):
        self.assertEqual(Arm(limits=((0,math.pi),(0,math.pi))).solve(-1,0)['status'], 'reachable')

    def test_degenerate_origin(self):
        self.assertEqual(Arm(0.5,0.5).solve(0,0)['status'], 'degenerate')

    def test_invalid_inputs(self):
        for lengths in ((0,1), (-1,1), (float('nan'),1), (float('inf'),1)):
            with self.assertRaises(ValueError):
                Arm(*lengths)
        with self.assertRaises(ValueError):
            Arm().solve(float('nan'),0)
        with self.assertRaises(ValueError):
            Arm(limits=((1,-1),(0,1)))

    def test_round_trip_grid(self):
        arm = Arm()
        for q1 in (-2,-1,0,1,2):
            for q2 in (-2,-1,0,1,2):
                _, tip = arm.forward(q1,q2)
                result = arm.solve(*tip)
                self.assertEqual(result['status'], 'reachable')
                self.assertTrue(all(s['position_error_m'] < 1e-10 for s in result['solutions']))


if __name__ == '__main__':
    unittest.main()

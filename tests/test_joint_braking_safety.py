"""Actual-command safety under synchronous turns and finite braking."""

from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'example'))
from mppi_safety import JointBrakingSafetyFilter


def parameters(n):
    return [dict(size=.3, v_min=0., v_max=1.2, w_min=-1.8, w_max=1.8,
                 v_accel_max=.8, v_brake_max=1.2, w_accel_max=2.) for _ in range(n)]


class BrakingSafetyTests(unittest.TestCase):
    def run_commands(self, states, desired, obstacles, steps):
        params = parameters(len(states)); profiles = [dict(rad_eps=.04) for _ in states]
        safety = JointBrakingSafetyFilter(params, profiles, obstacles, .1)
        previous = np.zeros((len(states), 2)); records = [states.copy()]; controls = []
        for _ in range(steps):
            applied = safety(states, desired, previous)
            change = applied-previous
            self.assertTrue(np.all(change[:,0] <= .08+1e-10))
            self.assertTrue(np.all(change[:,0] >= -.12-1e-10))
            self.assertTrue(np.all(abs(change[:,1]) <= .2+1e-10))
            self.assertTrue(safety.safe(states, safety.certificate))
            states[:,:2] += .1*applied[:,:1]*np.c_[np.cos(states[:,2]),np.sin(states[:,2])]
            states[:,2] += .1*applied[:,1]
            previous = applied; records.append(states.copy()); controls.append(applied.copy())
        return np.asarray(records), np.asarray(controls), safety

    def test_head_on_simultaneous_replanning_preserves_braking_room(self):
        states = np.array([[-2.,0.,0.,0.,0.],[2.,0.,np.pi,0.,0.]])
        trace, controls, safety = self.run_commands(states,np.array([[1.2,0.],[1.2,0.]]),[],100)
        self.assertGreaterEqual(np.linalg.norm(trace[:,0,:2]-trace[:,1,:2],axis=1).min(),.68-1e-9)
        self.assertGreater(controls[:,:,0].max(),1.)
        self.assertLess(controls[-1,:,0].max(),.01)
        self.assertGreater(safety.interventions,0)

    def test_turning_commands_stop_before_a_static_wall(self):
        wall = [[[1.,-10.],[1.1,-10.],[1.1,10.],[1.,10.]]]
        states=np.zeros((1,5))
        trace, _, safety = self.run_commands(states,np.array([[1.2,.3]]),wall,80)
        self.assertLessEqual(trace[:,:,0].max(),.66+1e-9)
        self.assertGreater(safety.interventions,0)

    def test_free_fleet_accepts_nominal_at_the_speed_limit(self):
        trace, controls, safety = self.run_commands(np.zeros((1,5)),np.array([[1.2,0.]]),[],40)
        self.assertAlmostEqual(controls[-1,0,0],1.2)
        self.assertEqual(safety.interventions,0)
        self.assertGreater(trace[-1,0,0],3.)

    def test_uncertifiable_initial_motion_is_an_error(self):
        safety=JointBrakingSafetyFilter(parameters(1),[dict(rad_eps=.04)],
            [[[.35,-2.],[.45,-2.],[.45,2.],[.35,2.]]],.1)
        with self.assertRaisesRegex(RuntimeError,'certificate'):
            safety(np.zeros((1,5)),np.array([[1.2,0.]]),np.array([[1.2,0.]]))


if __name__=='__main__':unittest.main()

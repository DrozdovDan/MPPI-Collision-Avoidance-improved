"""Regression tests for body removal, physical arrivals and paired execution."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'benchmarks'))
sys.path.insert(0,str(ROOT/'example'))
from multi_agent.warehouse_disappear_runtime import DisappearBrakingSafetyFilter, launch_disappear_mppi
from multi_agent.warehouse_disappear_evaluation import DisappearTerminalMonitor, disappear_disk_collisions
from multi_agent.warehouse_evaluation import EvaluationConfig, TerminalMonitor
from multi_agent.metrics import validate_execution
from mppi_safety import JointBrakingSafetyFilter
from bench_mr import parameters
from dyn_models import DiffDriveDyn
from params_names import *
from utils import launch_mppi


def params(n):
    return [dict(size=.2,r_vis=10.,v_min=0.,v_max=1.2,w_min=-1.8,w_max=1.8,
        v_accel_max=.8,v_brake_max=1.2,w_accel_max=2.) for _ in range(n)]


def task(goals,starts):
    return dict(dt=.1,robot_count=len(goals),robots=[dict(id=f'r{i}',radius=.2,start=[x,0.,0.],
        goal=dict(pose=[g,0.,0.],radius=.15),model='diff_drive',motion_limits=params(1)[0])
        for i,(x,g) in enumerate(zip(starts,goals))])


class DisappearanceTests(unittest.TestCase):
    def test_retired_disk_is_absent_from_safety_but_active_disk_is_present(self):
        safety=DisappearBrakingSafetyFilter(params(2),[dict(rad_eps=.04)]*2,[],.1)
        states=np.array([[0.,0.,0.,0.,0.],[.2,0.,0.,0.,0.]])
        plan=safety.plan(np.zeros((2,2)))
        self.assertFalse(safety.safe(states,plan))
        safety.set_active([True,False])
        self.assertTrue(safety.safe(states,plan))
        with self.assertRaisesRegex(ValueError,'monotonically'):safety.set_active([True,True])

    def test_static_obstacles_remain_active_after_other_robot_disappears(self):
        wall=[[[.1,-2.],[.3,-2.],[.3,2.],[.1,2.]]]
        safety=DisappearBrakingSafetyFilter(params(2),[dict(rad_eps=.04)]*2,wall,.1)
        safety.set_active([True,False])
        self.assertFalse(safety.safe(np.zeros((2,5)),safety.plan(np.array([[.08,0.],[0.,0.]]))))

    def test_incoming_interval_is_checked_before_disappearance(self):
        t=task([0.,1.],[0.,1.]);world=dict(bounds=[-10.,-10.,10.,10.],obstacles=[])
        states=np.zeros((2,2,5));states[0,:,:2]=[[0.,0.],[1.,0.]];states[1,:,:2]=[[1.,0.],[0.,0.]]
        active=np.array([[True,True],[False,False]])
        self.assertEqual(disappear_disk_collisions(t,world,states,active)['collisions'],1)
        active=np.array([[True,False],[True,False]])
        self.assertEqual(disappear_disk_collisions(t,world,states,active)['collisions'],0)

    def test_disappeared_goal_cannot_create_parked_goal_deadlock(self):
        t=task([8.,.8],[0.,.8]);paths=[[[0.,0.],[8.,0.]],[[.8,0.],[.9,0.],[.8,0.]]]
        cfg=EvaluationConfig(deadlock_window_seconds=3.,stall_window_seconds=5.)
        states=np.c_[[r['start'] for r in t['robots']],np.zeros((2,2))]
        old=TerminalMonitor(t,paths,cfg);new=DisappearTerminalMonitor(t,paths,cfg)
        old_result=None;new_result=None
        for step in range(1,70):
            if old_result is None:
                status,evidence=old.observe(step,states,np.zeros((2,2)))
                if status:old_result=(status,evidence)
            if new_result is None:
                status,evidence=new.observe(step,states,np.zeros((2,2)),np.array([True,False]))
                if status:new_result=(status,evidence)
        self.assertEqual(old_result[0],'deadlock')
        self.assertEqual(new_result[0],'stalled')
        self.assertEqual(new_result[1]['obstruction_edges'],[])
        self.assertEqual(new_result[1]['parked_goal_blocker_robot_ids'],[])

    def test_native_execution_matches_original_before_any_arrival(self):
        agents=params(2);profiles=[parameters()|dict(batch_size=16,time_steps=8,ma_cost_en=False)]*2
        start=np.array([[0.,0.,0.,0.,0.],[0.,3.,0.,0.,0.]])
        goals=np.array([[30.,0.,0.,0.,0.],[30.,3.,0.,0.,0.]])
        records=[]
        for launch,cls in ((launch_mppi,JointBrakingSafetyFilter),(launch_disappear_mppi,DisappearBrakingSafetyFilter)):
            controls=[]
            def observe(step,states,u,seconds,*presence):controls.append(u.copy())
            result=launch(start,goals,2,agents,profiles,
                {MAX_EXP_STEPS:30,EXP_TIME_STEP:.1,XY_GOAL_TOLERANCE:.15},[DiffDriveDyn(),DiffDriveDyn()],
                seed=42,anchor_paths=[[[0.,0.],[30.,0.]],[[0.,3.],[30.,3.]]],obstacles=[],
                park_at_goal=True,on_step=observe,safety_filter=cls(agents,profiles,[],.1))
            records.append((np.array(result[-1]),np.array(controls)))
        for a,b in zip(*records):np.testing.assert_array_equal(a,b)

    def test_settled_removal_releases_neighbors_and_does_not_teleport(self):
        class Stub:
            def __init__(self):self.neighbors=[]
            def set_agent_params(self,x):pass
            def set_alg_params(self,x):pass
            def set_obstacles(self,x):pass
            def set_anchor_path(self,x):pass
            def set_applied_control(self,x):pass
            def next_step(self,state,neighbors,radii,goal):
                self.neighbors.append(neighbors.copy())
                return np.array([.4,0.]),np.empty((0,0,5))
        controllers=[Stub(),Stub()];agents=params(2);profiles=[parameters()]*2
        start=np.array([[0.,0.,0.,0.,0.],[.8,0.,0.,0.,0.]])
        goals=np.array([[2.2,0.,0.,0.,0.],[.8,0.,0.,0.,0.]])
        controls=[];presence=[np.ones(2,dtype=bool)]
        def observe(step,states,u,seconds,active):controls.append(u.copy());presence.append(active.copy())
        with patch('multi_agent.warehouse_disappear_runtime.mppi_ca_cpp.Controller',side_effect=controllers):
            result=launch_disappear_mppi(start,goals,2,agents,profiles,
                {MAX_EXP_STEPS:120,EXP_TIME_STEP:.1,XY_GOAL_TOLERANCE:.15},[DiffDriveDyn(),DiffDriveDyn()],seed=42,
                park_at_goal=True,on_step=observe,safety_filter=DisappearBrakingSafetyFilter(agents,profiles,[],.1),obstacles=[])
        self.assertTrue(result[0]);states=np.array(result[-1]);controls=np.array(controls);presence=np.array(presence)
        self.assertEqual(controllers[0].neighbors[0].shape,(1,4))
        self.assertTrue(all(len(n)==0 for n in controllers[0].neighbors[1:]))
        self.assertEqual(len(controllers[1].neighbors),0)
        self.assertGreater(states[-1,0,0],2.)
        np.testing.assert_array_equal(states[:,1],np.broadcast_to(start[1],states[:,1].shape))
        arrival=np.flatnonzero(~presence[:,0])[0]
        self.assertTrue(np.all(controls[arrival-1,0]==0))
        self.assertGreater(controls[arrival-2,0,0],0.)
        t=task([2.2,.8],[0.,.8])
        self.assertTrue(validate_execution(t,states,controls)['passed'])
        self.assertEqual(disappear_disk_collisions(t,dict(bounds=[-10.,-10.,10.,10.],obstacles=[]),states,presence)['collisions'],0)


if __name__=='__main__':unittest.main()

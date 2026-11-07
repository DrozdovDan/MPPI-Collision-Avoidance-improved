"""Measured stopping, unbiased aggregation and native rate-limit regression."""

import copy
import json
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"benchmarks"))
sys.path.insert(0,str(ROOT/"build/release"))
from multi_agent.warehouse_evaluation import EvaluationConfig,TerminalMonitor,aggregate_rows,disk_collisions


def task_pair(goals):
    return dict(dt=.1,robot_count=2,robots=[dict(id=f"r{i}",start=[x,2.,0.],radius=.2,
        goal=dict(pose=[goal,2.,0.],radius=.15)) for i,(x,goal) in enumerate(zip((2.,2.55),goals))])


class EvaluationTests(unittest.TestCase):
    def observe_static(self,task,duration):
        cfg=EvaluationConfig(deadlock_window_seconds=3.,stall_window_seconds=5.)
        paths=[[r["start"][:2],r["goal"]["pose"][:2]] for r in task["robots"]]
        monitor=TerminalMonitor(task,paths,cfg)
        states=np.c_[[r["start"] for r in task["robots"]],np.zeros((2,2))]
        result=(None,None)
        for step in range(1,int(duration/.1)+1):
            result=monitor.observe(step,states,np.zeros((2,2)))
            if result[0]:break
        return result

    def test_mutual_obstruction_requires_measured_duration(self):
        task=task_pair([8.,1.])
        self.assertIsNone(self.observe_static(task,2.)[0])
        status,evidence=self.observe_static(task,5.)
        self.assertEqual(status,"deadlock")
        self.assertEqual(evidence["blocked_robot_ids"],[0,1])
        json.dumps(evidence,allow_nan=False)

    def test_same_direction_stationary_queue_is_a_stall_not_mutual_deadlock(self):
        status,evidence=self.observe_static(task_pair([8.,9.]),7.)
        self.assertEqual(status,"stalled")
        self.assertFalse(evidence["mutual_obstruction_groups"])
        json.dumps(evidence,allow_nan=False)

    def test_no_progress_wall_clock_is_not_used(self):
        task=task_pair([8.,1.])
        self.assertIsNone(self.observe_static(task,1.)[0])

    def test_stationary_queue_into_a_parked_goal_is_a_deadlock(self):
        task=task_pair([8.,2.55])
        status,evidence=self.observe_static(task,5.)
        self.assertEqual(status,'deadlock')
        self.assertEqual(evidence['deadlock_type'],'parked_goal_obstruction')
        self.assertEqual(evidence['parked_goal_blocker_robot_ids'],[1])
        self.assertEqual(evidence['blocked_robot_ids'],[0])

    def test_route_obstruction_detects_a_queue_with_an_avoidance_gap(self):
        task=task_pair([8.,3.3]);task['robots'][1]['start'][0]=3.3
        status,evidence=self.observe_static(task,5.)
        self.assertEqual(status,'deadlock')
        self.assertEqual(evidence['parked_goal_blocker_robot_ids'],[1])

    def test_parked_robot_behind_the_route_is_not_a_deadlock(self):
        task=task_pair([8.,2.55]);task['robots'][0]['start'][0]=3.3
        status,evidence=self.observe_static(task,7.)
        self.assertEqual(status,'stalled')
        self.assertEqual(evidence['parked_goal_blocker_robot_ids'],[])

    def test_failed_makespans_are_not_averaged_as_zero(self):
        rows=[dict(planner="rrt",success=True,makespan_seconds=100.,completion_fraction=1.,status="success"),
              dict(planner="rrt",success=False,makespan_seconds=None,completion_fraction=.5,status="deadlock")]
        row,=aggregate_rows(rows,["planner"])
        self.assertEqual(row["fleet_success_rate"],.5)
        self.assertEqual(row["mean_agent_completion_fraction"],.75)
        self.assertEqual(row["successful_makespan_mean_seconds"],100.)
        self.assertEqual(row["successful_makespan_sample_count"],1)

    def test_all_failure_group_has_no_makespan(self):
        row,=aggregate_rows([dict(planner="k_routes",success=False,makespan_seconds=None,
            completion_fraction=0.,status="global_planning_failure")],["planner"])
        self.assertIsNone(row["successful_makespan_mean_seconds"])
        self.assertEqual(row["fleet_success_rate"],0.)

    def test_paired_makespan_uses_only_the_same_successful_task(self):
        from multi_agent.warehouse_evaluation_report import paired_table
        rows=[]
        for task_id,method,success,seconds in (("a","rrt",True,100.),("a","k_routes",True,80.),
            ("b","rrt",False,None),("b","k_routes",True,120.)):
            rows.append(dict(task_id=task_id,scenario_id="scenario",planner=method,success=success,
                makespan_seconds=seconds,completion_fraction=1. if success else .5))
        row,=paired_table(rows,["scenario_id"])
        self.assertEqual(row["paired_tasks"],2)
        self.assertEqual(row["both_methods_successful_tasks"],1)
        self.assertEqual(row["paired_makespan_difference_k_minus_rrt_seconds"],-20.)
        self.assertEqual(row["success_rate_difference_k_minus_rrt"],.5)

    def test_fast_disk_collision_detects_crossing_between_samples(self):
        task=task_pair([8.,1.]);world=dict(bounds=[0.,0.,10.,10.],obstacles=[])
        states=np.zeros((2,2,5));states[0,:,:2]=[[2.,2.],[3.,2.]];states[1,:,:2]=[[3.,2.],[2.,2.]]
        result=disk_collisions(task,world,states)
        self.assertEqual(result["collisions"],1)
        self.assertLess(result["minimum_inter_robot_clearance"],0)

    def test_deadlock_audit_uses_a_visible_join_and_requires_the_same_blocker(self):
        from multi_agent.warehouse_evaluation_report import audit_deadlock_geometry
        from multi_agent.warehouse_evaluation import load_task
        fixture=json.loads((ROOT/'tests/fixtures/warehouse_deadlock_projection.json').read_text())
        task,world,_=load_task(ROOT/'benchmarks/scenarios/warehouse_uniform',fixture['task_id'])
        task=task|dict(robot_count=2,robots=[task['robots'][i] for i in fixture['robot_indices']])
        states=np.repeat(np.array(fixture['states'])[None],301,axis=0)
        evidence=dict(blocked_robot_ids=[0],obstruction_edges=[[0,1]])
        result=audit_deadlock_geometry(task,world,fixture['paths'],states,evidence,EvaluationConfig())
        correction,=result['visible_projection_corroborations']
        self.assertLess(correction['original_join_clearance_meters'],0.)
        self.assertGreater(correction['visible_join_clearance_meters'],.04)
        self.assertEqual(correction['preserved_obstruction_edges'],{'1':1.})
        states[:,1,1]-=10.
        with self.assertRaisesRegex(ValueError,'not corroborated'):
            audit_deadlock_geometry(task,world,fixture['paths'],states,evidence,EvaluationConfig())

    def test_native_emergency_fallback_respects_yaw_and_linear_braking(self):
        import mppi_ca_cpp as mppi
        controller=mppi.Controller()
        controller.set_agent_params(mppi.convert_agent_params(dict(size=.3,r_vis=10.,v_min=0.,v_max=1.2,
            w_min=-1.8,w_max=1.8,v_accel_max=.8,v_brake_max=1.2,w_accel_max=2.)))
        from bench_mr import parameters
        settings=parameters()|dict(batch_size=8,time_steps=5,dt=.1,
            init_value=np.array([.4,1.2]),control_mean=np.zeros(2),control_cov=np.eye(2),ma_cost_en=False)
        controller.set_alg_params(mppi.convert_alg_params(settings))
        controller.set_obstacles([[[-100.,-100.],[100.,-100.],[100.,100.],[-100.,100.]]])
        state=np.array([0.,0.,0.,.4,0.])
        u,_=controller.next_step(state,np.empty((0,4)),np.empty(0),np.array([10.,0.,0.]))
        np.testing.assert_allclose(u,[.28,1.0],atol=1e-10)

    def test_native_rate_limits_use_the_command_actually_applied(self):
        import mppi_ca_cpp as mppi
        from bench_mr import parameters
        controller=mppi.Controller()
        controller.set_agent_params(mppi.convert_agent_params(dict(size=.3,r_vis=10.,v_min=0.,v_max=1.2,
            w_min=-1.8,w_max=1.8,v_accel_max=.8,v_brake_max=1.2,w_accel_max=2.)))
        controller.set_alg_params(mppi.convert_alg_params(parameters()|dict(batch_size=8,time_steps=5,ma_cost_en=False)))
        controller.set_applied_control(np.array([.8,1.]))
        u,_=controller.next_step(np.array([0.,0.,0.,.8,0.]),np.empty((0,4)),np.empty(0),np.array([10.,0.,0.]))
        self.assertGreaterEqual(u[0],.68-1e-9);self.assertLessEqual(u[0],.88+1e-9)
        self.assertGreaterEqual(u[1],.8-1e-9);self.assertLessEqual(u[1],1.2+1e-9)

    def test_dense_anchor_projection_preserves_the_geometric_control(self):
        import mppi_ca_cpp as mppi
        from bench_mr import parameters
        corners=np.array([[0.,0.],[15.,0.],[15.,15.],[30.,15.]])
        dense=np.vstack([np.linspace(a,b,1000,endpoint=False) for a,b in zip(corners,corners[1:])]+[corners[-1:]])
        outputs=[]
        for path in [corners,dense]:
            controller=mppi.Controller()
            controller.set_agent_params(mppi.convert_agent_params(dict(size=.3,r_vis=10.,v_min=0.,v_max=1.2,
                w_min=-1.8,w_max=1.8,v_accel_max=.8,v_brake_max=1.2,w_accel_max=2.)))
            controller.set_alg_params(mppi.convert_alg_params(parameters()|dict(batch_size=16,time_steps=8,
                ma_cost_en=False,anchor_guided_fraction=1.,anchor_path_weight=2000.)))
            controller.set_anchor_path(path.tolist());mppi.set_random_seed(42)
            u,_=controller.next_step(np.array([3.,.1,0.,0.,0.]),np.empty((0,4)),np.empty(0),np.r_[corners[-1],0.])
            outputs.append(u)
        np.testing.assert_allclose(outputs[0],outputs[1],atol=1e-8)

    def test_anchor_reacquisition_does_not_credit_progress_through_shelving(self):
        import mppi_ca_cpp as mppi
        from multi_agent.warehouse_evaluation import load_task
        from multi_agent.runner import controller_profile
        from multi_agent.geometry import agent_parameters
        from bench_mr import boundary_obstacles
        task,world,_=load_task(ROOT/'benchmarks/scenarios/warehouse_uniform','warehouse_s02_v01')
        robot=task['robots'][27]
        # Recorded failure: the closer later aisle is across solid shelving.
        # The robot must reacquire the earlier corner before traversing it.
        path=[[26.1571699225,-70.0558764720],[30.1105542565,-71.4520121065],
              [30.3119606817,-72.0764791218],[30.3335132343,-75.1122559933],
              [30.8143005598,-75.6619651568],[43.0542206580,-75.9570015078]]
        controller=mppi.Controller()
        controller.set_agent_params(mppi.convert_agent_params(agent_parameters(robot)))
        profile=controller_profile(task)[27]|dict(batch_size=128,time_steps=31)
        controller.set_alg_params(mppi.convert_alg_params(profile))
        controller.set_obstacles(world['obstacles']+boundary_obstacles(world['bounds']))
        controller.set_anchor_path(path);mppi.set_random_seed(42)
        state=np.array([32.27,-71.38,-.14,0.,0.]);states=[state.copy()];controls=[]
        for _ in range(110):
            u,_=controller.next_step(state,np.empty((0,4)),np.empty(0),np.array(robot['goal']['pose']))
            velocity=u[0]*np.array([np.cos(state[2]),np.sin(state[2])])
            state=state+np.r_[velocity*.1,u[1]*.1,velocity-state[3:]]
            states.append(state.copy());controls.append(u.copy())
        self.assertLess(state[1],-75.)
        self.assertLess(state[0],34.)
        self.assertGreaterEqual(np.min(np.array(controls)[:,0]),0.)
        from multi_agent.metrics import validate_execution
        single=task|dict(robot_count=1,robots=[robot])
        self.assertTrue(validate_execution(single,np.array(states)[:,None,:],np.array(controls)[:,None,:])['passed'])
        self.assertEqual(disk_collisions(single,world,np.array(states)[:,None,:])['collisions'],0)

    def test_off_anchor_reacquisition_turns_before_entering_a_shelf_corner(self):
        import mppi_ca_cpp as mppi
        from multi_agent.warehouse_evaluation import load_task
        from multi_agent.runner import controller_profile
        from multi_agent.geometry import agent_parameters
        from multi_agent.metrics import validate_execution
        from bench_mr import boundary_obstacles
        from shapely.geometry import LineString,Point
        fixture=json.loads((ROOT/'tests/fixtures/warehouse_dd_reacquisition.json').read_text())
        task,world,_=load_task(ROOT/'benchmarks/scenarios/warehouse_uniform',fixture['task_id'])
        index=fixture['robot_index'];robot=task['robots'][index]
        controller=mppi.Controller()
        controller.set_agent_params(mppi.convert_agent_params(agent_parameters(robot)))
        controller.set_alg_params(mppi.convert_alg_params(controller_profile(task)[index]|
            dict(batch_size=128,time_steps=31,anchor_yield_after=15.)))
        controller.set_obstacles(world['obstacles']+boundary_obstacles(world['bounds']))
        controller.set_anchor_path(fixture['path']);mppi.set_random_seed(42)
        state=np.array(fixture['state']);states=[state.copy()];controls=[]
        for _ in range(100):
            u,_=controller.next_step(state,np.empty((0,4)),np.empty(0),np.array(robot['goal']['pose']))
            velocity=u[0]*np.array([np.cos(state[2]),np.sin(state[2])])
            state=state+np.r_[velocity*.1,u[1]*.1,velocity-state[3:]]
            states.append(state.copy());controls.append(u.copy())
        path=LineString(fixture['path'])
        self.assertLess(path.distance(Point(state[:2])),.1)
        self.assertGreater(path.project(Point(state[:2]))-path.project(Point(states[0][:2])),5.)
        self.assertGreaterEqual(np.min(np.array(controls)[:,0]),0.)
        single=task|dict(robot_count=1,robots=[robot])
        self.assertTrue(validate_execution(single,np.array(states)[:,None,:],np.array(controls)[:,None,:])['passed'])
        self.assertEqual(disk_collisions(single,world,np.array(states)[:,None,:])['collisions'],0)

    def test_blocked_no_progress_waits_and_resumes_when_the_route_clears(self):
        import mppi_ca_cpp as mppi
        from bench_mr import parameters
        def controller():
            c=mppi.Controller()
            c.set_agent_params(mppi.convert_agent_params(dict(size=.3,r_vis=10.,v_min=0.,v_max=1.2,
                w_min=-1.8,w_max=1.8,v_accel_max=.8,v_brake_max=1.2,w_accel_max=2.)))
            c.set_alg_params(mppi.convert_alg_params(parameters()|dict(batch_size=16,time_steps=31,
                anchor_guided_fraction=.5,anchor_yield_after=.5,anchor_path_weight=2000.)))
            c.set_anchor_path([[0.,0.],[10.,0.]])
            return c
        state=np.zeros(5);goal=np.array([10.,0.,0.]);radius=np.array([.3])
        for lateral,blocked in [(0.,True),(1.,False)]:
            with self.subTest(blocker_lateral=lateral):
                c=controller();mppi.set_random_seed(42)
                neighbor=np.array([[2.5,lateral,0.,0.]])
                # Measured feedback is held and acknowledged, as when an
                # actuator safety layer rejects nominal forward proposals.
                for _ in range(40):
                    u,_=c.next_step(state,neighbor,radius,goal)
                    c.set_applied_control(np.zeros(2))
                if blocked:
                    np.testing.assert_allclose(u,[0.,0.],atol=1e-12)
                    for _ in range(30):
                        u,_=c.next_step(state,np.empty((0,4)),np.empty(0),goal)
                        c.set_applied_control(np.zeros(2))
                    self.assertGreater(u[0],.01)
                else:self.assertGreater(u[0],.01)


if __name__=="__main__":unittest.main()

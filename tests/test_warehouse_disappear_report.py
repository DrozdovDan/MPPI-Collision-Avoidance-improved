"""A removal sample records the velocity of its incoming, present interval."""
from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'benchmarks'))
from multi_agent.warehouse_disappear_report import audit_removed_snapshot,audit_deadlock_subgraph


class SnapshotAuditTests(unittest.TestCase):
    def trace(self):
        states=np.zeros((4,1,5))
        states[:,:,:3]=[59.5,-8.3,1.5]
        states[0,0,1]-=2e-10
        states[1,0,3:5]=(states[1,0,:2]-states[0,0,:2])/.1
        controls=np.zeros((3,1,2))
        controls[0,0,0]=2e-9
        return states,controls

    def test_incoming_settled_velocity_then_zero_is_valid(self):
        states,controls=self.trace()
        self.assertGreater(abs(states[1,0,4]),0)
        audit_removed_snapshot(states,controls,0,1,.1)

    def test_any_pose_drift_is_rejected(self):
        for column in range(3):
            with self.subTest(column=column):
                states,controls=self.trace();states[2,0,column]+=1e-12
                with self.assertRaisesRegex(ValueError,'pose moved'):
                    audit_removed_snapshot(states,controls,0,1,.1)

    def test_post_removal_control_is_rejected(self):
        states,controls=self.trace();controls[1,0,0]=1e-12
        with self.assertRaisesRegex(ValueError,'received controls'):
            audit_removed_snapshot(states,controls,0,1,.1)

    def test_inconsistent_incoming_velocity_is_rejected(self):
        states,controls=self.trace();states[1,0,3]=1e-12
        with self.assertRaisesRegex(ValueError,'incoming displacement'):
            audit_removed_snapshot(states,controls,0,1,.1)

    def test_nonzero_later_velocity_is_rejected(self):
        states,controls=self.trace();states[2,0,4]=1e-12
        with self.assertRaisesRegex(ValueError,'after removal'):
            audit_removed_snapshot(states,controls,0,1,.1)

    def test_removal_on_last_sample_has_no_outgoing_interval(self):
        states,controls=self.trace()
        audit_removed_snapshot(states[:2],controls[:1],0,1,.1)


class DeadlockSubgraphTests(unittest.TestCase):
    def check(self,edges,rejected,active=None):
        def geometry(task,world,paths,states,evidence,cfg):
            if any(tuple(edge) in rejected for edge in evidence['obstruction_edges']):
                raise ValueError('Unsupported edge')
            return dict(passed=True)
        with patch('multi_agent.warehouse_disappear_report.audit_deadlock_geometry',side_effect=geometry):
            return audit_deadlock_subgraph(dict(robot_count=3),{},[],np.zeros((1,3,5)),
                dict(obstruction_edges=edges),None,
                np.ones((1,3),dtype=bool) if active is None else active,'Initial edge rejection')

    def test_redundant_false_edge_is_excluded_but_all_agents_remain_blocked(self):
        result=self.check([[0,1],[1,0],[2,0],[2,1]],{(2,1)})
        self.assertEqual(result['verified_blocked_robot_ids'],[0,1,2])
        self.assertEqual(result['verified_obstruction_edges'],[[0,1],[1,0],[2,0]])
        self.assertEqual(result['rejected_obstruction_edges'][0]['edge'],[2,1])

    def test_essential_false_edge_cannot_certify_fleet_deadlock(self):
        with self.assertRaisesRegex(ValueError,'do not cover robots'):
            self.check([[0,1],[1,0],[2,0]],{(2,0)})

    def test_disappeared_robot_cannot_support_certificate(self):
        with self.assertRaisesRegex(ValueError,'Absent robot'):
            self.check([[0,1],[1,0],[2,0]],set(),np.array([[True,True,False]]))


if __name__=='__main__':unittest.main()

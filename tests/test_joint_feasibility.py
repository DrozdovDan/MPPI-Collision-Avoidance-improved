"""Joint schedule construction and independent replay, including a swap."""

import copy
import json
from pathlib import Path
import sys
import tempfile
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'benchmarks'))
from multi_agent.warehouse_joint_feasibility import joint_certificate,replay_joint_certificate


class JointFeasibilityTests(unittest.TestCase):
    def test_swap_requires_a_holding_move_and_replays_without_contacts(self):
        source=ROOT/'benchmarks/scenarios/warehouse_uniform/tasks/warehouse_s01_v01.json'
        task=json.loads(source.read_text());task['id']='swap';task['robot_count']=2
        task['robots']=copy.deepcopy(task['robots'][:2]);task['source_map']='warehouse'
        starts=[[2.,5.,0.],[8.,5.,np.pi]]
        for i,r in enumerate(task['robots']):
            r['start']=starts[i];r['goal']['pose']=starts[1-i]
        with tempfile.TemporaryDirectory() as temporary:
            folder=Path(temporary);(folder/'tasks').mkdir()
            (folder/'tasks/swap.json').write_text(json.dumps(task))
            (folder/'maps.json').write_text(json.dumps(dict(worlds=dict(warehouse=dict(bounds=[0.,0.,10.,10.],obstacles=[])))))
            output=folder/'certificates'
            certificate=joint_certificate(folder,'swap',output)
            self.assertTrue(certificate['passed'])
            self.assertTrue(any(m['temporary'] for m in certificate['move_sequence']))
            audit=replay_joint_certificate(folder,'swap',output)
            self.assertTrue(audit['passed']);self.assertEqual(audit['checked_robots'],2)
            self.assertGreaterEqual(audit['minimum_inter_robot_clearance_meters'],.04)
            report=output/'swap/validation.json'
            invalid=json.loads(report.read_text());invalid['task_sha256']='wrong'
            report.write_text(json.dumps(invalid))
            with self.assertRaisesRegex(ValueError,'another task'):
                replay_joint_certificate(folder,'swap',output)


if __name__=='__main__':unittest.main()

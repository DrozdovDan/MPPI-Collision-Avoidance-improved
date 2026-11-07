"""Width geometry, probability law, and travel-right versus shared-center anchors."""
from pathlib import Path
import json
import sys
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/'benchmarks'))
from multi_agent.bottleneck_selection import CorridorWidths,proportional_probabilities,draw_width_route,audit_lane_geometry


class BottleneckSelectionTests(unittest.TestCase):
    def test_full_width_includes_both_sides_and_ignores_displaced_lane_clearance(self):
        widths=CorridorWidths(dict(bounds=[0.,-2.,20.,2.],obstacles=[]))
        for y in (0.,1.3,-1.1):
            route=dict(physical_path=[[2.,y],[18.,y]],path=[[2.,1.8],[18.,1.8]])
            witness=widths.route(route)
            self.assertAlmostEqual(witness['width'],4.)
            self.assertAlmostEqual(widths.check_witness(witness),4.)

    def test_thin_bottleneck_between_endpoints_is_found_and_reverse_is_identical(self):
        world=dict(bounds=[0.,-2.,20.,2.],obstacles=[
            [[8.1234,.5],[8.1235,.5],[8.1235,2.],[8.1234,2.]],
            [[8.1234,-2.],[8.1235,-2.],[8.1235,-.5],[8.1234,-.5]]])
        widths=CorridorWidths(world)
        a=widths.route(dict(physical_path=[[2.,0.],[18.,0.]]))
        b=widths.route(dict(physical_path=[[18.,0.],[2.,0.]]))
        self.assertAlmostEqual(a['width'],1.)
        self.assertEqual(a,b)
        self.assertAlmostEqual(widths.check_witness(a),1.)

    def test_width_proportional_distribution_on_an_exact_uniform_grid(self):
        counts=np.zeros(3,dtype=int)
        for index in range(3):
            for j in range(1000):
                selected,p,u=draw_width_route([1.,2.,3.],index,(j+.5)/1000)
                counts[selected]+=1
        np.testing.assert_array_equal(counts,[500,1000,1500])
        np.testing.assert_allclose(p,[1/6,2/6,3/6])

    def test_vertex_hit_survives_independent_geos_roundoff(self):
        worlds=json.loads((ROOT/'benchmarks/scenarios/warehouse_uniform/maps.json').read_text())['worlds']
        widths=CorridorWidths(worlds['warehouse'])
        witness=widths.chord([55.963905000000004,-68.53194675999475],
            [59.259134999999986,-59.5975899999999])
        self.assertAlmostEqual(witness['width'],10.329987003274244)
        self.assertAlmostEqual(widths.check_witness(witness),witness['width'])

    def test_equal_widths_reproduce_original_uniform_choice(self):
        for n in range(1,6):
            for index in range(n):
                for jitter in (0.,.4,np.nextafter(1.,0.)):
                    self.assertEqual(draw_width_route([2.]*n,index,jitter)[0],index)

    def test_invalid_weights_are_rejected_and_large_weights_do_not_overflow(self):
        for bad in ([],[0.],[1.,-1.],[np.nan],[np.inf]):
            with self.assertRaises(ValueError):proportional_probabilities(bad)
        np.testing.assert_allclose(proportional_probabilities([1e308,1e308]),[.5,.5])

    def test_forward_and_reverse_use_their_travel_right_side(self):
        for forward in (True,False):
            reference=[[1.,0.],[9.,0.]] if forward else [[9.,0.],[1.,0.]]
            offset=-.5 if forward else .5
            geometry=[[p[0],offset] for p in reference]
            section=dict(state='SPLIT',forward=forward,mode='RightOneWay' if forward else 'LeftOneWay',
                kind='corridor',geometry=geometry,reference_geometry=reference,
                opposing_geometry=[[p[0],-offset] for p in reference])
            self.assertEqual(audit_lane_geometry(dict(segments=[section]),.3,.04)['split_sections'],1)
            section['geometry']=section['opposing_geometry']
            with self.assertRaisesRegex(ValueError,'travel-left'):audit_lane_geometry(dict(segments=[section]),.3,.04)

    def test_shared_corridor_is_the_reference_center(self):
        section=dict(state='SHARED',kind='corridor',geometry=[[1.,0.],[9.,0.]],reference_geometry=[[1.,0.],[9.,0.]])
        self.assertEqual(audit_lane_geometry(dict(segments=[section]),.3,.04)['shared_corridor_sections'],1)
        section['geometry']=[[1.,.1],[9.,.1]]
        with self.assertRaisesRegex(ValueError,'center reference'):audit_lane_geometry(dict(segments=[section]),.3,.04)


if __name__=='__main__':unittest.main()

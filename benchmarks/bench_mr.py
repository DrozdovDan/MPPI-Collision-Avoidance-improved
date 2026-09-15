"""Import Bench-MR polygon tasks and evaluate the existing MPPI controller."""
import argparse
import ast
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
from shapely.geometry import LineString, MultiPoint, Point, Polygon, box
from shapely.ops import unary_union
from svgpathtools import Line, parse_path

ROOT = Path(__file__).resolve().parents[1]


def load_svg(filename):
    """Bench-MR conventions: group translation, Y flip, convex hull per path."""
    polygons = []

    def visit(element, offset):
        transform = element.get('transform')
        if transform:
            match = re.fullmatch(r'translate\(\s*([^, ]+)[, ]+([^ )]+)\s*\)', transform)
            if not match:
                raise ValueError(f'Unsupported SVG transform: {transform}')
            offset = offset + np.array([float(match[1]), float(match[2])])
        if element.tag.rsplit('}', 1)[-1] == 'path':
            path = parse_path(element.attrib['d'])
            if not path or any(not isinstance(segment, Line) for segment in path):
                raise ValueError('Only polygonal SVG paths are supported')
            if len(path.continuous_subpaths()) != 1:
                raise ValueError('Compound paths are not supported')
            vertices = [path[0].start] + [segment.end for segment in path]
            points = (np.array([[p.real, p.imag] for p in vertices]) + offset) * [1, -1]
            hull = MultiPoint(points).convex_hull
            if not isinstance(hull, Polygon):
                raise ValueError('Degenerate SVG polygon')
            polygons.append(list(map(list, hull.exterior.coords[:-1])))
        for child in element:
            visit(child, offset)

    visit(ET.parse(filename).getroot(), np.zeros(2))
    if not polygons:
        raise ValueError(f'No polygons in {filename}')
    return polygons


def notebook_scenarios(notebook):
    cells = json.loads(notebook.read_text())['cells']
    scenarios = None
    # Extract literal task data without executing the upstream notebook.
    for cell in cells:
        source = ''.join(cell.get('source', []))
        if source.startswith('scenarios ='):
            for node in ast.parse(source).body:
                if isinstance(node, ast.Assign) and any(
                    isinstance(target, ast.Name) and target.id == 'scenarios' for target in node.targets
                ):
                    scenarios = ast.literal_eval(node.value)
    if scenarios is None:
        raise ValueError('No literal scenarios definition in the upstream notebook')
    return scenarios


def import_suite(repo, output):
    notebook = repo / 'python/Benchmark Polygon Worlds.ipynb'
    testing = repo / 'python/Polygon Worlds Testing.ipynb'
    scenarios = [(name, cases, 'benchmark') for name, cases in notebook_scenarios(notebook)]
    scenarios += [(case[0], [case[1:]], 'testing') for case in notebook_scenarios(testing)]
    worlds, tasks, hashes = {}, [], {}
    for name, cases, group in scenarios:
        relative = f'bin/polygon_mazes/{name}.svg'
        polygons = load_svg(repo / relative)
        vertices = np.concatenate(polygons)
        worlds[name] = {'obstacles': polygons, 'bounds': [*vertices.min(axis=0), *vertices.max(axis=0)]}
        hashes[relative] = hashlib.sha256((repo / relative).read_bytes()).hexdigest()
        for index, case in enumerate(cases):
            start, goal = case[:2]
            task_id = f'{name}_{index}' if group == 'benchmark' else f'testing_{name}'
            tasks.append({'id': task_id, 'world': name, 'group': group,
                          'start': [start[k] for k in ('x', 'y', 'theta')],
                          'goal': [goal[k] for k in ('x', 'y', 'theta')],
                          'original_robot': case[2] if len(case) > 2 else 'polygon_mazes/car.svg'})
    hashes[str(notebook.relative_to(repo))] = hashlib.sha256(notebook.read_bytes()).hexdigest()
    hashes[str(testing.relative_to(repo))] = hashlib.sha256(testing.read_bytes()).hexdigest()
    suite = {'source': 'https://github.com/robot-motion/bench-mr',
             'revision': subprocess.check_output(['git', '-C', str(repo), 'rev-parse', 'HEAD'], text=True).strip(),
             'source_sha256': hashes, 'scaling': 1.0, 'worlds': worlds, 'tasks': tasks}
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(suite, indent=2) + '\n')
    print(f'Imported {len(tasks)} tasks from {len(worlds)} worlds into {output}')


def parameters():
    # Differential-drive profile from example/example.ipynb.
    return dict(batch_size=700, time_steps=20, dt=.1, init_value=np.zeros(2),
                control_mean=np.zeros(2), control_cov=np.diag([1., 6.]),
                dist_to_goal_weight=395., terminal_weight=1600., mppi_lambda=.25,
                ma_cost_en=True, ma_dist_weight=750., ma_collision_weight=1e7,
                ma_dist_threashhold=3.5, k_alpha=-2.96, rad_eps=.05,
                velocity_cost_weight=.5, terminal_lookahead_distance=2.,
                ma_en_close_to_goal_dist=1., ma_en_close_to_goal_ts=3,
                orca_time_boundary=1., orca_responsibility_factor=.5,
                ma_exclude_collision_traj=True, velocity_cost_close_to_goal_dist=.4,
                car_like_dyn=False)


def boundary_obstacles(bounds):
    x0, y0, x1, y1 = bounds
    width = max(x1-x0, y1-y0) + 1
    return [list(box(*b).exterior.coords[:-1]) for b in
            [(x0-width, y0-width, x0, y1+width), (x1, y0-width, x1+width, y1+width),
             (x0, y0-width, x1, y0), (x0, y1, x1, y1+width)]]


def evaluate(task, world, args, seed):
    sys.path.insert(0, str(ROOT / 'build/release'))
    import mppi_ca_cpp as mppi
    controller = mppi.Controller()
    max_speed = getattr(args, 'max_speed', 1.0)
    min_speed = getattr(args, 'min_speed', -0.01)
    max_angular_speed = getattr(args, 'max_angular_speed', 2.0)
    agent = dict(size=args.radius, r_vis=30., v_max=max_speed, v_min=min_speed,
                 w_max=max_angular_speed, w_min=-max_angular_speed)
    controller.set_agent_params(mppi.convert_agent_params(agent))
    alg_params = parameters()
    controller.set_alg_params(mppi.convert_alg_params(alg_params))
    # Bench-MR bounds constrain the reference point, not the full footprint.
    # Move artificial walls outward to compensate for the controller's radius.
    bounds = np.array(world['bounds']) + np.array([-1, -1, 1, 1]) * (args.radius + .05)
    controller.set_obstacles(world['obstacles'] + boundary_obstacles(bounds))
    mppi.set_random_seed(seed)
    obstacles = unary_union([Polygon(p) for p in world['obstacles']])
    state = np.array(task['start'], dtype=float)
    goal = np.array(task['goal'], dtype=float)
    original_bounds = box(*world['bounds'])
    states, controls, timings = [state.copy()], [], []
    status = 'step_limit'
    start_time = time.perf_counter()
    invalid_reasons = []
    for name, pose in [('start', state), ('goal', goal)]:
        if obstacles.distance(Point(pose[:2])) <= args.radius:
            invalid_reasons.append(f'{name}_collision')
        if not original_bounds.covers(Point(pose[:2])):
            invalid_reasons.append(f'{name}_out_of_bounds')
    if invalid_reasons:
        status = 'invalid_start_or_goal'
    else:
        for _ in range(args.max_steps):
            if np.linalg.norm(state[:2]-goal[:2]) <= args.goal_tolerance:
                status = 'success'
                break
            if time.perf_counter()-start_time >= args.timeout:
                status = 'time_limit'
                break
            tick = time.perf_counter()
            control, _ = controller.next_step(np.r_[state, 0., 0.], np.empty((0, 4)), np.empty(0), goal)
            timings.append(time.perf_counter()-tick)
            if not np.isfinite(control).all():
                status = 'nonfinite_control'
                break
            dt = alg_params['dt']
            next_state = state + dt * np.array([control[0]*math.cos(state[2]),
                                                control[0]*math.sin(state[2]), control[1]])
            segment = LineString([state[:2], next_state[:2]])
            controls.append(control.tolist())
            states.append(next_state.copy())
            state = next_state
            if segment.distance(obstacles) <= args.radius:
                status = 'collision'
                break
            if not original_bounds.covers(segment):
                status = 'out_of_bounds'
                break
        else:
            if np.linalg.norm(state[:2]-goal[:2]) <= args.goal_tolerance:
                status = 'success'
    elapsed = time.perf_counter()-start_time
    states = np.asarray(states)
    path = LineString(states[:, :2]) if len(states) > 1 else Point(state[:2])
    heading_error = abs(math.atan2(math.sin(state[2]-goal[2]), math.cos(state[2]-goal[2])))
    return dict(task=task['id'], world=task['world'], group=task['group'],
                start=task['start'], goal=task['goal'], seed=seed, status=status,
                invalid_reasons=','.join(invalid_reasons),
                success=status == 'success', pose_success=status == 'success' and heading_error <= args.heading_tolerance,
                steps=len(controls), simulation_seconds=len(controls)*alg_params['dt'], wall_seconds=elapsed,
                planning_seconds=sum(timings), mean_step_ms=1000*np.mean(timings) if timings else 0.,
                path_length=path.length, min_clearance=path.distance(obstacles)-args.radius,
                goal_distance=float(np.linalg.norm(state[:2]-goal[:2])), heading_error=heading_error,
                states=states.tolist(), controls=controls)


def save_results(suite, results, args):
    args.output.mkdir(parents=True, exist_ok=True)
    metadata = dict(source_revision=suite['revision'], radius=args.radius,
                    max_steps=args.max_steps, timeout=args.timeout, goal_tolerance=args.goal_tolerance,
                    heading_tolerance=args.heading_tolerance, profile=parameters(),
                    agent=dict(size=args.radius, r_vis=30., v_max=getattr(args, 'max_speed', 1.0),
                               v_min=getattr(args, 'min_speed', -0.01),
                               w_max=getattr(args, 'max_angular_speed', 2.0),
                               w_min=-getattr(args, 'max_angular_speed', 2.0)),
                    max_speed=getattr(args, 'max_speed', 1.0),
                    min_speed=getattr(args, 'min_speed', -0.01),
                    max_angular_speed=getattr(args, 'max_angular_speed', 2.0),
                    seed=args.seed, repeats=args.repeats, numpy_version=np.__version__,
                    controller_revision=subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip(),
                    controller_sha256=hashlib.sha256((ROOT/'build/release/libcontroller_library.so').read_bytes()).hexdigest())
    (args.output/'results.json').write_text(json.dumps({'metadata': metadata, 'runs': results},
        indent=2, default=lambda v: v.tolist()) + '\n')
    keys = [k for k in results[0] if k not in ('states', 'controls')]
    with (args.output/'summary.csv').open('w') as stream:
        writer = csv.DictWriter(stream, keys, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(results)


def tasks_for_results(suite, results):
    tasks_by_id = {task['id']: task for task in suite['tasks']}
    for run in results:
        if run['task'] not in tasks_by_id:
            tasks_by_id[run['task']] = {
                'id': run['task'], 'world': run['world'], 'group': run['group'],
                'start': run['start'], 'goal': run['goal'], 'original_robot': 'custom',
            }
    result_ids = {run['task'] for run in results}
    return [task for task in tasks_by_id.values() if task['id'] in result_ids]


def plot_results(suite, results, output):
    os.environ.setdefault('MPLCONFIGDIR', '/tmp/mppi-matplotlib')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Polygon as Patch
    tasks = tasks_for_results(suite, results)
    if not tasks:
        raise ValueError('No completed runs to plot')
    rows = math.ceil(len(tasks)/5)
    fig, axes = plt.subplots(rows, 5, figsize=(20, rows*3.5), constrained_layout=True, squeeze=False)
    for ax, task in zip(axes.flat, tasks):
        for polygon in suite['worlds'][task['world']]['obstacles']:
            ax.add_patch(Patch(polygon, facecolor='0.75', edgecolor='0.4', linewidth=.4))
        runs = [r for r in results if r['task'] == task['id']]
        for run in runs:
            states = np.array(run['states'])
            ax.plot(states[:, 0], states[:, 1], color='#17855c' if run['success'] else '#c64040', linewidth=1.5)
        ax.plot(*task['start'][:2], 'o', color='#176ac6', markersize=4)
        ax.plot(*task['goal'][:2], '*', color='#111111', markersize=8)
        statuses = {r['status'] for r in runs}
        if statuses == {'invalid_start_or_goal'}:
            label = 'invalid start/goal'
        elif statuses == {'time_limit'}:
            label = f'{len(runs)} timeouts'
        else:
            label = f"{sum(r['success'] for r in runs)}/{len(runs)} reached"
        ax.set_title(f"{task['id']}\n{label}", fontsize=10)
        ax.set_aspect('equal')
        ax.autoscale_view()
        ax.tick_params(labelsize=7)
    for ax in list(axes.flat)[len(tasks):]:
        ax.set_visible(False)
    fig.savefig(output/'trajectories.png', dpi=150)
    plt.close(fig)


def write_report(suite, results, args):
    attempted = [r for r in results if r['status'] != 'invalid_start_or_goal']
    invalid = {r['task'] for r in results if r['status'] == 'invalid_start_or_goal'}
    lines = ['# Current MPPI on Bench-MR polygon tasks', '',
             f"Source revision: `{suite['revision']}`.", '',
             f"Evaluated {len({r['task'] for r in results})} tasks with {args.repeats} repeat(s) each. "
             f"{len(invalid)} tasks were invalid before planning. "
             f"Of {len(attempted)} attempted runs, {sum(r['success'] for r in attempted)} reached the XY goal; "
             f"{sum(r['pose_success'] for r in attempted)} also met the diagnostic heading tolerance.", '',
             f"Collision runs: {sum(r['status'] == 'collision' for r in attempted)}. "
             f"Out-of-bounds runs: {sum(r['status'] == 'out_of_bounds' for r in attempted)}.", '',
             f"Limits: {args.max_steps} steps and {args.timeout:g} wall seconds per run. "
             f"Robot radius: {args.radius:g}; XY tolerance: {args.goal_tolerance:g}; "
             f"diagnostic heading tolerance: {args.heading_tolerance:g} rad.", '',
             f"Control limits: linear [{getattr(args, 'min_speed', -0.01):g}, "
             f"{getattr(args, 'max_speed', 1.0):g}] units/s; angular +/-"
             f"{getattr(args, 'max_angular_speed', 2.0):g} rad/s.", '',
             '| Task | Reached / attempted | Statuses | Mean final goal distance |',
             '| --- | ---: | --- | ---: |']
    for task in tasks_for_results(suite, results):
        runs = [r for r in results if r['task'] == task['id']]
        if not runs:
            continue
        attempted_runs = [r for r in runs if r['status'] != 'invalid_start_or_goal']
        statuses = ', '.join(sorted({r['status'] for r in runs}))
        distance = f"{np.mean([r['goal_distance'] for r in runs]):.3f}"
        lines.append(f"| {task['id']} | {sum(r['success'] for r in runs)} / {len(attempted_runs)} | {statuses} | {distance} |")
    lines += ['', 'The original 20-task notebook includes start/goal coordinates outside the current '
              'SVG bounds or inside obstacles. Invalid tasks are input-validation results, not MPPI failures. '
              'See `summary.csv` for exact reasons.', '',
              'These are adapted single-robot tests using the existing circular differential-drive model. '
              'They are not native Bench-MR vehicle-planning scores. MPPI optimizes position, not final heading. '
              'No global planner or obstacle-aware ORCA was added. Timeouts do not establish unsolvability.', '',
              'See [trajectory overview](trajectories.png), [metrics](summary.csv), and '
              '[full trajectories and configuration](results.json).', '']
    (args.output/'REPORT.md').write_text('\n'.join(lines))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    importer = sub.add_parser('import')
    importer.add_argument('repo', type=Path)
    importer.add_argument('--output', type=Path, default=ROOT/'benchmarks/bench_mr_tasks.json')
    reporter = sub.add_parser('report')
    reporter.add_argument('--suite', type=Path, default=ROOT/'benchmarks/bench_mr_tasks.json')
    reporter.add_argument('--output', type=Path, default=ROOT/'benchmarks/results/current')
    runner = sub.add_parser('run')
    runner.add_argument('--suite', type=Path, default=ROOT/'benchmarks/bench_mr_tasks.json')
    runner.add_argument('--output', type=Path, default=ROOT/'benchmarks/results/current')
    runner.add_argument('--repeats', type=int, default=1)
    runner.add_argument('--seed', type=int, default=42)
    runner.add_argument('--max-steps', type=int, default=2000)
    runner.add_argument('--timeout', type=float, default=30.)
    runner.add_argument('--radius', type=float, default=.3)
    runner.add_argument('--max-speed', type=float, default=1.)
    runner.add_argument('--min-speed', type=float, default=-.01)
    runner.add_argument('--max-angular-speed', type=float, default=2.)
    runner.add_argument('--goal-tolerance', type=float, default=.3)
    runner.add_argument('--heading-tolerance', type=float, default=.15)
    runner.add_argument('--task', help='Run a single task ID')
    args = parser.parse_args()
    if args.command == 'import':
        import_suite(args.repo, args.output)
        return
    if args.command == 'report':
        suite = json.loads(args.suite.read_text())
        saved = json.loads((args.output/'results.json').read_text())
        results = saved['runs']
        for key, value in saved['metadata'].items():
            setattr(args, key, value)
        args.repeats = len({r['seed'] for r in results})
        plot_results(suite, results, args.output)
        write_report(suite, results, args)
        return
    if min(args.repeats, args.max_steps, args.timeout, args.radius, args.max_speed,
           args.max_angular_speed, args.goal_tolerance, args.heading_tolerance) <= 0:
        parser.error('Run limits, tolerances, and radius must be positive')
    if args.min_speed > args.max_speed:
        parser.error('--min-speed cannot exceed --max-speed')
    suite = json.loads(args.suite.read_text())
    tasks = [t for t in suite['tasks'] if args.task is None or t['id'] == args.task]
    if not tasks:
        parser.error('Unknown task ID')
    results = []
    for task in tasks:
        for repeat in range(args.repeats):
            result = evaluate(task, suite['worlds'][task['world']], args, args.seed+repeat)
            results.append(result)
            save_results(suite, results, args)
            print(f"{result['task']} seed={result['seed']}: {result['status']}, "
                  f"distance={result['goal_distance']:.2f}, steps={result['steps']}", flush=True)
    plot_results(suite, results, args.output)
    write_report(suite, results, args)
    attempted = [r for r in results if r['status'] != 'invalid_start_or_goal']
    print(f"Reached: {sum(r['success'] for r in attempted)}/{len(attempted)} attempted runs; "
          f"invalid: {len(results)-len(attempted)}; saved to {args.output}")


if __name__ == '__main__':
    main()

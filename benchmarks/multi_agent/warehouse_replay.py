"""Full, synchronized GIF replay of immutable audited warehouse recordings.

Every saved 0.1 s pose is rendered. GIF frames are streamed to disk so a long,
large comparison does not require retaining all uncompressed images in memory.
No planner, controller, integrator, or terminal detector is rerun or modified.
"""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
from PIL import GifImagePlugin, Image, ImageChops

from .geometry import ROOT
from .warehouse_evaluation import load_task, save_json

METHODS = ('rrt', 'k_routes', 'k_routes_width')
LABELS = ('RRT-Connect', 'K routes: uniform', 'K routes: bottleneck width')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class StreamingGif:
    """Fixed-palette, delta-frame GIF; memory use is bounded by two images."""
    def __init__(self, filename, palette):
        self.stream = Path(filename).open('wb')
        self.palette = palette
        self.previous = None
        self.frames = 0

    def append(self, rgb, duration):
        frame = rgb.quantize(palette=self.palette, dither=Image.Dither.NONE)
        pixels = np.array(frame)
        # Palette slot 255 duplicates slot 0 and is reserved for transparency.
        pixels[pixels == 255] = 0
        frame = Image.fromarray(pixels)
        frame.putpalette(self.palette.getpalette())
        if self.previous is None:
            header, _ = GifImagePlugin.getheader(frame, info={'loop': 0, 'optimize': False})
            for block in header:
                self.stream.write(block)
            bounds = (0, 0, frame.width, frame.height)
        else:
            bounds = ImageChops.difference(frame, self.previous).getbbox()
            if bounds is None:
                bounds = (0, 0, 1, 1)
        patch = frame.crop(bounds)
        if self.previous is not None:
            data = np.array(patch)
            unchanged = data == np.array(self.previous.crop(bounds))
            data[unchanged] = 255
            patch = Image.fromarray(data)
            patch.putpalette(self.palette.getpalette())
        for block in GifImagePlugin.getdata(patch, offset=bounds[:2], duration=duration,
                                            disposal=1, transparency=255, include_color_table=False):
            self.stream.write(block)
        self.previous = frame
        self.frames += 1

    def close(self):
        self.stream.write(b';')
        self.stream.close()


def _palette(colors):
    entries = [[255, 255, 255], [0, 0, 0]]
    for color in colors:
        for opacity in (1., .9, .45, .25):
            entries.append(np.rint(255*(opacity*np.array(color)+(1-opacity))).astype(int).tolist())
    entries += [[v, v, v] for v in np.linspace(0, 255, 96).astype(int)]
    entries += [[40, 140, 65], [180, 45, 45], [240, 240, 240]]
    entries = (entries+[[255, 255, 255]]*256)[:256]
    entries[255] = entries[0]
    image = Image.new('P', (1, 1))
    image.putpalette(np.array(entries, dtype=np.uint8).reshape(-1).tolist())
    return image


def load_recording(root, task_id, method):
    directory = Path(root)/f'{task_id}__{method}__p42__c42'
    audit = json.loads((Path(root)/'audit.json').read_text())
    if not audit['passed']:
        raise ValueError('Replay requires passed independent audits')
    record = next(r for r in audit['traces'] if r['run_id'] == directory.name)
    for name, key in [('run.json', 'run_sha256'), ('trajectory.npz', 'trajectory_sha256')]:
        if sha(directory/name) != record[key]:
            raise ValueError(f'Recording differs from audited bytes: {directory/name}')
    run = json.loads((directory/'run.json').read_text())
    if sha(directory/'paths.json') != run['anchor_validation']['paths_sha256']:
        raise ValueError('Selected anchors differ from the audited recording')
    with np.load(directory/'trajectory.npz', allow_pickle=False) as data:
        states = data['states'].copy()
    paths = json.loads((directory/'paths.json').read_text())
    if states.shape[1:] != (32, 5) or len(paths) != 32:
        raise ValueError('Incomplete 32-agent recording')
    return dict(directory=directory, run=run, states=states, paths=paths,
                audited_run_sha256=record['run_sha256'],
                audited_trajectory_sha256=record['trajectory_sha256'])


def render(task_id, output, *, suite=ROOT/'benchmarks/scenarios/warehouse_uniform',
           baseline=ROOT/'benchmarks/results/warehouse_uniform_corrected',
           weighted=ROOT/'benchmarks/results/warehouse_bottleneck_width'):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.colors import hsv_to_rgb
    from matplotlib.lines import Line2D
    from matplotlib.patches import Circle
    from corridor_graph.traffic_visualization import _map

    output = Path(output); output.mkdir(parents=True, exist_ok=True)
    task, world, task_hash = load_task(suite, task_id)
    recordings = [load_recording(baseline if m != 'k_routes_width' else weighted, task_id, m)
                  for m in METHODS]
    if not recordings[0]['run']['success'] or any(r['run']['success'] for r in recordings[1:]):
        raise ValueError('Requested example must have RRT success and both K policies failing')
    for item in recordings:
        run = item['run']
        if run['scenario_sha256'] != task_hash or run['controller_profiles'] != recordings[0]['run']['controller_profiles']:
            raise ValueError('Task assignments or controller settings differ')
        np.testing.assert_array_equal(item['states'][0], recordings[0]['states'][0])
        if run['status'] not in ('success', 'deadlock'):
            raise ValueError('This replay selects directly recorded success/deadlock outcomes')
    colors = [hsv_to_rgb([(i*.618033988749895) % 1, .72, .78]) for i in range(32)]
    blocked = np.concatenate([r['states'][-1, r['run']['termination_evidence']['blocked_robot_ids'], :2]
                              for r in recordings[1:]])
    minimum = blocked.min(axis=0)-[3., 3.]
    maximum = blocked.max(axis=0)+[3., 3.]
    center = (minimum+maximum)/2
    span = np.maximum(maximum-minimum, [12., 10.])
    zoom = (center-span/2, center+span/2)
    fig, axes = plt.subplots(2, 3, figsize=(24, 14), dpi=100,
                             gridspec_kw={'height_ratios': [2.2, 1.]})
    fig.subplots_adjust(left=.035, right=.99, bottom=.10, top=.885, wspace=.13, hspace=.13)
    fig.suptitle(f'{task_id} | {task["traffic_pattern"]} | 32 agents, diameter '
                 f'{task["physical_scale"]["diameter_meters"]:.1f} m | same MPPI/ORCA controller',
                 y=.982, fontsize=17)
    clock = fig.text(.5, .943, '', ha='center', fontsize=13, animated=True)
    dynamic = [clock]
    panels = []
    goals = np.array([r['goal']['pose'][:2] for r in task['robots']])
    for col, (item, label) in enumerate(zip(recordings, LABELS)):
        panel = dict(item=item, title=axes[0, col].set_title('', fontsize=12), artists=[])
        panel['title'].set_animated(True); dynamic.append(panel['title'])
        for row in range(2):
            ax = axes[row, col]; _map(ax, world)
            ax.set(xlabel='x (m)', ylabel='y (m)')
            if row:
                ax.set(xlim=(zoom[0][0], zoom[1][0]), ylim=(zoom[0][1], zoom[1][1]),
                       title='Same close-up region in every method')
            ax.scatter(goals[:, 0], goals[:, 1], marker='*', s=25 if row else 10,
                       c=colors, alpha=.45, linewidths=.3, zorder=3)
            for i, (robot, path) in enumerate(zip(task['robots'], item['paths'])):
                path = np.asarray(path)
                ax.plot(path[:, 0], path[:, 1], '--', color=colors[i], lw=.8 if row else .65, alpha=.45)
                trail, = ax.plot([], [], color=colors[i], lw=1.7 if row else 1.1,
                                 alpha=.9, zorder=4, animated=True)
                body = Circle(item['states'][0, i, :2], robot['radius'], facecolor=colors[i],
                              edgecolor='black', linewidth=.8, zorder=5, animated=True)
                ax.add_patch(body)
                heading, = ax.plot([], [], color='black', lw=1., zorder=6, animated=True)
                text = ax.text(0, 0, f'{i:02d}', ha='center', va='bottom', fontsize=8 if row else 6.5,
                               color='black', weight='bold', clip_on=True,
                               bbox=dict(facecolor='white', edgecolor='none', alpha=.8, pad=.15),
                               zorder=7, animated=True)
                panel['artists'].append((i, robot, trail, body, heading, text, bool(row)))
                dynamic.extend((trail, body, heading, text))
        panels.append(panel)
    fig.legend([Line2D([], [], color='black', ls='--', alpha=.45),
                Line2D([], [], color='black'), Line2D([], [], color='black', marker='*', ls='')],
               ['Selected planned path (all 32)', 'Executed trail (all 32)', 'Goal'],
               loc='lower left', bbox_to_anchor=(.035, .052), ncol=3, fontsize=10, frameon=False)
    fig.text(.99, .065, 'Every recorded 0.1 s step | 1x playback | terminal panels hold their last recorded pose',
             ha='right', fontsize=10)
    for i, color in enumerate(colors):
        fig.text(.05+i*.029, .035, f'{i:02d}', color=color, weight='bold', fontsize=11, ha='center')
    fig.text(.5, .012, 'Agent IDs and colors match across methods. No resimulation, interpolated poses, or skipped recording steps.',
             ha='center', fontsize=10)
    fig.canvas.draw()
    background = fig.canvas.copy_from_bbox(fig.bbox)
    dynamic.sort(key=lambda artist: artist.get_zorder())
    dt = task['dt']; frame_count = max(len(r['states']) for r in recordings)
    gif = output/f'{task_id}__rrt_vs_both_k_routes__full.gif'
    writer = StreamingGif(gif, _palette(colors))
    snapshots = {0: 'start.png', frame_count-1: 'final.png', len(recordings[0]['states'])-1: 'rrt_finish.png'}
    try:
        for step in range(frame_count):
            clock.set_text(f'Simulation time: {step*dt:.1f} / {(frame_count-1)*dt:.1f} s')
            for panel, label in zip(panels, LABELS):
                item = panel['item']; j = min(step, len(item['states'])-1)
                poses = item['states'][j]; t = j*dt; run = item['run']
                done = sum(a is not None and a <= t+1e-9 for a in run['individual_arrival_seconds'])
                ended = j == len(item['states'])-1
                state = run['status'].upper()+' (recording ended)' if ended else 'RUNNING'
                panel['title'].set_text(f'{label}\nt={t:.1f} s | {done}/32 finished | {state}')
                panel['title'].set_color('#288c41' if ended and run['success'] else '#b42d2d' if ended else 'black')
                for i, robot, trail, body, heading, text, zoomed in panel['artists']:
                    p = poses[i]; radius = robot['radius']
                    trail.set_data(item['states'][:j+1, i, 0], item['states'][:j+1, i, 1])
                    body.center = p[:2]
                    heading.set_data([p[0], p[0]+.9*radius*np.cos(p[2])],
                                     [p[1], p[1]+.9*radius*np.sin(p[2])])
                    text.set_position((p[0], p[1]+radius+(0.12 if zoomed else .25)))
            fig.canvas.restore_region(background)
            for artist in dynamic:
                fig.draw_artist(artist)
            rgb = Image.fromarray(np.asarray(fig.canvas.buffer_rgba())).convert('RGB')
            writer.append(rgb, 2000 if step == 0 else 5000 if step == frame_count-1 else round(1000*dt))
            if step in snapshots:
                rgb.save(output/snapshots[step])
            if step % 100 == 0 or step == frame_count-1:
                print(f'Rendered {step+1}/{frame_count} full recorded steps', flush=True)
    finally:
        writer.close(); plt.close(fig)
    selection = ('Shortest full recording among tasks where RRT succeeds and both K policies fail.'
                 if task_id == 'warehouse_s13_v02' else 'Explicitly requested task ID.')
    summary = dict(task_id=task_id, scenario_sha256=task_hash, selected_example_rule=selection,
        robot_count=32, diameter_meters=task['physical_scale']['diameter_meters'], dt=dt,
        rendered_recording_steps=frame_count, skipped_recording_steps=0, interpolated_poses=False,
        source_states_unchanged=True, playback_speed=1., initial_hold_seconds=2., final_hold_seconds=5.,
        gif=gif.name, gif_sha256=sha(gif), renderer_sha256=sha(__file__),
        methods=[dict(method=m, status=r['run']['status'], finished_agents=sum(a is not None for a in r['run']['individual_arrival_seconds']),
                      recorded_states=len(r['states']), recorded_until_seconds=r['run']['simulated_seconds'],
                      makespan_seconds=r['run']['makespan_seconds'],
                      source_directory=str(r['directory'].relative_to(ROOT)),
                      run_sha256=r['audited_run_sha256'], trajectory_sha256=r['audited_trajectory_sha256'])
                 for m, r in zip(METHODS, recordings)])
    save_json(output/'summary.json', summary)
    lines = [f'# Full recorded comparison: {task_id}', '',
             f'Same frozen starts, goals, 32 robots (diameter {task["physical_scale"]["diameter_meters"]:.1f} m), physics, and normal MPPI/ORCA settings.', '',
             f'[Open full GIF]({gif.name})', '',
             '| Method | Finished agents | Outcome | Recording ends | Makespan |',
             '| --- | ---: | --- | ---: | ---: |']
    for label, r in zip(LABELS, summary['methods']):
        makespan = 'undefined' if r['makespan_seconds'] is None else f'{r["makespan_seconds"]:.1f} s'
        lines.append(f'| {label} | {r["finished_agents"]}/32 | {r["status"]} | {r["recorded_until_seconds"]:.1f} s | {makespan} |')
    lines += ['', 'The GIF shows the entire map above and the same blocking region below. Every selected anchor is dashed; every executed trail is solid. Filled circles are actual-size robot footprints with IDs; stars are goals. Agent IDs/colors match across methods.', '',
              f'All {frame_count} recorded time steps from 0 to {(frame_count-1)*dt:.1f} s appear at 10 fps (1x playback), with a 2 s initial hold and 5 s final hold. A method that has terminated holds its last saved pose while the other recordings continue. Actual trace samples are used directly; there is no resimulation or pose interpolation.', '',
              selection+' It is one example from the existing measurements.', '',
              '[Final frame](final.png) · [Start frame](start.png) · [Audited source hashes and metadata](summary.json)', '',
              'Reproduce from the project root:', '', '```bash',
              f'OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONPATH=benchmarks .venv/bin/python -m multi_agent.warehouse_replay --task {task_id}',
              '```', '']
    (output/'README.md').write_text('\n'.join(lines))
    return gif


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--task', default='warehouse_s13_v02')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    destination = args.output or ROOT/'benchmarks/scenarios/warehouse_uniform/width_evaluation/full_replays'/args.task
    print(render(args.task, destination), flush=True)

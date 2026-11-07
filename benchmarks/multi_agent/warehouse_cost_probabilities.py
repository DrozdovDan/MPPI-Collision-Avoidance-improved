"""Compare edge-cost and bottleneck probabilities on saved candidates only.

No planner, controller, simulator, or execution-result module is imported.
Run from the project root with PYTHONPATH=benchmarks and --help for options.
"""

import argparse
from collections import defaultdict
import csv
import gzip
import hashlib
import json
from pathlib import Path

import numpy as np

from . import edge_cost_selection
from .bottleneck_selection import proportional_probabilities
from .edge_cost_selection import edge_cost_distribution, graph_route_edges

ROOT = Path(__file__).resolve().parents[2]


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')


def write_csv(path, rows):
    with Path(path).open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        for row in rows:
            writer.writerow({k: json.dumps(v, separators=(',', ':')) if isinstance(v, (list, dict)) else v
                             for k, v in row.items()})


def entropy(p):
    p = np.asarray(p); positive = p[p > 0]
    return float(-np.dot(positive, np.log(positive)))


def summarize(rows):
    """Give every robot query equal weight, rather than every candidate path."""
    multi = [r for r in rows if r['k'] > 1]
    metrics = ('total_variation', 'bottleneck_top_probability', 'cost_top_probability',
               'bottleneck_entropy', 'cost_entropy', 'bottleneck_expected_graph_length',
               'cost_expected_graph_length', 'bottleneck_expected_physical_length',
               'cost_expected_physical_length', 'bottleneck_expected_bottleneck',
               'cost_expected_bottleneck')
    def cohort(items):
        return dict(queries=len(items),
                    **{f'mean_{key}': float(np.mean([r[key] for r in items])) if items else None
                       for key in metrics},
                    modal_route_changed=sum(r['modal_route_changed'] for r in items),
                    cost_top_over_99_percent=sum(r['cost_top_probability'] > .99 for r in items),
                    queries_with_zero_edge_candidate=sum(r['zero_edge_routes'] > 0 for r in items),
                    all_candidates_zero_edges=sum(r['zero_edge_routes'] == r['k'] for r in items))
    return dict(all_queries=cohort(rows), multiple_candidates=cohort(multi),
                candidate_routes=sum(r['k'] for r in rows),
                zero_edge_candidate_routes=sum(r['zero_edge_routes'] for r in rows))


def compare(suite, baseline, weighted, output, *, lambda_weight, tau, task_ids=None):
    suite, baseline, weighted, output = map(Path, (suite, baseline, weighted, output))
    manifest = json.loads((suite / 'manifest.json').read_text())
    preflight = json.loads((weighted / 'selection_preflight.json').read_text())
    if sha(suite / 'manifest.json') != preflight['baseline_contract']['frozen_manifest_sha256']:
        raise ValueError('Frozen manifest differs from the bottleneck comparison')
    approved = {item['task_id']: item['sha256'] for m in preflight['maps'] for item in m['selection_inputs']}
    entries = manifest['scenarios']
    if task_ids:
        unknown = set(task_ids) - {e['id'] for e in entries}
        if unknown:
            raise ValueError(f'Unknown task IDs: {sorted(unknown)}')
        entries = [e for e in entries if e['id'] in task_ids]
    graph_edges = {}
    graph_hashes = {}
    for name in sorted({e['source_map'] for e in entries}):
        filename = suite / f'geometry_cache/{name}.resources.json.gz'
        graph = json.loads(gzip.decompress(filename.read_bytes()))
        graph_edges[name] = {e['id']: e for e in graph['routing_geometry']['edges']}
        graph_hashes[name] = sha(filename)
    sources, queries, probabilities, examples = [], [], [], []
    for ordinal, item in enumerate(entries, 1):
        task_id = item['id']; name = item['source_map']
        task = json.loads((suite / item['file']).read_text())
        if fingerprint({k: v for k, v in task.items() if k != 'generation_seconds'}) != item['scenario_sha256']:
            raise ValueError(f'Task changed: {task_id}')
        prepared_path = weighted / f'selection_inputs/{task_id}.json'
        if sha(prepared_path) != approved[task_id]:
            raise ValueError(f'Audited bottleneck inputs changed: {task_id}')
        prepared = json.loads(prepared_path.read_text())
        native_path = baseline / f'{task_id}__k_routes__p42__c42/agents.routes.json'
        native_bytes = native_path.read_bytes()
        native_sha = hashlib.sha256(native_bytes).hexdigest()
        if native_sha != prepared['native_candidates_sha256']:
            raise ValueError(f'Native candidates changed: {task_id}')
        native = json.loads(native_bytes)
        if len(native['queries']) != len(prepared['robots']) or len(native['queries']) != len(task['robots']):
            raise ValueError('Robot count differs')
        sources.append(dict(task_id=task_id, native_candidates_sha256=native_sha,
                            bottleneck_input_sha256=approved[task_id], scenario_sha256=item['scenario_sha256']))
        for agent_index, (robot, query, old) in enumerate(zip(task['robots'], native['queries'], prepared['robots'])):
            if (robot['id'] != query['name'] or robot['id'] != old['id'] or
                    robot['radius'] != query['radius'] or not query['validation_passed'] or
                    len(query['routes']) != len(old['routes'])):
                raise ValueError('Robot/candidate mapping differs')
            edges = [graph_route_edges(r, graph_edges[name]) for r in query['routes']]
            result = edge_cost_distribution(edges, robot['radius'], lambda_weight=lambda_weight, tau=tau)
            widths = np.array([r['bottleneck_width'] for r in old['routes']])
            p_old = proportional_probabilities(widths)
            np.testing.assert_allclose(p_old, old['probabilities'], rtol=1e-14, atol=1e-15)
            p_new = np.array(result['probabilities'])
            physical_lengths = np.array([r['physical_metrics']['length'] for r in query['routes']])
            graph_lengths = np.array([r['length'] for r in result['routes']])
            common = dict(task_id=task_id, source_map=name, scenario_id=item['scenario_id'],
                          variant_index=item['variant_index'], diameter_meters=2 * robot['radius'],
                          traffic_pattern=item['traffic_pattern'], agent_index=agent_index, agent_id=robot['id'])
            row = dict(common, k=len(edges), w_max=result['w_max'],
                       bottleneck_probabilities=p_old.tolist(), cost_probabilities=p_new.tolist(),
                       total_variation=float(.5 * np.abs(p_new - p_old).sum()),
                       modal_route_changed=bool(np.argmax(p_new) != np.argmax(p_old)),
                       bottleneck_top_probability=float(p_old.max()), cost_top_probability=float(p_new.max()),
                       bottleneck_entropy=entropy(p_old), cost_entropy=entropy(p_new),
                       bottleneck_expected_graph_length=float(p_old @ graph_lengths),
                       cost_expected_graph_length=float(p_new @ graph_lengths),
                       bottleneck_expected_physical_length=float(p_old @ physical_lengths),
                       cost_expected_physical_length=float(p_new @ physical_lengths),
                       bottleneck_expected_bottleneck=float(p_old @ widths),
                       cost_expected_bottleneck=float(p_new @ widths),
                       zero_edge_routes=sum(not e for e in edges))
            queries.append(row)
            details = []
            for index, (route, edge_list, record) in enumerate(zip(query['routes'], edges, result['routes'])):
                if route['id'] != old['routes'][index]['route_id'] or fingerprint(route['path']) != old['routes'][index]['path_sha256']:
                    raise ValueError('Candidate route identity differs')
                candidate = dict(common, route_index=index, route_id=route['id'], edge_count=len(edge_list),
                                 edge_ids=route['edge_ids'], edge_lengths=[e['length'] for e in edge_list],
                                 edge_widths=[e['width'] for e in edge_list], q=record['q'],
                                 w_max=result['w_max'], graph_length=record['length'],
                                 physical_length=float(physical_lengths[index]), width_exposure=record['width_exposure'],
                                 cost=record['cost'], bottleneck_width=float(widths[index]),
                                 bottleneck_probability=float(p_old[index]), cost_probability=float(p_new[index]),
                                 probability_change=float(p_new[index] - p_old[index]))
                probabilities.append(candidate); details.append(candidate)
            if task_id == 'warehouse_s13_v02' and agent_index in (1, 2, 11, 20):
                examples.append(dict(query=row, routes=details))
        if ordinal % 25 == 0:
            print(f'Compared {ordinal}/{len(entries)} tasks, {len(queries)} robot candidate sets', flush=True)
    groups = defaultdict(list)
    for row in queries:
        groups[row['source_map']].append(row)
    report = dict(simulations_run=0, planners_run=0, lambda_weight=lambda_weight, tau=tau,
                  tasks=len(entries), robot_queries=len(queries), candidate_routes=len(probabilities),
                  edge_definition='Stored route.edge_ids, preserving order and multiplicity.',
                  length_definition='Full stored routing_geometry edge.length, including partial-edge entries at full edge length. Connectors excluded.',
                  width_definition='Stored routing_geometry edge.min_width, no radius or safety margin subtracted from w_e.',
                  normalization='W_max is the maximum stored edge width in this robot candidate set only.',
                  zero_edge_definition='Empty edge sums give length=exposure=cost=0. For all-empty sets W_max=null and p is uniform; q is never evaluated.',
                  bottleneck_definition='Unchanged audited full-path transverse bottleneck widths, p=w/sum(w).',
                  overall=summarize(queries), by_map={name: summarize(rows) for name, rows in sorted(groups.items())},
                  inputs=dict(manifest_sha256=sha(suite / 'manifest.json'),
                              bottleneck_preflight_sha256=sha(weighted / 'selection_preflight.json'),
                              graph_bundle_sha256=graph_hashes),
                  implementation_sha256={str(p.relative_to(ROOT)): sha(p)
                                         for p in (Path(__file__), Path(edge_cost_selection.__file__))})
    output.mkdir(parents=True, exist_ok=True)
    write_json(output / 'summary.json', report)
    write_json(output / 'inputs.json', sources)
    write_json(output / 'examples.json', examples)
    write_csv(output / 'probabilities.csv', probabilities)
    write_csv(output / 'queries.csv', queries)
    # Retain the requested five-start/goal scenario grouping, without motion metrics.
    scenarios = defaultdict(list)
    for row in queries:
        scenarios[(row['source_map'], row['scenario_id'])].append(row)
    write_json(output / 'by_scenario.json', [dict(source_map=m, scenario_id=s, **summarize(rows))
                                           for (m, s), rows in sorted(scenarios.items())])
    plot(output, queries, examples)
    write_readme(output, report, examples)
    return report


def plot(output, queries, examples):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(2, 2, figsize=(12, 8), constrained_layout=True)
    for ax, agent in zip(axes[0], (11, 20)):
        example = next((e for e in examples if e['query']['agent_index'] == agent), None)
        if example is None:
            ax.set_visible(False); continue
        routes = example['routes']; x = np.arange(len(routes))
        a = ax.bar(x - .18, [r['bottleneck_probability'] for r in routes], .36, label='Bottleneck')
        b = ax.bar(x + .18, [r['cost_probability'] for r in routes], .36, label='Edge cost')
        ax.bar_label(a, fmt='%.3f', fontsize=9); ax.bar_label(b, fmt='%.3f', fontsize=9)
        ax.set_xticks(x, [f'Path {r["route_index"]}\nneck {r["bottleneck_width"]:.2f} m' for r in routes])
        ax.set_ylim(0, 1.14); ax.set_ylabel('Probability'); ax.set_title(f'warehouse_s13_v02, agent {agent}')
        ax.legend()
    for ax, name in zip(axes[1], ('warehouse', 'warehouse2')):
        rows = [r for r in queries if r['source_map'] == name and r['k'] > 1]
        if not rows:
            ax.set_visible(False); continue
        bins = np.linspace(0, 1, 26)
        ax.hist([r['bottleneck_top_probability'] for r in rows], bins, alpha=.6, label='Bottleneck')
        ax.hist([r['cost_top_probability'] for r in rows], bins, alpha=.6, label='Edge cost')
        ax.set_title(f'{name}: {len(rows)} queries with K > 1'); ax.set_xlabel('Largest path probability')
        ax.set_ylabel('Robot candidate sets'); ax.legend()
    fig.suptitle('Saved-path probabilities only; no simulations')
    fig.savefig(output / 'comparison.png', dpi=150)
    fig.savefig(output / 'comparison.svg')
    plt.close(fig)


def write_readme(output, report, examples):
    text = [f'# Edge-cost versus bottleneck probabilities\n',
            f'λ = {report["lambda_weight"]:g}; temperature τ = {report["tau"]:g}. '
            f'{report["tasks"]} frozen tasks, {report["robot_queries"]:,} robot queries and '
            f'{report["candidate_routes"]:,} unchanged candidate paths. No simulations or planners were run.\n',
            '```text\nW_max = max(w_e across all edges in this robot’s K routes)\n'
            'q_e = clamp(1 - (w_e - 2*r)/(2*W_max), 0, 1)\n'
            'L_k = sum(l_e)\nC_k = L_k + lambda * sum(l_e*q_e)\n'
            'p_k = softmax(-C_k/tau)\n```\n',
            'Use stored graph `edge.length` and `edge.min_width`, retaining repeated edges. '
            'As requested, start/goal/direct connectors are excluded, and partial-edge entries '
            'use the full stored edge length. The physical radius r is used without a safety-margin addition. '
            'W_max is recomputed for each robot’s returned candidate set. The stable softmax subtracts '
            'the minimum cost before exponentiation.\n',
            'The comparison policy is unchanged: full-path transverse bottleneck width divided by '
            'the sum of those widths. Its widths need not equal the graph’s stored minimum-width field; '
            'both inputs are included in the CSV.\n',
            '## Probability comparison\n',
            '| Map | Queries with K > 1 | Mean largest p: bottleneck | Mean largest p: edge cost | Mean total variation | Changed modal path |\n',
            '| --- | ---: | ---: | ---: | ---: | ---: |\n']
    for name, data in report['by_map'].items():
        m = data['multiple_candidates']; n = m['queries']
        if n:
            text.append(f'| {name} | {n} | {m["mean_bottleneck_top_probability"]:.4f} | '
                        f'{m["mean_cost_top_probability"]:.4f} | {m["mean_total_variation"]:.4f} | '
                        f'{m["modal_route_changed"]}/{n} ({100*m["modal_route_changed"]/n:.1f}%) |\n')
    text += ['\nAverages give each robot query equal weight. Total variation is '
             '`0.5 * sum(abs(p_cost - p_bottleneck))`. Changed modal path compares the first '
             'maximum index in each vector; it is a probability comparison, not a sampled assignment.\n',
             '## The replay case\n',
             '| Agent | Path index | Bottleneck (m) | Graph L (m) | Width exposure (m) | C | Bottleneck p | Edge-cost p |\n',
             '| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |\n']
    for example in examples:
        for r in example['routes']:
            text.append(f'| {r["agent_index"]} | {r["route_index"]} | {r["bottleneck_width"]:.3f} | '
                        f'{r["graph_length"]:.3f} | {r["width_exposure"]:.3f} | {r["cost"]:.3f} | '
                        f'{r["bottleneck_probability"]:.8g} | {r["cost_probability"]:.8g} |\n')
    all_data = report['overall']
    text += ['\n## Zero-edge candidates\n',
             f'{all_data["zero_edge_candidate_routes"]} candidates have no stored graph edges. '
             'Their graph-only L and C are exactly zero despite a nonzero physical path length. '
             'They remain in the candidate sets and receive the probability implied by the formula; '
             'no length floor or connector penalty is added. If every candidate has no graph edges, '
             'all costs are zero and probabilities are uniform. W_max is recorded as null in that '
             'case because no edge q is evaluated. `physical_length` is provided separately for inspection.\n',
             '## Files and reproduction\n',
             '- [Every candidate probability and all edge inputs](probabilities.csv)\n',
             '- [Both probability vectors for every robot query](queries.csv)\n',
             '- [Map and overall statistics, definitions and hashes](summary.json)\n',
             '- [Per-scenario statistics across the five endpoint variants](by_scenario.json)\n',
             '- [Exact replay-case examples](examples.json)\n',
             '- [Per-task source checksums](inputs.json)\n',
             '- [Probability comparison figure](comparison.png) ([SVG](comparison.svg))\n',
             '\n```bash\nPYTHONPATH=benchmarks .venv/bin/python -m multi_agent.warehouse_cost_probabilities '
             f'--lambda-weight {report["lambda_weight"]:g} --tau {report["tau"]:g}\n```\n',
             '\nThe CLI only reads saved candidates, graph bundles and audited bottleneck inputs. '
             'The new selector is callable through `edge_cost_distribution` and '
             '`draw_edge_cost_route` in `multi_agent.edge_cost_selection`. '
             'No simulation outcomes, success rates or makespans are inferred.\n']
    (output / 'README.md').write_text(''.join(text))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--suite', type=Path, default=ROOT / 'benchmarks/scenarios/warehouse_uniform')
    parser.add_argument('--baseline', type=Path, default=ROOT / 'benchmarks/results/warehouse_uniform_corrected')
    parser.add_argument('--weighted', type=Path, default=ROOT / 'benchmarks/results/warehouse_bottleneck_width')
    parser.add_argument('--output', type=Path, default=ROOT / 'benchmarks/scenarios/warehouse_uniform/edge_cost_probabilities')
    parser.add_argument('--lambda-weight', type=float, required=True)
    parser.add_argument('--tau', type=float, required=True)
    parser.add_argument('--task', action='append', dest='task_ids')
    args = vars(parser.parse_args())
    report = compare(**args)
    print(json.dumps({k: report[k] for k in ('tasks', 'robot_queries', 'candidate_routes', 'simulations_run')}, indent=2))


if __name__ == '__main__':
    main()

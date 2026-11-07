"""Generate, validate, visualize, and run saved multi-agent scenario suites."""

import argparse
import json
from pathlib import Path

from multi_agent import MAP_NAMES
from multi_agent.geometry import ROOT, load_suite


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", type=Path)
    commands = parser.add_subparsers(dest="command", required=True)
    generator = commands.add_parser("generate")
    generator.add_argument("--seed", type=int, default=20261016)
    warehouse = commands.add_parser("generate-warehouse", help="250 tasks: 25 designs × 5 variants × 2 maps, 32 identical agents each")
    warehouse.add_argument("--seed", type=int, default=20261026)
    warehouse.add_argument("--workers", type=int, default=4)
    preflight=commands.add_parser("preflight-warehouse",help="Verify both full-fleet anchor sets for every frozen task")
    preflight.add_argument("--workers",type=int,default=12)
    preflight.add_argument("--output",type=Path,default=ROOT/'benchmarks/results/warehouse_uniform_corrected')
    joint=commands.add_parser("certify-warehouse",help="Construct joint bounded-control feasibility schedules")
    joint.add_argument("--workers",type=int,default=12)
    joint.add_argument("--output",type=Path)
    evaluation = commands.add_parser("evaluate-warehouse", help="Paired RRT/K-route evaluation of all warehouse variants")
    evaluation.add_argument("--workers", type=int, default=12)
    evaluation.add_argument("--task", nargs="+")
    evaluation.add_argument("--output",type=Path,default=ROOT / "benchmarks/results/warehouse_uniform_corrected")
    evaluation.add_argument("--batch-size",type=int,default=128)
    evaluation.add_argument("--horizon-states",type=int,default=31)
    weighted=commands.add_parser('evaluate-warehouse-width',help='Width-proportional sampling from the frozen keep-right K-route sets')
    weighted.add_argument('--workers',type=int,default=20)
    weighted.add_argument('--task',nargs='+')
    weighted.add_argument('--baseline',type=Path,default=ROOT/'benchmarks/results/warehouse_uniform_corrected')
    weighted.add_argument('--output',type=Path,default=ROOT/'benchmarks/results/warehouse_bottleneck_width')
    weighted.add_argument('--prepare-only',action='store_true')
    weighted.add_argument('--no-publish',action='store_true')
    warehouse_report=commands.add_parser("report-warehouse",help="Export paired warehouse result tables and figures")
    warehouse_report.add_argument("--output",type=Path,default=ROOT / "benchmarks/results/warehouse_uniform_corrected")
    warehouse_report.add_argument("--destination",type=Path,default=ROOT / "benchmarks/WAREHOUSE_EVALUATION_RESULTS.md")
    warehouse_report.add_argument("--publish",action="store_true",help="Publish complete compact results and recorded GIFs beside the frozen tasks")
    commands.add_parser("validate")
    commands.add_parser("visualize")
    commands.add_parser("catalog-pilots",help="Audit saved pilot traces and publish the ten recorded category examples")
    report = commands.add_parser("report")
    report.add_argument("--output",type=Path,default=ROOT / "benchmarks/results/multi_agent_runs")
    report.add_argument("--audit",action="store_true",help="Recompute metrics from saved unmodified controls/states")
    runner = commands.add_parser("run")
    runner.add_argument("--scenario", nargs="+")
    runner.add_argument("--map", choices=MAP_NAMES)
    runner.add_argument("--min-robots", type=int, default=0)
    runner.add_argument("--max-robots", type=int, default=100000)
    runner.add_argument("--planner", choices=("rrt", "k_routes", "both"), default="both")
    runner.add_argument("--selection-seeds", type=int, nargs="+", default=[42])
    runner.add_argument("--controller-seeds", type=int, nargs="+", default=[42])
    runner.add_argument("--max-steps", type=int)
    runner.add_argument("--wall-seconds", type=float, default=120.)
    runner.add_argument("--workers", type=int, default=1)
    runner.add_argument("--pilot", action="store_true")
    runner.add_argument("--animate", action="store_true")
    runner.add_argument("--k", type=int, default=5)
    runner.add_argument("--deadlock-config", type=Path, help="JSON overrides of DeadlockConfig")
    runner.add_argument("--output", type=Path, default=ROOT / "benchmarks/results/multi_agent_runs")
    args = parser.parse_args()
    if args.suite is None:
        args.suite = ROOT / "benchmarks/scenarios" / ("warehouse_uniform" if args.command in ("generate-warehouse", "evaluate-warehouse", "evaluate-warehouse-width", "preflight-warehouse", "certify-warehouse") else "multi_agent")
    if args.command == "generate-warehouse":
        from multi_agent.warehouse_generation import generate_warehouse_suite
        generate_warehouse_suite(args.suite, seed=args.seed, workers=args.workers)
    elif args.command == "evaluate-warehouse":
        from multi_agent.warehouse_evaluation import run_suite
        run_suite(args.suite,args.output,workers=args.workers,task_ids=args.task,
                  config=dict(batch_size=args.batch_size,horizon_states=args.horizon_states))
    elif args.command == "preflight-warehouse":
        from multi_agent.warehouse_preflight import preflight_suite
        preflight_suite(args.suite,args.output,workers=args.workers)
    elif args.command == 'evaluate-warehouse-width':
        from multi_agent.warehouse_width_benchmark import run_width_suite
        result=run_width_suite(args.suite,args.baseline,args.output,workers=args.workers,
            task_ids=args.task,prepare_only=args.prepare_only,publish=not args.no_publish)
        print({k:result[k] for k in ('passed','tasks','candidate_routes')} if isinstance(result,dict) else result)
    elif args.command == "certify-warehouse":
        from multi_agent.warehouse_joint_feasibility import certify_suite
        certify_suite(args.suite,args.output or args.suite/'joint_feasibility',workers=args.workers)
    elif args.command == "report-warehouse":
        from multi_agent.warehouse_evaluation_report import write_report,publish_results
        print(publish_results(args.output) if args.publish else write_report(args.output,destination=args.destination))
    elif args.command == "generate":
        from multi_agent.generation import generate_suite
        generate_suite(args.suite, config=dict(seed=args.seed))
    elif args.command == "validate":
        from multi_agent.generation import validate_suite
        print(json.dumps({k:v for k,v in validate_suite(args.suite).items() if k != "results"}, indent=2))
    elif args.command == "visualize":
        from multi_agent.visualization import visualize_suite
        print(visualize_suite(args.suite))
    elif args.command == "catalog-pilots":
        from multi_agent.visualization import catalog_pilots
        catalog_pilots(args.suite)
    elif args.command == "report":
        from multi_agent.runner import report_saved_runs
        report_saved_runs(args.suite,args.output,audit=args.audit)
    else:
        from multi_agent.metrics import DeadlockConfig
        from multi_agent.runner import run_batch
        _, _, tasks = load_suite(args.suite)
        if args.scenario:
            unknown = set(args.scenario) - {t["id"] for t in tasks}
            if unknown: parser.error(f"Unknown scenario IDs: {sorted(unknown)}")
        tasks = [t for t in tasks if (not args.scenario or t["id"] in args.scenario)
                 and (not args.map or t["source_map"] == args.map)
                 and args.min_robots <= t["robot_count"] <= args.max_robots]
        if not tasks: parser.error("No scenarios match the filters")
        if args.workers < 1: parser.error("workers must be positive")
        config = DeadlockConfig(**json.loads(args.deadlock_config.read_text())) if args.deadlock_config else DeadlockConfig()
        run_batch(args.suite, tasks, ["rrt", "k_routes"] if args.planner == "both" else [args.planner], args.output,
                  args.selection_seeds, args.controller_seeds, workers=args.workers,
                  max_steps=args.max_steps, wall_seconds=args.wall_seconds, pilot=args.pilot,
                  k=args.k, deadlock_config=config, animate=args.animate)


if __name__ == "__main__":
    main()

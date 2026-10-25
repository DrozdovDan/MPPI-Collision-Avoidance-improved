"""Generate, validate, visualize, and run the saved 50-task multi-agent suite."""

import argparse
import json
from pathlib import Path

from multi_agent import MAP_NAMES
from multi_agent.geometry import ROOT, load_suite


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suite", type=Path, default=ROOT / "benchmarks/scenarios/multi_agent")
    commands = parser.add_subparsers(dest="command", required=True)
    generator = commands.add_parser("generate")
    generator.add_argument("--seed", type=int, default=20261016)
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
    if args.command == "generate":
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

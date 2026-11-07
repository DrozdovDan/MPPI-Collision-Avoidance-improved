"""Regenerate the sampler audit summary and its paired trajectory figure."""
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.stats import norm

root = Path(__file__).resolve().parent
repo = root.parents[1]
variants = ["old-truncation", "gaussian-future", "gaussian-all", "gaussian-one-step"]
summary, measurements, manifests = [], {}, {}
for name in variants:
    manifest = json.loads((root/name/"manifest.json").read_text())
    rows = [json.loads(s) for s in (root/name/"runs.jsonl").read_text().splitlines()]
    assert len(rows) == len(manifest["tasks"]) == 14
    assert len({r["run_id"] for r in rows}) == 14
    measurements[name], manifests[name] = {r["run_id"]: r for r in rows}, manifest
    counts = Counter(r["status"] for r in rows)
    groups = {s: [r for r in rows if r["scenario"] == s] for s in ["circle", "random"]}
    summary.append(dict(variant=name, runs=14, successes=counts["success"],
                        timeouts=counts["timeout"], collisions=counts["collision"],
                        circle_successes=sum(r["status"] == "success" for r in groups["circle"]),
                        circle_runs=len(groups["circle"]),
                        random_successes=sum(r["status"] == "success" for r in groups["random"]),
                        random_runs=len(groups["random"])))

reference = measurements[variants[0]]
for name in variants[1:]:
    assert set(measurements[name]) == set(reference)
    for key, row in measurements[name].items():
        for field in ["scenario", "agents", "instance", "repeat", "instance_seed", "seed"]:
            assert row[field] == reference[key][field]
    with np.load(root/variants[0]/"instances.npz") as a, np.load(root/name/"instances.npz") as b:
        assert set(a.files) == set(b.files)
        assert all(np.array_equal(a[k], b[k]) for k in a.files)
assert len({m["source_sha256"] for m in manifests.values()}) == 1
assert len({m["native_sha256"] for m in manifests.values()}) == 1
for a,b,expected in [("old-truncation","gaussian-future",{"truncate_rollouts"}),
                     ("gaussian-future","gaussian-all",{"reject_first_step_tails"}),
                     ("gaussian-all","gaussian-one-step",{"time_horizon"})]:
    left,right = manifests[a]["config"], manifests[b]["config"]
    assert {k for k in left if left[k] != right[k]} == expected

original = {r["run_id"]: r for r in map(json.loads,
            (repo/"results/uncertainty-smoke/runs.jsonl").read_text().splitlines())
            if r["method"] == "uncertainty-mppi"}
for key, row in reference.items():
    for field in ["status", "steps", "min_separation_m", "max_goal_error_m",
                  "infeasible_decisions", "mean_effective_samples"]:
        assert row[field] == original[key][field]

with (root/"summary.csv").open("w",newline="") as f:
    writer = csv.DictWriter(f,fieldnames=list(summary[0]))
    writer.writeheader(); writer.writerows(summary)

case = "circle-05-00-00-uncertainty-mppi"
demo = json.loads((root/"fixed-circle5/run.json").read_text())
assert demo["steps"] == measurements["gaussian-all"][case]["steps"] == 298
assert demo["max_goal_error_m"] == measurements["gaussian-all"][case]["max_goal_error_m"]
with np.load(root/"original.npz") as before, np.load(root/"fixed-circle5/trajectory.npz") as after:
    assert np.array_equal(before["goals"],after["goals"])
    fig,ax = plt.subplots(figsize=(8,4.5),layout="constrained")
    for data,label,color in [(before,"Original rejected samples","#d55e00"),
                             (after,"Corrected Gaussian proposals","#009e73")]:
        errors = np.linalg.norm(data["states"][:,:,:2]-data["goals"],axis=2).max(axis=1)
        ax.plot(0.1*np.arange(len(errors)),errors,label=label,color=color,lw=2)
        ax.scatter(0.1*(len(errors)-1),errors[-1],color=color,s=35)
    ax.axhline(0.4,color="gray",ls="--",lw=1,label="Goal tolerance")
    ax.set(xlabel="Simulation time (s)",ylabel="Maximum goal error (m)",
           title="Paired five-agent Circle replay",ylim=(0,12.5),xlim=(0,101))
    ax.legend(loc="upper right"); ax.grid(alpha=0.3)
    for suffix in ["png","pdf","svg"]:
        fig.savefig(root/("sampler-progress."+suffix),dpi=200,bbox_inches="tight")
    plt.close(fig)

# The conditional sampler changes the proposal's first moment near a limit.
mu,sigma,lo,hi = 1.0,1.0,-1.0,1.0
a,b = (lo-mu)/sigma,(hi-mu)/sigma
mass = norm.cdf(b)-norm.cdf(a)
conditional_mean = mu+sigma*(norm.pdf(a)-norm.pdf(b))/mass
clipped_mean = lo*norm.cdf(a)+mu*mass+sigma*(norm.pdf(a)-norm.pdf(b))+hi*norm.sf(b)
audit = dict(summary=summary, audit_source_sha256=manifests[variants[0]]["source_sha256"],
             audit_native_sha256=manifests[variants[0]]["native_sha256"],
             historical_raw_sha256=hashlib.sha256(
                 (repo/"results/paper-comparison/runs.jsonl").read_bytes()).hexdigest(),
             exact_legacy_replay=True, paired_tasks_and_geometries=True,
             example_case={name:{k:measurements[name][case][k] for k in ["status","steps"]}
                           for name in variants},
             proposal_example=dict(mean=mu,std=sigma,lower=lo,upper=hi,
                                   rejected_mean=float(conditional_mean),clipped_mean=float(clipped_mean)))
(root/"audit.json").write_text(json.dumps(audit,indent=2)+"\n")
print(json.dumps(audit,indent=2))

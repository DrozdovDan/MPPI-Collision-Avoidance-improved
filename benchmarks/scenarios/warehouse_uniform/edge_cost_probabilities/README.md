# Edge-cost versus bottleneck probabilities
λ = 1; temperature τ = 1. 250 frozen tasks, 8,000 robot queries and 25,381 unchanged candidate paths. No simulations or planners were run.
```text
W_max = max(w_e across all edges in this robot’s K routes)
q_e = clamp(1 - (w_e - 2*r)/(2*W_max), 0, 1)
L_k = sum(l_e)
C_k = L_k + lambda * sum(l_e*q_e)
p_k = softmax(-C_k/tau)
```
Use stored graph `edge.length` and `edge.min_width`, retaining repeated edges. As requested, start/goal/direct connectors are excluded, and partial-edge entries use the full stored edge length. The physical radius r is used without a safety-margin addition. W_max is recomputed for each robot’s returned candidate set. The stable softmax subtracts the minimum cost before exponentiation.
The comparison policy is unchanged: full-path transverse bottleneck width divided by the sum of those widths. Its widths need not equal the graph’s stored minimum-width field; both inputs are included in the CSV.
## Probability comparison
| Map | Queries with K > 1 | Mean largest p: bottleneck | Mean largest p: edge cost | Mean total variation | Changed modal path |
| --- | ---: | ---: | ---: | ---: | ---: |
| warehouse | 3407 | 0.5474 | 0.9665 | 0.5376 | 1588/3407 (46.6%) |
| warehouse2 | 3462 | 0.4067 | 0.9786 | 0.6738 | 2106/3462 (60.8%) |

Averages give each robot query equal weight. Total variation is `0.5 * sum(abs(p_cost - p_bottleneck))`. Changed modal path compares the first maximum index in each vector; it is a probability comparison, not a sampled assignment.
## The replay case
| Agent | Path index | Bottleneck (m) | Graph L (m) | Width exposure (m) | C | Bottleneck p | Edge-cost p |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 0 | 1.187 | 47.451 | 32.290 | 79.741 | 0.44506556 | 1.0302514e-14 |
| 1 | 1 | 1.480 | 28.219 | 19.316 | 47.535 | 0.55493444 | 1 |
| 2 | 0 | 1.187 | 7.729 | 4.694 | 12.422 | 0.31957457 | 1 |
| 2 | 1 | 1.187 | 68.768 | 50.977 | 119.745 | 0.31946439 | 2.4573994e-47 |
| 2 | 2 | 1.341 | 33.199 | 22.019 | 55.218 | 0.36096104 | 2.5937671e-19 |
| 11 | 0 | 7.359 | 9.478 | 5.252 | 14.731 | 0.86111087 | 1 |
| 11 | 1 | 1.187 | 25.120 | 21.613 | 46.732 | 0.13888913 | 1.2639473e-14 |
| 20 | 0 | 9.240 | 22.437 | 12.386 | 34.823 | 0.79560481 | 0.9998698 |
| 20 | 1 | 1.187 | 42.587 | 38.152 | 80.739 | 0.1021976 | 1.146081e-20 |
| 20 | 2 | 1.187 | 25.442 | 18.328 | 43.770 | 0.1021976 | 0.00013020225 |

## Zero-edge candidates
680 candidates have no stored graph edges. Their graph-only L and C are exactly zero despite a nonzero physical path length. They remain in the candidate sets and receive the probability implied by the formula; no length floor or connector penalty is added. If every candidate has no graph edges, all costs are zero and probabilities are uniform. W_max is recorded as null in that case because no edge q is evaluated. `physical_length` is provided separately for inspection.
## Files and reproduction
- [Every candidate probability and all edge inputs](probabilities.csv)
- [Both probability vectors for every robot query](queries.csv)
- [Map and overall statistics, definitions and hashes](summary.json)
- [Per-scenario statistics across the five endpoint variants](by_scenario.json)
- [Exact replay-case examples](examples.json)
- [Per-task source checksums](inputs.json)
- [Probability comparison figure](comparison.png) ([SVG](comparison.svg))

```bash
PYTHONPATH=benchmarks .venv/bin/python -m multi_agent.warehouse_cost_probabilities --lambda-weight 1 --tau 1
```

The CLI only reads saved candidates, graph bundles and audited bottleneck inputs. The new selector is callable through `edge_cost_distribution` and `draw_edge_cost_route` in `multi_agent.edge_cost_selection`. No simulation outcomes, success rates or makespans are inferred.

"""Convert oriented routing walks to ordered, repeat-preserving traffic usage."""

import math

from .traffic_model import DirectedEdge, ResourceTraversal


def _orient_path(path, routing, start_node):
    if not path:
        return []
    if all(isinstance(p, DirectedEdge) for p in path):
        directed = path
    elif all(isinstance(p, int) for p in path):
        def from_node(node):
            result = []
            for edge_id in path:
                edge = routing.edges[edge_id]
                if edge.u == node:
                    result.append(DirectedEdge(edge_id, 1))
                    node = edge.v
                elif edge.v == node:
                    result.append(DirectedEdge(edge_id, -1))
                    node = edge.u
                else:
                    return None
            return result
        first = routing.edges[path[0]]
        candidates = ([from_node(start_node)] if start_node is not None else
                      [from_node(n) for n in dict.fromkeys((first.u, first.v))])
        candidates = [p for p in candidates if p is not None]
        if len(path) == 1 and start_node is None:
            candidates = candidates[:1]  # documented u->v default for a single edge
        if len(candidates) != 1:
            raise ValueError("edge path is disconnected or direction is ambiguous; supply start_node or DirectedEdge")
        directed = candidates[0]
    else:
        raise ValueError("path must contain only edge IDs or only DirectedEdge objects")
    previous = start_node
    for traversal in directed:
        if traversal.direction not in (-1, 1):
            raise ValueError("direction must be +1 or -1")
        edge = routing.edges[traversal.edge_id]
        begin, end = (edge.u, edge.v) if traversal.direction == 1 else (edge.v, edge.u)
        if previous is not None and begin != previous:
            raise ValueError("routing path has disconnected consecutive edges")
        previous = end
    return directed


def path_resource_usage(traffic, routing, path, start_node=None, tolerance=1e-7):
    """Order by route entry arclength, retaining overlapping constraints.

    Touching intervals of one resource merge across edge boundaries even when
    another resource overlaps them. Leaving and re-entering, or reversing a
    narrow chain, produces a new traversal. Partial walks may start/end inside
    a resource. Entries/exits then describe those actual positions, not a
    fictitious complete crossing. Bare IDs require unambiguous orientation.
    """
    directed = _orient_path(list(path), routing, start_node)
    events, offset = [], 0.
    for step in directed:
        edge = routing.edges[step.edge_id]
        for interval in traffic.edge_intervals[step.edge_id]:
            if step.direction == 1:
                start, end = interval.start, interval.end
                entry, exit = interval.start_port, interval.end_port
            else:
                start, end = edge.length - interval.end, edge.length - interval.start
                entry, exit = interval.end_port, interval.start_port
            sign = interval.direction * step.direction
            events.append((offset + start, offset + end, interval.resource_id, sign, entry, exit))
        offset += edge.length
    events.sort(key=lambda e: (e[0], e[1], e[2]))
    groups, last = [], {}
    for start, end, resource_id, sign, entry, exit in events:
        key = (resource_id, sign)
        previous = last.get(key)
        resource = traffic.resources[resource_id]
        new_lap = (previous is not None and resource.closed and
                   entry in resource.entrances and groups[previous][5] in resource.exits)
        if (previous is not None and abs(groups[previous][1] - start) <= tolerance
                and math.dist(groups[previous][5].position, entry.position) <= tolerance and not new_lap):
            old = groups[previous]
            groups[previous] = (old[0], end, resource_id, sign, old[4], exit, old[6] + end - start)
        else:
            last[key] = len(groups)
            groups.append((start, end, resource_id, sign, entry, exit, end - start))
    groups.sort(key=lambda e: (e[0], e[1], e[2]))
    return [ResourceTraversal(resource_id, entry, exit,
                              "forward" if sign == 1 else "reverse" if sign == -1 else "port_to_port",
                              float(length))
            for _, _, resource_id, sign, entry, exit, length in groups]

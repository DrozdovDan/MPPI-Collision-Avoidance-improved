"""Joint command certification for acceleration-limited disk robots.

The nominal commands still come from MPPI/ORCA. A command is applied only if
the entire fleet can subsequently brake to rest without a swept-disk contact.
The previous certificate supplies a feasible fallback at the next timestep.
This requires synchronous, noiseless states and commands; it is not a proof
for delays, uncertain localization, or unmodeled dynamics.
"""

import numpy as np
import shapely


class JointBrakingSafetyFilter:
    def __init__(self, agent_params, profiles, obstacles, dt):
        self.dt = float(dt)
        self.n = len(agent_params)
        self.radius = np.array([p['size'] + a.get('rad_eps', 0.)
                                for p, a in zip(agent_params, profiles)])
        self.maximum = np.array([[p['v_max'], p['w_max']] for p in agent_params])
        self.minimum = np.array([[p['v_min'], p['w_min']] for p in agent_params])
        self.accel = np.array([p['v_accel_max'] for p in agent_params])
        self.brake = np.array([p['v_brake_max'] for p in agent_params])
        self.yaw_accel = np.array([p['w_accel_max'] for p in agent_params])
        if (np.any(self.brake <= 0) or np.any(self.yaw_accel <= 0) or
                np.any(self.minimum[:, 0] < 0)):
            raise ValueError('The braking certificate requires forward disk robots with positive braking bounds')
        self.horizon = 2 + int(np.ceil(max(np.max(self.maximum[:, 0]/self.brake),
            np.max(np.maximum(abs(self.minimum[:, 1]), self.maximum[:, 1])/self.yaw_accel))/self.dt))
        self.tree = shapely.STRtree([shapely.Polygon(p) for p in obstacles])
        self.i, self.j = np.triu_indices(self.n, 1)
        self.step = self.interventions = self.rejected_joint_proposals = 0
        self.certificate = None

    def bound(self, desired, previous):
        desired = np.clip(desired, self.minimum, self.maximum)
        rate = np.where(desired[:, 0] > previous[:, 0], self.accel, self.brake)*self.dt
        result = desired.copy()
        result[:, 0] = np.clip(desired[:, 0], previous[:, 0]-rate, previous[:, 0]+rate)
        delta = self.yaw_accel*self.dt
        result[:, 1] = np.clip(desired[:, 1], previous[:, 1]-delta, previous[:, 1]+delta)
        return result

    def plan(self, first):
        controls = [first.copy()]
        for _ in range(self.horizon-1):
            controls.append(self.bound(np.zeros_like(first), controls[-1]))
        return np.asarray(controls)

    def safe(self, states, plan):
        pose = states[:, :3].copy()
        trace = [pose[:, :2].copy()]
        for control in plan:
            pose[:, :2] += self.dt*control[:, :1]*np.c_[np.cos(pose[:, 2]), np.sin(pose[:, 2])]
            pose[:, 2] += self.dt*control[:, 1]
            trace.append(pose[:, :2].copy())
        xy = np.asarray(trace)
        relative = xy[:, self.i]-xy[:, self.j]
        delta = np.diff(relative, axis=0)
        squared = np.sum(delta*delta, axis=-1)
        t = np.divide(-np.sum(relative[:-1]*delta, axis=-1), squared,
                      out=np.zeros_like(squared), where=squared > 0)
        distance = np.linalg.norm(relative[:-1]+np.clip(t, 0., 1.)[..., None]*delta, axis=-1)
        if np.any(distance < self.radius[self.i]+self.radius[self.j]-1e-10):
            return False
        if len(self.tree.geometries):
            segments = shapely.linestrings(np.stack([xy[:-1], xy[1:]], axis=2).reshape(-1, 2, 2))
            if self.tree.query(segments, predicate='dwithin',
                               distance=np.tile(self.radius-1e-10, len(plan))).size:
                return False
        return True

    def __call__(self, states, nominal, previous):
        nominal = self.bound(np.asarray(nominal), previous)
        candidate = self.plan(nominal)
        if self.safe(states, candidate):
            selected = candidate
        else:
            self.rejected_joint_proposals += 1
            if self.certificate is None:
                selected = self.plan(self.bound(np.zeros_like(nominal), previous))
            else:
                selected = np.concatenate([self.certificate[1:], np.zeros((1, self.n, 2))])
            if not self.safe(states, selected):
                raise RuntimeError('The previous joint braking certificate is no longer valid')
            # Keep safe progress by unaffected robots. Rotate priority instead
            # of giving a permanent advantage to a particular agent ID.
            first = selected[0].copy()
            for i in np.roll(np.arange(self.n), -(self.step % self.n)):
                baseline = first[i].copy()
                for fraction in (1., .5, .25, .125):
                    trial = first.copy(); trial[i] = baseline + fraction*(nominal[i]-baseline)
                    proposal = self.plan(trial)
                    if self.safe(states, proposal):
                        first = trial; selected = proposal; break
        self.interventions += int(np.any(abs(selected[0]-nominal) > 1e-10))
        self.certificate = selected
        self.step += 1
        return selected[0].copy()

    def statistics(self):
        return dict(steps=self.step, intervened_steps=self.interventions,
                    rejected_joint_proposals=self.rejected_joint_proposals,
                    certificate='synchronous swept disks plus bounded braking to rest')

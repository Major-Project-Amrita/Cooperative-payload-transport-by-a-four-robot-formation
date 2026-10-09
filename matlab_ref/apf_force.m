function [vec, info] = apf_force(pos, goal, neighbors, apf_cfg, eps_cfg)
%APF_FORCE  Artificial Potential Field: attractive (goal) + repulsive (neighbors).
%
%   [vec, info] = APF_FORCE(pos, goal, neighbors, apf_cfg, eps_cfg)
%
%   pos        1x2 current position of this agent.
%   goal       1x2 attraction target (a waypoint, or an assigned formation slot).
%   neighbors  Mx2 positions of other agents to repel from. Pass zeros(0,2)
%              for an agent with no formation-mates (e.g. the rigid-transport
%              leader in Phase 2, which has no APF repulsion term at all).
%   apf_cfg    .w_att     attractive gain
%              .w_rep     repulsive gain
%              .rep_dist  repulsion acts only inside this radius (m)
%   eps_cfg    small constant, avoids divide-by-zero at pos == goal / neighbor
%
%   vec   1x2 UNNORMALISED resultant APF force = attractive + repulsive.
%   info  .f_att, .f_rep  the two components separately (already gain-scaled),
%         useful for logging or plotting the potential field's pull apart.
%
%   FORCE LAW
%   Attractive (conic, not quadratic):
%       F_att = w_att * (goal - pos) / ||goal - pos||
%   Constant magnitude everywhere except at the goal itself, so the agent
%   does not crawl to a stop far from the target the way a quadratic well
%   would. Local deceleration near the target is handled by the caller's own
%   slow_radius logic (see Simulation7_sdf.m), so nothing is lost by leaving
%   this term unsaturated.
%
%   Repulsive (linear in the gap, active only inside rep_dist):
%       F_rep = w_rep * sum_j  (rep_dist - d_j) * (pos - neighbor_j)/d_j ,  d_j < rep_dist
%   Ramps up linearly as the gap closes, is exactly zero at d = rep_dist (no
%   discontinuity at the boundary), and sums over every neighbor within range.
%   This is the SAME force law as the inter-robot repulsion already tuned and
%   verified in Simulation7_sdf.m (w_repulse = 1.5, repulse_dist = 1.2) - it
%   is pulled out here as a standalone, reusable, explicitly-named function
%   rather than left as inline arithmetic inside a loop, so the APF part of
%   the controller is visible and callable on its own.
%
%   Deliberately NOT the classical Khatib inverse-square law (1/d - 1/rep_dist):
%   that form diverges as d -> 0 and needs its own gain re-tuning. Linear
%   repulsion is an equally standard APF variant; swap in 1/d^2 if you need
%   the classical form for a report, but re-run world_feasibility.m and a
%   batch experiment afterward, because the safe grid_spacing values in the
%   README were generated against THIS force law.
%
%   OBSTACLE AVOIDANCE IS DELIBERATELY NOT COMPUTED HERE.
%   It is handled by p-VFH (see PVFH_SCAN.m). A polar-histogram valley search
%   degrades far more gracefully in narrow corridors than an APF repulsive
%   term, where two nearby obstacle faces can cancel and leave the agent with
%   zero usable gradient - the classical APF "local minimum" trap. The two
%   algorithms are combined exactly as Simulation7_sdf.m does it:
%     Phase 1 (forming up)     : CASCADED   - apf_force sets a virtual goal,
%                                 pvfh_scan steers toward it while dodging obstacles.
%     Phase 2 (rigid transport): WEIGHTED SUM - apf_force's attractive term and
%                                 pvfh_scan's steering vector are combined by
%                                 w_goal / w_vfh.
%   Both are standard, documented APF+VFH hybrid architectures.
%
%   See also PVFH_SCAN, GOAL_VECTOR, SIMULATION7_SDF.

f_att = apf_cfg.w_att * goal_vector(goal, pos, eps_cfg);

f_rep = [0 0];
for j = 1:size(neighbors,1)
    d_vec = pos - neighbors(j,:);
    d = norm(d_vec);
    if d < apf_cfg.rep_dist
        f_rep = f_rep + (d_vec / (d + eps_cfg)) * (apf_cfg.rep_dist - d);
    end
end
f_rep = apf_cfg.w_rep * f_rep;

vec = f_att + f_rep;

if nargout > 1
    info = struct('f_att', f_att, 'f_rep', f_rep);
end
end

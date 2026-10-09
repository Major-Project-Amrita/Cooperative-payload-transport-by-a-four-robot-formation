function vec = goal_vector(goal, pos, eps_val)
    d = goal - pos;
    vec = d / (norm(d) + eps_val);
end

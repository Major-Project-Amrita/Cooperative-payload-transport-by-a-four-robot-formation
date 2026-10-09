function xcheck(algo, sg, seed, name)
CFG_OVERRIDE = struct('algo', algo, 'seed', seed, 'run_name', name, 'interactive', false, ...
  'timesteps', 3000, 'viz', struct('enable', false), ...
  'log', struct('enable', true, 'file', fullfile('logs', [name '.csv'])));
CFG_OVERRIDE.vfh = struct('sense', struct('kind','lidar','sigma0',sg,'k_rng',0.03,'fov_deg',60,'max_range',inf,'frame','body'), 'n_beams', 288);
Simulation8_dynamic;
end

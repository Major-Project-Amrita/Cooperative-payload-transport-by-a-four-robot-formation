function LOG = log_step(LOG, cfg, t, phase, lpos, lhead, dist, clr, rpos)
    LOG.t(end+1)     = t;
    LOG.phase(end+1) = phase;
    LOG.lx(end+1)    = lpos(1);
    LOG.ly(end+1)    = lpos(2);
    LOG.lh(end+1)    = lhead;
    LOG.dist(end+1)  = dist;
    LOG.clr(end+1)   = clr;
    LOG.rx(end+1,:)  = rpos(:,1)';
    LOG.ry(end+1,:)  = rpos(:,2)';
end

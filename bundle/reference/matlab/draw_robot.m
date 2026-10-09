function draw_robot(p, r, col)
%DRAW_ROBOT  Draw one robot at its true physical radius.
%   Drawing the real footprint rather than a fixed-size marker means the
%   figure shows whether the robot actually fits, which a marker cannot.
    th = linspace(0, 2*pi, 20);
    fill(p(1)+r*cos(th), p(2)+r*sin(th), col, 'EdgeColor','k');
end

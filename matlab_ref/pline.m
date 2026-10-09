function h = pline(x, y, varargin)
%PLINE  Plot a curve as a line PLUS point markers.
%
%   h = PLINE(x, y, 'Name', value, ...)   same options as PLOT
%
%   MATLAB R2025a's WebGL figure engine on some Linux GPU drivers drops the
%   line segments wherever a curve bends, leaving only the straight
%   stretches on screen. Markers are drawn independently of the segments, so
%   the curve stays readable. A standalone file (not a script-local
%   function) so Octave can call it from anywhere in Simulation8_dynamic.
h = plot(x, y, '.-', 'MarkerSize', 6, varargin{:});
end

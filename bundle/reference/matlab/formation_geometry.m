function [offsets, half_diag, K] = formation_geometry(kind, spacing)
%FORMATION_GEOMETRY  Body-frame slot offsets for the supported shapes.
%
%   [offsets, half_diag, K] = FORMATION_GEOMETRY(kind, spacing)
%
%   kind:     'square' (2x2 reference payload block, default)
%             'line'   (1xK file, K=4)
%             'triangle' (K=3 equilateral)
%             'diamond' or 'v' (K=4 diamond: the square rotated 45 deg)
%   spacing:  centre-to-centre slot distance (m). For the diamond the
%             ADJACENT-slot distance equals spacing (vertices are s/sqrt(2)
%             from the centroid); for the triangle the circumradius is
%             s/sqrt(3) so side length equals spacing.
%
%   offsets   Kx2, centred on the centroid (sum = 0), body frame (+x forward)
%   half_diag formation circumradius = max ||offset||; the VFH sense margin
%             is half_diag + robot_radius and the feasibility gate needs
%             bottleneck > half_diag + robot_radius
%
%   Feasibility note for this repo's dynamic world (bottleneck half-corridor
%   1.22 m, robot_radius 0.18): square/diamond need 1.19 m, triangle
%   0.78 m -- all fit at s = 1.18. A 4-robot line at s = 1.18 has
%   half-length 1.77 m and CANNOT turn inside the 2.45 m corridor; use
%   spacing <= 0.6 for the line, or expect the feasibility check to fail
%   loudly.
%
%   See also FORMATION_CLEARANCE_DYN, WORLD_FEASIBILITY.

if nargin < 1 || isempty(kind), kind = 'square'; end
if nargin < 2, error('formation_geometry: spacing is required'); end

switch lower(kind)
    case 'square'
        K = 4;
        [gx, gy] = meshgrid(0:1, 0:1);
        offsets  = [gx(:) gy(:)] * spacing;
        offsets  = offsets - mean(offsets, 1);
    case 'line'
        K = 4;
        offs = ((0:K-1) - (K-1)/2) * spacing;
        offsets = [offs(:), zeros(K, 1)];
    case 'triangle'
        K = 3;
        R = spacing / sqrt(3);
        th = (0:K-1) * 2*pi/K + pi/2;      % apex forward
        offsets = R * [cos(th(:)) sin(th(:))];
    case {'diamond', 'v'}
        K = 4;
        d = spacing / sqrt(2);
        offsets = [0 d; -d 0; 0 -d; d 0];
    otherwise
        error('formation_geometry: unknown kind ''%s'' (square|line|triangle|diamond|v)', kind);
end

half_diag = max(sqrt(sum(offsets.^2, 2)));
end

function draw_grid(points, n, line_width, style)
%DRAW_GRID  Draw the formation lattice lines through the slot positions.
    X = reshape(points(:,1), n, n);
    Y = reshape(points(:,2), n, n);
    for r = 1:n, plot(X(r,:), Y(r,:), style, 'LineWidth', line_width); end
    for c = 1:n, plot(X(:,c), Y(:,c), style, 'LineWidth', line_width); end
end

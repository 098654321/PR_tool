function out = congestion_colormap(u)
%CONGESTION_COLORMAP Routing congestion color map (green -> yellow -> red).
%
%   rgb = congestion_colormap(u)
%       Map normalized scalar u in [0, 1] to RGB.
%
%   cmap = congestion_colormap([])
%       Return a 256x3 colormap for normalized values in [0, 1].

    if nargin == 0 || (isnumeric(u) && isempty(u))
        query = linspace(0, 1, 256).';
        out = map_congestion(query);
        return;
    end

    u = double(u);
    u_clamped = min(max(u, 0), 1.25);
    out = map_congestion(u_clamped);

    if isscalar(u)
        out = out(:).';
    end
end

function out = map_congestion(u)
    anchors_u = [0, 0.25, 0.5, 0.75, 1.0, 1.25];
    anchors_rgb = [
        0.15, 0.75, 0.15;
        0.40, 0.90, 0.40;
        1.00, 1.00, 0.20;
        1.00, 0.55, 0.00;
        0.85, 0.00, 0.00;
        0.50, 0.00, 0.50];
    out = interp1(anchors_u, anchors_rgb, u, 'linear', 'extrap');
    out = max(min(out, 1), 0);
end

function fig = draw_cob_unit_usage(unit_data, unit_id, all_ok, varargin)
%DRAW_COB_UNIT_USAGE Draw COB array switch/channel utilization for one unit.
%
%   fig = draw_cob_unit_usage(unit_data, unit_id, all_ok)
%   fig = draw_cob_unit_usage(..., 'Parent', ax, 'ShowLabels', true, 'SavePath', path)
%   fig = draw_cob_unit_usage(..., 'Phase', 'pre-route')

    %#ok<NASGU> all_ok kept for API compatibility
    p = inputParser;
    addParameter(p, 'Parent', [], @(x) isempty(x) || isgraphics(x, 'axes'));
    addParameter(p, 'ShowLabels', false, @islogical);
    addParameter(p, 'SavePath', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'Phase', 'post-solve', @(x) ischar(x) || isstring(x));
    parse(p, varargin{:});
    opts = p.Results;
    phase_label = char(opts.Phase);

    tile_size = 0.52;
    gap = 0.58;
    pitch = tile_size + gap;
    band_half = tile_size * 0.18;

    [rows, cols] = size(unit_data.switches.used);
    style = struct( ...
        'zero_color', [1, 1, 1], ...
        'edge_color', [0.25, 0.25, 0.25], ...
        'edge_lw', 0.6, ...
        'band_half', band_half);

    if isempty(opts.Parent)
        fig = figure('Color', 'w', 'Name', sprintf('COBUnit U%d (%s)', unit_id, phase_label));
        ax = axes('Parent', fig);
    else
        ax = opts.Parent;
        fig = ancestor(ax, 'figure');
    end
    cla(ax);
    hold(ax, 'on');

    draw_cob_tiles(ax, unit_data.switches, rows, cols, pitch, tile_size, style, opts.ShowLabels);
    draw_h_channels(ax, unit_data.h_channel, rows, cols, pitch, tile_size, style);
    draw_v_channels(ax, unit_data.v_channel, rows, cols, pitch, tile_size, style);

    x_max = (cols - 1) * pitch + tile_size / 2 + gap;
    y_max = (rows - 1) * pitch + tile_size / 2 + gap;
    xlim(ax, [-gap, x_max + gap]);
    ylim(ax, [-gap, y_max + gap]);
    axis(ax, 'equal');
    set(ax, 'YDir', 'normal');
    box(ax, 'on');
    xlabel(ax, 'COB column');
    ylabel(ax, 'COB row');
    title(ax, sprintf('COBUnit U%d MCF Resource Usage (%s)', unit_id, phase_label));

    colormap(ax, build_usage_colorbar_colormap());
    c = colorbar(ax);
    c.Label.String = 'Utilization';
    caxis(ax, [0, 1]);
    c.Ticks = 0:0.25:1;

    hold(ax, 'off');

    if strlength(string(opts.SavePath)) > 0
        exportgraphics(fig, opts.SavePath, 'Resolution', 150);
    end
end

function ratio = usage_ratio(used, total)
    if total <= 0
        ratio = 0;
    else
        ratio = used / total;
    end
end

function rgb = usage_to_rgb(ratio, style)
    if ratio <= 0
        rgb = style.zero_color;
    else
        rgb = congestion_colormap(ratio);
    end
end

function cmap = build_usage_colorbar_colormap(n)
    % Row 1 = white (used=0). Row j maps to normalized utilization.
    if nargin < 1
        n = 256;
    end
    cmap = zeros(n, 3);
    cmap(1, :) = [1, 1, 1];
    for j = 2:n
        level = (j - 1) / (n - 1);
        cmap(j, :) = congestion_colormap(level);
    end
end

function draw_cob_tiles(ax, switches, rows, cols, pitch, tile_size, style, show_labels)
    half = tile_size / 2;
    for r = 0:(rows - 1)
        for c = 0:(cols - 1)
            used = switches.used(r + 1, c + 1);
            total = switches.total(r + 1, c + 1);
            ratio = usage_ratio(used, total);
            rgb = usage_to_rgb(ratio, style);

            x0 = c * pitch - half;
            y0 = r * pitch - half;
            patch(ax, ...
                [x0, x0 + tile_size, x0 + tile_size, x0], ...
                [y0, y0, y0 + tile_size, y0 + tile_size], ...
                ratio, ...
                'FaceColor', 'flat', ...
                'EdgeColor', style.edge_color, ...
                'LineWidth', style.edge_lw);

            if show_labels && used > 0
                text(ax, c * pitch, r * pitch, ...
                    sprintf('%d/%d', used, total), ...
                    'HorizontalAlignment', 'center', ...
                    'VerticalAlignment', 'middle', ...
                    'FontSize', 7, ...
                    'FontWeight', 'bold', ...
                    'Color', pick_label_color(rgb));
            elseif show_labels
                text(ax, c * pitch, r * pitch, ...
                    '0', ...
                    'HorizontalAlignment', 'center', ...
                    'VerticalAlignment', 'middle', ...
                    'FontSize', 6, ...
                    'Color', [0.45, 0.45, 0.45]);
            end
        end
    end
end

function draw_channel_rect(ax, x_coords, y_coords, used, total, style)
    ratio = usage_ratio(used, total);
    patch(ax, x_coords, y_coords, ratio, ...
        'FaceColor', 'flat', ...
        'EdgeColor', style.edge_color, ...
        'LineWidth', style.edge_lw);
end

function draw_h_channels(ax, h_channel, rows, cols, pitch, tile_size, style)
    half = tile_size / 2;
    band_half = style.band_half;

    for r = 0:(rows - 1)
        for c = 0:(cols - 2)
            used = h_channel.used(r + 1, c + 1);
            total = h_channel.total(r + 1, c + 1);

            x_left = c * pitch + half;
            x_right = (c + 1) * pitch - half;
            y_center = r * pitch;
            y_line_bottom = y_center - band_half;
            y_line_top = y_center + band_half;

            draw_channel_rect(ax, ...
                [x_left, x_right, x_right, x_left], ...
                [y_line_bottom, y_line_bottom, y_line_top, y_line_top], ...
                used, total, style);
        end
    end
end

function draw_v_channels(ax, v_channel, rows, cols, pitch, tile_size, style)
    half = tile_size / 2;
    band_half = style.band_half;

    for r = 0:(rows - 2)
        for c = 0:(cols - 1)
            used = v_channel.used(r + 1, c + 1);
            total = v_channel.total(r + 1, c + 1);

            x_center = c * pitch;
            x_line_left = x_center - band_half;
            x_line_right = x_center + band_half;
            y_bottom = r * pitch + half;
            y_top = (r + 1) * pitch - half;

            draw_channel_rect(ax, ...
                [x_line_left, x_line_right, x_line_right, x_line_left], ...
                [y_bottom, y_bottom, y_top, y_top], ...
                used, total, style);
        end
    end
end

function color = pick_label_color(rgb)
    luminance = 0.299 * rgb(1) + 0.587 * rgb(2) + 0.114 * rgb(3);
    if luminance > 0.55
        color = [0, 0, 0];
    else
        color = [1, 1, 1];
    end
end

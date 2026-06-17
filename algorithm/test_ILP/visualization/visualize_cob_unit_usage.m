function fig = visualize_cob_unit_usage(unit_file_path, unit, varargin)
%VISUALIZE_COB_UNIT_USAGE Visualize MCF resource usage for one COBUnit.
%
%   fig = visualize_cob_unit_usage(unit_file_path, unit)
%   fig = visualize_cob_unit_usage(unit_file_path, unit, 'ShowLabels', true)
%   fig = visualize_cob_unit_usage(unit_file_path, unit, 'Phase', 'pre-route')
%   fig = visualize_cob_unit_usage(unit_file_path, unit, 'SavePath', 'u8.png')
%
%   unit_file_path : path to resource-usage/unitN.txt
%   unit           : COBUnit index 0..15 (must match file content)
%   Phase          : 'post-solve' (default) | 'pre-route'
%
%   Examples:
%     visualize_cob_unit_usage('../../output/resource-usage/unit8.txt', 8);
%     visualize_cob_unit_usage('../../output/resource-usage/unit8.txt', 8, 'Phase', 'pre-route');

    if nargin < 2
        error('visualize_cob_unit_usage:NotEnoughInputs', ...
            'Usage: visualize_cob_unit_usage(unit_file_path, unit, ...)');
    end

    unit = double(unit);
    if unit < 0 || unit > 15 || mod(unit, 1) ~= 0
        error('visualize_cob_unit_usage:BadUnit', ...
            'unit must be an integer in [0, 15], got %g.', unit);
    end

    p = inputParser;
    addParameter(p, 'ShowLabels', false, @islogical);
    addParameter(p, 'SavePath', '', @(x) ischar(x) || isstring(x));
    addParameter(p, 'Phase', 'post-solve', @(x) ischar(x) || isstring(x));
    addParameter(p, 'Parent', [], @(x) isempty(x) || isgraphics(x, 'axes'));
    parse(p, varargin{:});
    phase = char(p.Results.Phase);

    script_dir = fileparts(mfilename('fullpath'));
    addpath(script_dir);

    data = parse_mcf_resource_usage(unit_file_path, phase);
    unit_data = data.units(unit + 1);

    draw_args = {'ShowLabels', p.Results.ShowLabels, 'SavePath', p.Results.SavePath, ...
        'Phase', phase};
    if ~isempty(p.Results.Parent)
        draw_args = [draw_args, {'Parent', p.Results.Parent}];
    end
    fig = draw_cob_unit_usage(unit_data, unit, data.all_ok, draw_args{:});
end

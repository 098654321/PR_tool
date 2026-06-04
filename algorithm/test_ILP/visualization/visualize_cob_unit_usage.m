function fig = visualize_cob_unit_usage(log_path, unit, varargin)
%VISUALIZE_COB_UNIT_USAGE Visualize MCF resource usage for one COBUnit.
%
%   fig = visualize_cob_unit_usage(log_path, unit)
%   fig = visualize_cob_unit_usage(log_path, unit, 'ShowLabels', true)
%   fig = visualize_cob_unit_usage(log_path, unit, 'SavePath', 'u8.png')
%
%   log_path : path to debug.log (or any log containing MCF resource usage)
%   unit     : COBUnit index 0..15 (matches log "Unit N:")
%
%   Examples:
%     visualize_cob_unit_usage('../../output/debug.log', 8);
%     visualize_cob_unit_usage('projects/0602_visualization_5$8$13/case5.log', 0);

    if nargin < 2
        error('visualize_cob_unit_usage:NotEnoughInputs', ...
            'Usage: visualize_cob_unit_usage(log_path, unit, ...)');
    end

    unit = double(unit);
    if unit < 0 || unit > 15 || mod(unit, 1) ~= 0
        error('visualize_cob_unit_usage:BadUnit', ...
            'unit must be an integer in [0, 15], got %g.', unit);
    end

    script_dir = fileparts(mfilename('fullpath'));
    addpath(script_dir);

    data = parse_mcf_resource_usage(log_path);
    unit_data = data.units(unit + 1);

    fig = draw_cob_unit_usage(unit_data, unit, data.all_ok, varargin{:});
end

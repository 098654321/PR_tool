%MATLAB_MAIN Entry script for COBUnit MCF resource visualization.
% Edit the parameters below, then click Run in the editor (or press F5).

%% Parameters (edit here)
log_path        = '/Users/jiaheng/FDU_files/Tao_group/论文/papers/ASP-DAC27_SiIF/Projects/0614-test-pre-post-route/case5.log';
unit            = 15;          % COBUnit index 0..15
resource_phase  = 'post-solve';  % 'post-solve' | 'pre-route'
show_labels     = true;      % show used/total on each COB tile
save_png        = '';         % e.g. 'unit8.png'; leave empty to only display

%% Setup paths
script_dir = fileparts(mfilename('fullpath'));
addpath(script_dir);

if ~startsWith(log_path, '/') && ~(ispc && length(log_path) >= 2 && log_path(2) == ':')
    log_path = fullfile(script_dir, log_path);
end

if ~isfile(log_path)
    error('matlab_main:LogNotFound', 'Log file not found:\n  %s', log_path);
end

if strlength(string(save_png)) > 0
    if ~startsWith(save_png, '/') && ~(ispc && length(save_png) >= 2 && save_png(2) == ':')
        save_png = fullfile(script_dir, save_png);
    end
end

%% Run
fprintf('Log:   %s\n', log_path);
fprintf('Unit:  U%d\n', unit);
fprintf('Phase: %s\n', resource_phase);

if strlength(string(save_png)) > 0
    fig = visualize_cob_unit_usage(log_path, unit, ...
        'Phase', resource_phase, 'ShowLabels', show_labels, 'SavePath', save_png);
    fprintf('Saved: %s\n', save_png);
else
    fig = visualize_cob_unit_usage(log_path, unit, ...
        'Phase', resource_phase, 'ShowLabels', show_labels);
end

data = parse_mcf_resource_usage(log_path, resource_phase);
u = data.units(unit + 1);
fprintf('all_ok=%s  switches=%d/%d  channels=%d/%d\n', ...
    string(data.all_ok), ...
    u.summary.switches_used, u.summary.switches_total, ...
    u.summary.channels_used, u.summary.channels_total);

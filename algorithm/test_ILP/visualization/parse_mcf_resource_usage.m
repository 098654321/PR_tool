function data = parse_mcf_resource_usage(unit_file_path, phase)
%PARSE_MCF_RESOURCE_USAGE Parse MCF resource usage from a per-unit file.
%
%   data = parse_mcf_resource_usage(unit_file_path)
%   data = parse_mcf_resource_usage(unit_file_path, phase)
%
%   unit_file_path : path to resource-usage/unitN.txt
%   phase          : 'post-solve' (default) | 'pre-route'
%
%   Returns a struct compatible with visualize_cob_unit_usage; data.units
%   is a 16-element cell with parsed data in the matching unit slot.

    if nargin < 1 || strlength(string(unit_file_path)) == 0
        error('parse_mcf_resource_usage:InvalidInput', ...
            'unit_file_path must be a non-empty path to resource-usage/unitN.txt.');
    end
    if nargin < 2 || strlength(string(phase)) == 0
        phase = 'post-solve';
    end
    phase = char(phase);
    if ~strcmp(phase, 'post-solve') && ~strcmp(phase, 'pre-route')
        error('parse_mcf_resource_usage:BadPhase', ...
            'phase must be ''post-solve'' or ''pre-route'', got ''%s''.', phase);
    end

    if ~isfile(unit_file_path)
        error('parse_mcf_resource_usage:FileNotFound', ...
            'Unit resource file not found: %s', unit_file_path);
    end

    raw = fileread(unit_file_path);
    if strlength(strtrim(raw)) == 0
        error('parse_mcf_resource_usage:EmptyFile', ...
            ['Unit resource file is empty: %s. ', ...
             'Run test_ILP with --enable-mcf-routing --show-resource-usage.'], ...
            unit_file_path);
    end

    section_marker = sprintf('=== %s ===', phase);
    idx = strfind(raw, section_marker);
    if isempty(idx)
        hint = ['Run test_ILP with --enable-mcf-routing --show-resource-usage ', ...
                'and ensure this unit completed successfully for the requested phase.'];
        error('parse_mcf_resource_usage:NoResourceSection', ...
            'No ''%s'' section in %s. %s', section_marker, unit_file_path, hint);
    end

    tail = raw(idx(end):end);
    next_idx = regexp(tail, '\n=== ', 'once');
    if isempty(next_idx)
        section_text = tail;
    else
        section_text = tail(1:next_idx(1) - 1);
    end

    header_pat = sprintf('MCF resource usage \\(%s, unit=(\\d+), ok=(true|false)\\)', phase);
    header_match = regexp(section_text, header_pat, 'tokens', 'once');
    if isempty(header_match)
        error('parse_mcf_resource_usage:BadHeader', ...
            'Could not parse header for phase ''%s'' in %s.', phase, unit_file_path);
    end

    unit_id = str2double(header_match{1});
    if unit_id < 0 || unit_id > 15
        error('parse_mcf_resource_usage:BadUnitId', ...
            'Invalid unit id %d in %s.', unit_id, unit_file_path);
    end

    unit_pat = sprintf('Unit\\s+%d:', unit_id);
    unit_start = regexp(section_text, unit_pat, 'once');
    if isempty(unit_start)
        error('parse_mcf_resource_usage:NoUnitBlock', ...
            'Missing Unit %d block in phase ''%s'' (%s).', unit_id, phase, unit_file_path);
    end

    data = struct();
    data.phase = phase;
    data.all_ok = strcmp(header_match{2}, 'true');
    data.rows = 9;
    data.cols = 12;
    data.units = repmat(empty_unit(data.rows, data.cols), 1, 16);
    data.units(unit_id + 1) = parse_unit_block(section_text(unit_start:end), data.rows, data.cols);
end

function unit = empty_unit(rows, cols)
    unit = struct();
    unit.switches = struct( ...
        'used', zeros(rows, cols), ...
        'total', 48 * ones(rows, cols));
    unit.h_channel = struct( ...
        'used', zeros(rows, cols - 1), ...
        'total', 8 * ones(rows, cols - 1));
    unit.v_channel = struct( ...
        'used', zeros(rows - 1, cols), ...
        'total', 8 * ones(rows - 1, cols));
    unit.summary = struct( ...
        'switches_used', 0, ...
        'switches_total', 0, ...
        'switches_modeled', 0, ...
        'channels_used', 0, ...
        'channels_total', 0);
end

function unit = parse_unit_block(block, rows, cols)
    unit = empty_unit(rows, cols);

    sw_tokens = regexp(block, ...
        'switches COB\((\d+),(\d+)\)=(\d+)/(\d+)', 'tokens');
    for i = 1:numel(sw_tokens)
        r = str2double(sw_tokens{i}{1}) + 1;
        c = str2double(sw_tokens{i}{2}) + 1;
        used = str2double(sw_tokens{i}{3});
        total = str2double(sw_tokens{i}{4});
        unit.switches.used(r, c) = used;
        unit.switches.total(r, c) = total;
    end

    if numel(sw_tokens) ~= rows * cols
        error('parse_mcf_resource_usage:SwitchCount', ...
            'Expected %d switch entries, found %d.', ...
            rows * cols, numel(sw_tokens));
    end

    h_tokens = regexp(block, ...
        'channel H COB\((\d+),(\d+)\)-COB\((\d+),(\d+)\)=(\d+)/(\d+)', 'tokens');
    for i = 1:numel(h_tokens)
        r = str2double(h_tokens{i}{1}) + 1;
        c = str2double(h_tokens{i}{2}) + 1;
        used = str2double(h_tokens{i}{5});
        total = str2double(h_tokens{i}{6});
        unit.h_channel.used(r, c) = used;
        unit.h_channel.total(r, c) = total;
    end

    v_tokens = regexp(block, ...
        'channel V COB\((\d+),(\d+)\)-COB\((\d+),(\d+)\)=(\d+)/(\d+)', 'tokens');
    for i = 1:numel(v_tokens)
        r = str2double(v_tokens{i}{1}) + 1;
        c = str2double(v_tokens{i}{2}) + 1;
        used = str2double(v_tokens{i}{5});
        total = str2double(v_tokens{i}{6});
        unit.v_channel.used(r, c) = used;
        unit.v_channel.total(r, c) = total;
    end

    summary_match = regexp(block, ...
        'unit_summary switches=(\d+)/(\d+) modeled=(\d+) channels=(\d+)/(\d+)', ...
        'tokens', 'once');
    if ~isempty(summary_match)
        unit.summary.switches_used = str2double(summary_match{1});
        unit.summary.switches_total = str2double(summary_match{2});
        unit.summary.switches_modeled = str2double(summary_match{3});
        unit.summary.channels_used = str2double(summary_match{4});
        unit.summary.channels_total = str2double(summary_match{5});
    end
end

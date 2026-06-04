function data = parse_mcf_resource_usage(log_path)
%PARSE_MCF_RESOURCE_USAGE Parse MCF post-solve resource usage from debug.log.
%
%   data = parse_mcf_resource_usage(log_path)
%
%   Reads the last "MCF resource usage (post-solve, ...)" block and returns
%   per-COBUnit switch and channel utilization for all 16 units.

    if nargin < 1 || strlength(string(log_path)) == 0
        error('parse_mcf_resource_usage:InvalidInput', ...
            'log_path must be a non-empty path to a debug.log file.');
    end

    if ~isfile(log_path)
        error('parse_mcf_resource_usage:FileNotFound', ...
            'Log file not found: %s', log_path);
    end

    raw = fileread(log_path);
    marker = 'MCF resource usage (post-solve, all_ok=';
    idx = strfind(raw, marker);
    if isempty(idx)
        error('parse_mcf_resource_usage:NoResourceBlock', ...
            ['No MCF resource usage block found. Run test_ILP with ', ...
             '--enable-mcf-routing and ensure MCF completed.']);
    end

    tail = raw(idx(end):end);
    all_ok_match = regexp(tail, ...
        'MCF resource usage \(post-solve, all_ok=(true|false)\)', ...
        'tokens', 'once');
    if isempty(all_ok_match)
        error('parse_mcf_resource_usage:BadHeader', ...
            'Could not parse all_ok flag from MCF resource usage header.');
    end

    data = struct();
    data.all_ok = strcmp(all_ok_match{1}, 'true');
    data.rows = 9;
    data.cols = 12;
    data.units = repmat(empty_unit(data.rows, data.cols), 1, 16);

    unit_pat = 'Unit\s+(\d+):';
    unit_starts = regexp(tail, unit_pat, 'start');
    unit_ids = regexp(tail, unit_pat, 'tokens');
    if numel(unit_starts) < 16
        error('parse_mcf_resource_usage:IncompleteUnits', ...
            'Expected 16 Unit blocks, found %d.', numel(unit_starts));
    end

    for k = 1:16
        block_start = unit_starts(k);
        if k < numel(unit_starts)
            block_end = unit_starts(k + 1) - 1;
        else
            block_end = numel(tail);
        end
        block = tail(block_start:block_end);
        unit_id = str2double(unit_ids{k}{1});
        if unit_id < 0 || unit_id > 15
            error('parse_mcf_resource_usage:BadUnitId', ...
                'Invalid unit id %d in log.', unit_id);
        end

        unit = parse_unit_block(block, data.rows, data.cols);
        data.units(unit_id + 1) = unit;
    end
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

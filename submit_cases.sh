#!/usr/bin/env bash

# Submit selected test_ILP cases to LSF.

set -euo pipefail

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "${script_dir}/.." && pwd)"
pr_tool_dir="${workspace_dir}/PR_tool"

if [[ ! -d "$pr_tool_dir" ]]; then
    echo "PR_tool directory was not found next to Projects: $pr_tool_dir" >&2
    exit 1
fi
cd "$pr_tool_dir"

read -r -p 'Project subdirectory name under Projects: ' project_name
if [[ -z "$project_name" || "$project_name" == */* || "$project_name" == '.' || "$project_name" == '..' ]]; then
    echo 'Project name must be a non-empty single directory name without /, . or ..' >&2
    exit 2
fi

read -r -p 'Case numbers (comma- or space-separated, e.g. 7, 8, 12): ' case_input
case_input="${case_input//,/ }"
if [[ -z "${case_input//[[:space:]]/}" ]]; then
    echo 'Enter at least one case number.' >&2
    exit 2
fi
read -r -a case_numbers <<< "$case_input"

declare -A seen_cases=()
for number in "${case_numbers[@]}"; do
    if [[ ! "$number" =~ ^[0-9]+$ || ! "$number" =~ [1-9] ]]; then
        echo "Invalid case number: $number" >&2
        exit 2
    fi
    case_name="case${number}"
    if [[ -n "${seen_cases[$case_name]+x}" ]]; then
        echo "Duplicate case number: $number" >&2
        exit 2
    fi
    seen_cases[$case_name]=1
    if [[ ! -d "test/config/${case_name}" ]]; then
        echo "Case configuration was not found: test/config/${case_name}" >&2
        exit 2
    fi
done

read -r -p 'LSF node for bsub -m: ' node
if [[ -z "$node" || "$node" == *[[:space:]]* ]]; then
    echo 'Enter one non-empty node name without whitespace.' >&2
    exit 2
fi

read -r -p 'Additional test_ILP arguments (space-separated; omit -vv and -o): ' extra_args_line
extra_args=()
if [[ "$extra_args_line" =~ [^[:space:]] ]]; then
    read -r -a extra_args <<< "$extra_args_line"
    for arg in "${extra_args[@]}"; do
        if [[ "$arg" == '-o' || "$arg" == '--output' || "$arg" =~ ^-v+$ ]]; then
            echo "Do not include reserved argument '$arg'; the script always supplies -vv and -o." >&2
            exit 2
        fi
    done
fi

command -v bsub >/dev/null || {
    echo 'bsub was not found. Run this script on a Linux login node with LSF loaded.' >&2
    exit 127
}
command -v xmake >/dev/null || {
    echo 'xmake was not found. Load or install xmake before running this script.' >&2
    exit 127
}

echo 'Building test_ILP once before submitting jobs...'
xmake build test_ILP

test_binary='output/test_ILP'
if [[ ! -x "$test_binary" ]]; then
    echo "Built executable was not found: $test_binary" >&2
    exit 1
fi

for number in "${case_numbers[@]}"; do
    case_name="case${number}"
    output_dir="../Projects/${project_name}/${case_name}"
    mkdir -p "$output_dir"
    echo "Submitting ${case_name} to ${node}..."
    test_args=(-vv -o "$output_dir")
    if [[ "$extra_args_line" =~ [^[:space:]] ]]; then
        test_args+=("${extra_args[@]}")
    fi
    bsub \
        -m "$node" \
        -J "$case_name" \
        -o "${output_dir}/%J.out" \
        -e "${output_dir}/%J.err" \
        "$test_binary" "test/config/${case_name}" "${test_args[@]}"
done

echo 'All jobs were submitted.'

from pathlib import Path
import json
import re
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
rows = json.loads((HERE.parent / 'qa/stage_data.json').read_text())
log = (ROOT / 'Projects/v24-0930-reselection/No.3/case10/debug.log').read_text()
paths = re.findall(r'> route net=(\d+) demand=(\d+) source=(\d+) path=(.*)', log)
assert len(paths) == 320

def segment(node):
    m = re.fullmatch(r'T U\d+ ([HV])\((\d+),(\d+)\) idx=\d+', node)
    if not m:
        return None
    orientation, r, c = m[1], int(m[2]), int(m[3])
    return ((c-1, r), (c, r)) if orientation == 'H' else ((c, r-1), (c, r))

background, highlighted = set(), set()
highlight_paths = 0
for net, demand, source, path in paths:
    if net == '5':
        highlight_paths += 1
    for node in path.split(' -> '):
        edge = segment(node)
        if edge:
            (highlighted if net == '5' else background).add(edge)
assert highlight_paths == 8

plt.rcParams.update({'font.family': 'serif', 'font.serif': ['Times New Roman', 'DejaVu Serif'],
                     'font.size': 7, 'pdf.fonttype': 42, 'ps.fonttype': 42})
fig, (route_ax, gain_ax) = plt.subplots(1, 2, figsize=(3.48, 2.0),
    gridspec_kw={'width_ratios': [.95, 1.15]}, layout=None)
fig.subplots_adjust(left=.01, right=.97, bottom=.16, top=.90, wspace=.78)
route_ax.add_collection(LineCollection(list(background), colors='#c3c7cd', linewidths=.6))
route_ax.add_collection(LineCollection(list(highlighted), colors='#0072B2', linewidths=1.1))
grid = [(c, r) for r in range(9) for c in range(13)]
route_ax.scatter(*zip(*grid), marker='s', s=2.8, color='#E5C74D', edgecolors='#8d7f38', linewidths=.2, zorder=3)
route_ax.set(xlim=(-1.05, 13.05), ylim=(-1.05, 9.05), aspect='equal')
route_ax.axis('off')
route_ax.set_title('(a) Mu_CPU_1 routing', pad=3, fontsize=7)
route_ax.legend(handles=[Line2D([0], [0], color='#c3c7cd', lw=1, label='Other nets'),
                        Line2D([0], [0], color='#0072B2', lw=1.2, label='Multi-sink net')],
                loc='upper center', bbox_to_anchor=(.5, -.10), frameon=False,
                fontsize=6.5, ncol=1, handlelength=1.4, borderaxespad=0, labelspacing=.2)

selected = rows
ilp_gain = [100*(1-r['ilp_wl']/r['sat_wl']) for r in selected]
rrr_gain = [100*(r['ilp_wl']-r['final_wl'])/r['sat_wl'] for r in selected]
assert len(selected) == 13 and len({r['case'] for r in selected}) == 13
gain_ax.barh(range(13), ilp_gain, height=.68, color='#0072B2', label='ILP')
gain_ax.barh(range(13), rrr_gain, left=ilp_gain, height=.68, color='#D55E00', label='RRR')
gain_ax.set_yticks(range(13), [r['case'] for r in selected])
gain_ax.invert_yaxis()
gain_ax.set(xlim=(0, 20), xticks=[0, 10, 20], xlabel='Reduction vs. SAT (%)')
gain_ax.set_title('(b) Stage gains', pad=3, fontsize=7)
gain_ax.tick_params(axis='both', length=2, labelsize=6, pad=1)
gain_ax.spines[['top', 'right']].set_visible(False)
gain_ax.legend(loc='lower right', bbox_to_anchor=(1.04, .08), ncol=1,
               frameon=False, handlelength=1.1, fontsize=7, borderaxespad=0, labelspacing=.2)
fig.savefig(HERE / 'stage_results.pdf')
fig.savefig(HERE / 'stage_results.png', dpi=240)
plt.close(fig)
print('Generated final-route projection from 320 logged paths and stage gains from all 13 cases.')

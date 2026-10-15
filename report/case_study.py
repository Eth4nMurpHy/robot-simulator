"""Worked scheduling example for the report: one small instance, FCFS versus CP-SAT.

Run from the repository root with OR-Tools available:
    .venv/bin/python report/case_study.py
Writes results/case/{tasks,schedule_fcfs,schedule_cpsat}.csv, LaTeX table rows in results/case/*.tex
and the figure results/case_study.png. CP-SAT runs with one worker and a deterministic budget, so the
output is reproducible (same OR-Tools version).
"""
import csv
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

from simulator import generate_tasks, random_robot_soc, simulate  # noqa: E402

SEED, COUNT, RELEASE_GAP = 6, 12, 25
# Initial SOC as the app's "random per robot" option draws it for this seed (50/54/52/81 %).
CONFIG = dict(robot_soc=random_robot_soc(SEED), safety_soc=25, chargers=2)
CPSAT_OPTIONS = dict(workers=1, deterministic_time=3, time_limit=120)
OUT = ROOT / 'results' / 'case'
ROBOTS = ['R1', 'R2', 'R3', 'R4']
COLORS = {'move': '#c9ced6', 'load': '#3b75af', 'unload': '#3b75af', 'cross': '#d62728',
          'charge': '#2ca02c', 'charge_wait': '#ff9f1c', 'cross_wait': '#ff9f1c'}


def num(value, digits=0):
    return f'{value:.{digits}f}'


def write_csv(path, rows):
    with open(path, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def charges(result):
    """One row per charging session: arrival at H, queueing, charge window, SOC before/after."""
    rows = []
    for a in result['actions']:
        if a['action'] != 'charge':
            continue
        own = [b for b in result['actions'] if b['robot_id'] == a['robot_id'] and b['end_time'] <= a['start_time']]
        arrive = max(b['end_time'] for b in own if b['action'] == 'move' and b['to_node'] == 'H')
        wait = sum(b['end_time'] - b['start_time'] for b in own
                   if b['action'] == 'charge_wait' and b['start_time'] >= arrive)
        rows.append(dict(robot=a['robot_id'], bay=a['resource'], arrive=arrive, wait=wait, start=a['start_time'],
                         end=a['end_time'], soc_in=a['start_soc'], soc_out=a['end_soc']))
    return sorted(rows, key=lambda r: r['start'])


def crossings(result, robot):
    """Entry times into X; superscript c = heading to the charger, H = returning from it."""
    acts = [a for a in result['actions'] if a['robot_id'] == robot]
    out = []
    for i, a in enumerate(acts):
        if a['action'] != 'cross':
            continue
        nxt = next((b for b in acts[i + 1:] if b['action'] == 'move'), None)
        mark = r'$^{\mathrm{H}}$' if a['from_node'] == 'H' else r'$^{\mathrm{c}}$' if nxt and nxt['to_node'] == 'H' else ''
        out.append(num(a['start_time']) + mark)
    return out


def tex_tables(fcfs, cpsat):
    by_id = lambda result: {t['task_id']: t for t in result['tasks']}
    f, c = by_id(fcfs), by_id(cpsat)
    lines = []
    for t in sorted(cpsat['tasks'], key=lambda t: (t['release_time'], t['task_id'])):
        g = f[t['task_id']]
        loop = t['pickup'][0]
        lines.append(' & '.join([t['task_id'], f"{loop}: {t['pickup']}$\\to${t['dropoff']}",
                                 num(t['release_time']), num(t['due_time']), num(t['priority']),
                                 g['robot_id'], num(g['completion_time']), num(g['tardiness']),
                                 t['robot_id'], num(t['completion_time']), num(t['tardiness'])]) + r' \\')
    (OUT / 'tasks.tex').write_text('\n'.join(lines) + '\n')

    lines = []
    for label, result in [('FCFS', fcfs), ('CP-SAT', cpsat)]:
        for r in charges(result):
            lines.append(' & '.join([label, r['robot'], r['bay'], num(r['arrive']), num(r['wait'], 1),
                                     f"{num(r['start'])}--{num(r['end'])}", num(r['end'] - r['start']),
                                     f"{num(r['soc_in'], 1)} $\\to$ {num(r['soc_out'], 1)}"]) + r' \\')
        if label == 'FCFS':
            lines.append(r'\midrule')
    (OUT / 'charging.tex').write_text('\n'.join(lines) + '\n')

    lines = []
    for i, robot in enumerate(ROBOTS):
        acts = [a for a in cpsat['actions'] if a['robot_id'] == robot]
        cells = crossings(cpsat, robot)
        served = sum(t['robot_id'] == robot for t in cpsat['tasks'])
        lines.append(' & '.join([robot, cpsat['robots'][i]['loop'], str(served), ', '.join(cells),
                                 num(CONFIG['robot_soc'][i], 1), num(min(a['end_soc'] for a in acts), 1),
                                 num(acts[-1]['end_soc'], 1)]) + r' \\')
    (OUT / 'robots.tex').write_text('\n'.join(lines) + '\n')


def draw(fcfs, cpsat):
    fig, axes = plt.subplots(2, 2, figsize=(13, 6.4), sharex=True, gridspec_kw=dict(height_ratios=[1.1, 1]))
    end = max(fcfs['summary']['makespan'], cpsat['summary']['makespan']) + 40
    for col, (title, result) in enumerate([('FCFS', fcfs), ('CP-SAT', cpsat)]):
        gantt, soc = axes[0][col], axes[1][col]
        for row, robot in enumerate(ROBOTS):
            y = len(ROBOTS) - 1 - row
            for a in result['actions']:
                if a['robot_id'] != robot or a['action'] not in COLORS:
                    continue
                gantt.barh(y, a['end_time'] - a['start_time'], left=a['start_time'], height=.6,
                           color=COLORS[a['action']], edgecolor='none')
                if a['action'] == 'load':
                    gantt.text(a['start_time'], y + .36, a['task_id'][1:], fontsize=6.5, ha='left', va='bottom')
            times = [a['start_time'] for a in result['actions'] if a['robot_id'] == robot]
            socs = [a['start_soc'] for a in result['actions'] if a['robot_id'] == robot]
            last = [a for a in result['actions'] if a['robot_id'] == robot][-1]
            soc.plot(times + [last['end_time']], socs + [last['end_soc']], lw=1.4, label=robot)
        gantt.set_yticks(range(len(ROBOTS)), [f'{r} ({result["robots"][i]["loop"]})' for i, r in
                                              reversed(list(enumerate(ROBOTS)))])
        s = result['summary']
        gantt.set_title(f"{title}: makespan {s['makespan']:.0f} s, tardiness {s['total_tardiness']:.0f} s, "
                        f"charger queue {s['charge_wait']:.0f} s", fontsize=10)
        soc.axhline(CONFIG['safety_soc'], color='k', ls='--', lw=.8)
        soc.set_ylim(0, 100)
        soc.set_xlim(0, end)
        soc.set_xlabel('time (s)')
        soc.grid(alpha=.3)
    axes[1][0].set_ylabel('SOC (%)')
    axes[1][1].legend(loc='lower right', ncol=4, fontsize=8)
    handles = [Patch(color=COLORS[k], label=label) for k, label in
               [('move', 'travel'), ('load', 'load / unload (task no.)'), ('cross', 'crossing X'),
                ('charge', 'charging'), ('charge_wait', 'queueing')]]
    handles.append(plt.Line2D([], [], color='k', ls='--', lw=.8, label='safety threshold'))
    fig.legend(handles=handles, loc='upper center', ncol=6, fontsize=9, frameon=False)
    fig.tight_layout(rect=(0, 0, 1, .95))
    fig.savefig(ROOT / 'results' / 'case_study.png', dpi=160)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    tasks = generate_tasks(COUNT, SEED, release_gap=RELEASE_GAP)
    fcfs = simulate(tasks, CONFIG, 'fcfs')
    cpsat = simulate(tasks, CONFIG, 'cpsat', None, CPSAT_OPTIONS)
    for result in (fcfs, cpsat):
        assert result['validation']['passed'], result['validation']['errors']
    write_csv(OUT / 'tasks.csv', tasks)
    for name, result in [('fcfs', fcfs), ('cpsat', cpsat)]:
        write_csv(OUT / f'schedule_{name}.csv', result['actions'])
    tex_tables(fcfs, cpsat)
    draw(fcfs, cpsat)
    print('CP-SAT', cpsat.get('optimizer', {}).get('status'), {k: round(v, 3) for k, v in cpsat['summary'].items()
                                                               if isinstance(v, (int, float))})


if __name__ == '__main__':
    main()

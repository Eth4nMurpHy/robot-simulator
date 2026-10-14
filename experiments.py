"""Batch experiments from the proposal (section 7): repeated seeds, identical inputs per strategy.

    .venv/bin/python experiments.py --seeds 10 --out results
    python3 experiments.py --seeds 3 --strategies fcfs staged deadline joint   # no OR-Tools needed

Writes runs.csv (one row per run), summary.csv (mean/std/worst per group), results.md and figures.
"""
import argparse
import csv
import importlib.util
import math
import os
import statistics
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from simulator import STRATEGIES, InputError, generate_tasks, random_robot_soc, simulate

# cpsat-nores: ablation of offline CP-SAT without the end-of-horizon battery reserve the others keep.
OPTIMIZERS = ['cpsat', 'cpsat-nores', 'rolling'] if importlib.util.find_spec('ortools') else []
BASE = dict(count=40, a_share=.5, chargers=2, safety_soc=10, urgent=0)
# (experiment, setting label, overrides of BASE)
EXPERIMENTS = [
    ('algorithms', 'base', {}),
    *[('load', f'{n} tasks', {'count': n}) for n in (20, 40, 60)],
    *[('chargers', f'{n} bay' + 's' * (n > 1), {'chargers': n}) for n in (1, 2)],
    *[('imbalance', label, {'a_share': share}) for label, share in (('1:1', .5), ('2:1', 2 / 3), ('3:1', .75))],
    *[('safety', f'{s}%', {'safety_soc': s}) for s in (10, 20, 30)],
    *[('dynamic', f'+{round(u * 100)}% urgent', {'urgent': u}) for u in (0, .1, .2)],
]
METRICS = ['objective', 'makespan', 'total_tardiness', 'tardiness_rate', 'cross_wait', 'charge_wait',
           'energy', 'charging_sessions', 'violations', 'runtime']


def instance(setting, seed):
    tasks = generate_tasks(setting['count'], seed, setting['a_share'], urgent=setting['urgent'])
    config = dict(chargers=setting['chargers'], safety_soc=setting['safety_soc'], robot_soc=random_robot_soc(seed))
    return tasks, config


def run(job):
    setting, strategy, seed, options = job
    tasks, config = instance(dict(setting), seed)
    row = dict(strategy=strategy, seed=seed, tasks=len(tasks), status='ok', message='')
    started = time.perf_counter()
    try:
        if strategy == 'cpsat-nores':
            result = simulate(tasks, config, 'cpsat', None, {**options.get('cpsat', {}), 'terminal_reserve': False})
        else:
            result = simulate(tasks, config, strategy, None, options.get(strategy))
        row.update({k: result['summary'][k] for k in METRICS})
        row.update({f'term_{k}': v for k, v in result['summary']['objective_terms'].items()})
        solver = result.get('solver') or {}
        row.update(solver_status=solver.get('status', ''), gap=solver.get('gap', ''),
                   fallbacks=solver.get('fallback_count', int(bool(solver.get('fallback')))) if solver else '')
    except (InputError, RuntimeError) as exc:
        # A heuristic that cannot keep every robot above the safety SOC is an "energy deadlock".
        row.update(status='failed', message=str(exc)[:200], runtime=time.perf_counter() - started)
    return row


def add_relative(rows):
    # Paired comparison: seeds differ a lot, so also report J relative to FCFS on the same instance.
    ref = {(r['experiment'], r['setting'], r['seed']): r['objective'] for r in rows if r['strategy'] == 'fcfs' and r['status'] == 'ok'}
    for r in rows:
        base = ref.get((r['experiment'], r['setting'], r['seed']))
        r['j_vs_fcfs'] = r['objective'] / base - 1 if r['status'] == 'ok' and base else ''


def summarise(rows):
    groups = {}
    for row in rows:
        groups.setdefault((row['experiment'], row['setting'], row['strategy']), []).append(row)
    out = []
    for (experiment, setting, strategy), items in groups.items():
        ok = [r for r in items if r['status'] == 'ok']
        line = dict(experiment=experiment, setting=setting, strategy=strategy, runs=len(items),
                    failures=len(items) - len(ok))
        for key in METRICS + ['j_vs_fcfs']:
            values = [float(r[key]) for r in ok if r.get(key, '') != '']
            line[f'{key}_mean'] = statistics.fmean(values) if values else math.nan
            line[f'{key}_std'] = statistics.stdev(values) if len(values) > 1 else 0.
            line[f'{key}_worst'] = max(values) if values else math.nan
            line[f'{key}_median'] = statistics.median(values) if values else math.nan
        out.append(line)
    return out


def write_csv(path, rows):
    keys = list(dict.fromkeys(k for r in rows for k in r))
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, keys)
        writer.writeheader()
        writer.writerows(rows)


def markdown(summary, strategies):
    lines = ['# Experiment results', '',
             'Mean ± std over seeds. J is the normalised objective (each term divided by the FCFS value on the same',
             'instance, equal weights; FCFS = 1 unless one of its terms is below its floor, lower is better).',
             '"J vs FCFS" is the paired change against FCFS on the same instance.',
             'Medians are given because J is sensitive to near-zero FCFS terms (see the floors in README).',
             'Failures = runs with no safe schedule.', '']
    for experiment in dict.fromkeys(s['experiment'] for s in summary):
        lines += [f'## {experiment}', '', '| Setting | Strategy | J mean ± std | J median | J vs FCFS (mean / median) | Makespan (s) | Late rate | Tardiness (s) | X wait (s) | Charge wait (s) | Energy (%) | Runtime (s) | Failures |',
                  '|---|---|---|---|---|---|---|---|---|---|---|---|---|']
        for s in summary:
            if s['experiment'] != experiment:
                continue
            f = lambda k, d=0: f"{s[k + '_mean']:.{d}f} ± {s[k + '_std']:.{d}f}"
            lines.append(f"| {s['setting']} | {s['strategy']} | {f('objective', 3)} | {s['objective_median']:.3f} | {s['j_vs_fcfs_mean'] * 100:+.1f}% / {s['j_vs_fcfs_median'] * 100:+.1f}% | {f('makespan')} | {s['tardiness_rate_mean']:.2f} | "
                         f"{f('total_tardiness')} | {f('cross_wait', 1)} | {f('charge_wait')} | {f('energy', 1)} | {s['runtime_mean']:.2f} | {s['failures']}/{s['runs']} |")
        lines.append('')
    return '\n'.join(lines)


def figures(rows, summary, strategies, out):
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
    except ImportError:
        print('matplotlib not installed; skipping figures.')
        return
    colors = dict(zip(['fcfs', 'staged', 'deadline', 'joint', 'cpsat', 'cpsat-nores', 'rolling'],
                      ['#9a9a9a', '#c9a227', '#4a7fb5', '#3a8f5c', '#b4412b', '#e8a08f', '#7a4fa3']))
    base = [r for r in rows if r['experiment'] == 'algorithms' and r['status'] == 'ok']
    if base:
        fig, axes = plt.subplots(1, 3, figsize=(13, 4))
        for ax, key, title in zip(axes, ['objective', 'total_tardiness', 'charge_wait'], ['Objective J', 'Total tardiness (s)', 'Charging queue (s)']):
            data = [[float(r[key]) for r in base if r['strategy'] == s] for s in strategies]
            box = ax.boxplot(data, patch_artist=True)
            ax.set_xticks(range(1, len(strategies) + 1), strategies, rotation=20)
            for patch, s in zip(box['boxes'], strategies):
                patch.set_facecolor(colors.get(s, '#888'))
            ax.set_title(title)
        fig.tight_layout()
        fig.savefig(out / 'algorithms.png', dpi=160)
        plt.close(fig)
    for experiment in dict.fromkeys(s['experiment'] for s in summary if s['experiment'] != 'algorithms'):
        items = [s for s in summary if s['experiment'] == experiment]
        settings = list(dict.fromkeys(s['setting'] for s in items))
        fig, axes = plt.subplots(1, 2, figsize=(11, 4))
        width = .8 / len(strategies)
        for ax, key, title in zip(axes, ['objective', 'tardiness_rate'], ['Objective J (median, IQR)', 'Late-task rate (median, IQR)']):
            for j, strategy in enumerate(strategies):
                stats = []
                for g in settings:
                    values = sorted(float(r[key]) for r in rows if r['experiment'] == experiment and r['setting'] == g
                                    and r['strategy'] == strategy and r['status'] == 'ok')
                    q = statistics.quantiles(values, n=4) if len(values) > 1 else values * 3
                    stats.append((statistics.median(values), q[0], q[2]) if values else (math.nan, 0, 0))
                ax.bar([i + j * width for i in range(len(settings))], [m for m, _, _ in stats], width,
                       yerr=[[m - lo for m, lo, _ in stats], [hi - m for m, _, hi in stats]],
                       label=strategy, color=colors.get(strategy, '#888'), capsize=2)
            ax.set_xticks([i + width * (len(strategies) - 1) / 2 for i in range(len(settings))], settings)
            ax.set_title(f'{title} — {experiment}')
        handles, labels = axes[0].get_legend_handles_labels()
        fig.legend(handles, labels, loc='upper center', ncol=len(strategies), fontsize=8, frameon=False)
        fig.tight_layout(rect=(0, 0, 1, 0.93))
        fig.savefig(out / f'{experiment}.png', dpi=160)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--seeds', type=int, default=10)
    parser.add_argument('--first-seed', type=int, default=1)
    parser.add_argument('--strategies', nargs='+', default=[*STRATEGIES, *OPTIMIZERS])
    parser.add_argument('--experiments', nargs='+', default=list(dict.fromkeys(e for e, _, _ in EXPERIMENTS)))
    parser.add_argument('--time-limit', type=float, default=120, help='CP-SAT wall seconds per offline solve')
    parser.add_argument('--workers', type=int, default=min(8, os.cpu_count() or 1), help='CP-SAT workers (offline)')
    parser.add_argument('--jobs', type=int, default=max(1, (os.cpu_count() or 2) // 2), help='parallel heuristic/rolling runs')
    parser.add_argument('--out', default='results')
    parser.add_argument('--reuse', help='runs.csv of an earlier run: reuse its rows for --reuse-strategies')
    parser.add_argument('--reuse-strategies', nargs='+', default=['cpsat'])
    parser.add_argument('--shard', default='0/1', help='K/N: run only every N-th distinct instance, offset K (merge with --reuse)')
    args = parser.parse_args()
    unknown = set(args.strategies) - {*STRATEGIES, *OPTIMIZERS}
    if unknown:
        parser.error('Unknown or unavailable strategies: ' + ', '.join(sorted(unknown)))
    options = dict(cpsat=dict(time_limit=args.time_limit, workers=args.workers))
    groups = [(e, label, {**BASE, **override}) for e, label, override in EXPERIMENTS if e in args.experiments]
    seeds = range(args.first_seed, args.first_seed + args.seeds)
    # The base setting recurs in several experiments: solve each distinct instance once.
    try:
        shard, shards = map(int, args.shard.split('/'))
        assert 0 <= shard < shards
    except (ValueError, AssertionError):
        parser.error('--shard must be K/N with 0 <= K < N')
    # Shard distinct instances (setting, seed), not runs, so every strategy of an instance stays together.
    instances = list(dict.fromkeys((tuple(sorted(setting.items())), seed) for _, _, setting in groups for seed in seeds))
    unique = [(inst, s, seed) for inst, seed in instances[shard::shards] for s in args.strategies]
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    done = {}
    if args.reuse:
        # Keys must match this run's settings; rows of other strategies are recomputed.
        labels = {(e, label): tuple(sorted(setting.items())) for e, label, setting in groups}
        with open(args.reuse, newline='') as f:
            for row in csv.DictReader(f):
                key = (labels.get((row['experiment'], row['setting'])), row['strategy'], int(row['seed']))
                if key[0] and row['strategy'] in args.reuse_strategies and key in set(unique):
                    text = ('experiment', 'setting', 'strategy', 'status', 'message', 'solver_status')
                    counts = ('seed', 'tasks', 'charging_sessions', 'violations', 'fallbacks')
                    numeric = {k: (int if k in counts else float)(float(v)) for k, v in row.items() if k not in text and v != ''}
                    done[key] = {k: v for k, v in row.items() if k not in ('experiment', 'setting', 'j_vs_fcfs')} | numeric
        print(f'Reusing {len(done)} rows from {args.reuse}.')

    def record(key, row):
        done[key] = row
        print(f"[{len(done)}/{len(unique)}] {dict(key[0])} {row['strategy']:<9} seed {row['seed']:<3} "
              + (f"J={row['objective']:.3f} ({row['runtime']:.1f}s)" if row['status'] == 'ok' else 'FAILED ' + row['message']), flush=True)

    light = [k for k in unique if not k[1].startswith('cpsat') and k not in done]
    with ProcessPoolExecutor(args.jobs) as pool:
        for key, row in zip(light, pool.map(run, [(*k, options) for k in light])):
            record(key, row)
    for key in (k for k in unique if k[1].startswith('cpsat') and k not in done):  # one at a time: CP-SAT uses its own workers
        record(key, run((*key, options)))
    rows = [dict(experiment=e, setting=label, **done[key])
            for e, label, setting in groups for seed in seeds for s in args.strategies
            if (key := (tuple(sorted(setting.items())), s, seed)) in done]
    add_relative(rows)
    summary = summarise(rows)
    write_csv(out / 'runs.csv', rows)
    write_csv(out / 'summary.csv', summary)
    (out / 'results.md').write_text(markdown(summary, args.strategies))
    figures(rows, summary, args.strategies, out)
    print(f'Wrote {out}/runs.csv, summary.csv, results.md and figures.')


if __name__ == '__main__':
    main()

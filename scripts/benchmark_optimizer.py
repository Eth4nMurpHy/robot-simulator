"""Run the report matrix with .venv/bin/python scripts/benchmark_optimizer.py."""
import argparse
import json
from pathlib import Path
import platform
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from simulator import generate_tasks, simulate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='docs/optimizer_benchmark.json')
    args = parser.parse_args()
    import ortools
    report = dict(python=platform.python_version(), ortools=ortools.__version__,
                  platform=platform.platform(), options=dict(cpsat=dict(time_limit=30),rolling={}), rows=[])
    for seed in (1,2,3):
        for count in (20,40):
            tasks = generate_tasks(count,seed)
            for strategy in ('fcfs','deadline','joint','cpsat','rolling'):
                row = dict(seed=seed,count=count,strategy=strategy)
                try:
                    result = simulate(tasks,strategy=strategy,
                                      custom_strategy=report['options'].get(strategy))
                    row.update(result['summary'])
                    row['solver'] = result.get('solver')
                except Exception as exc:
                    row['error'] = str(exc)
                report['rows'].append(row)
                Path(args.output).write_text(json.dumps(report,indent=2)+'\n')
                print(json.dumps({k:v for k,v in row.items() if k!='solver'}),flush=True)


if __name__ == '__main__':
    main()

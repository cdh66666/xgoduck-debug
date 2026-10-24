"""Compare saved independent evaluations, retaining failed seeds in the plot.

Plotting only: requires matplotlib in addition to the training requirements.
"""
import argparse
from collections import Counter
import json
import math
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--evaluation', nargs=2, action='append', required=True,
                        metavar=('LABEL', 'JSON'))
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    reports = [(label, json.loads(Path(path).read_text('utf-8')))
               for label, path in args.evaluation]
    commands = [[0., 0., 0.], [.2, 0., 0.], [.3, 0., 0.],
                [-.3, 0., 0.], [0., 0., 1.], [0., 0., -1.]]
    titles = ['Stand', 'Forward 0.2 m/s', 'Forward 0.3 m/s',
              'Backward -0.3 m/s', 'Turn +1 rad/s', 'Turn -1 rad/s']
    colors = ['#1976b9', '#e36a23', '#32945b', '#ad49a1']
    fig, axes = plt.subplots(3, 2, figsize=(14, 11), constrained_layout=True)
    summary = {'scope': 'Native MuJoCo only; no resets, recovery or hardware deployment',
               'evaluations': []}
    for model_index, (label, report) in enumerate(reports):
        color = colors[model_index % len(colors)]
        item = {'label': label, 'result': report['result'],
                'checkpoint_sha256': report['checkpoint_sha256'], 'commands': []}
        for command_index, command in enumerate(commands):
            ax = axes.flat[command_index]
            cases = [case for case in report['cases'] if case['command'] == command]
            passed = sum(case['passed'] for case in cases)
            failures = Counter(reason for case in cases for reason in case['failure'] or [])
            stop_rows = [row for case in cases for row in case['rows']
                         if row['t'] >= (3 if command_index == 0 else 15)
                         and row['velocity'] is not None]
            item['commands'].append({
                'command': command, 'passed': passed, 'total': len(cases),
                'complete': sum(case['ticks'] == (750 if command_index == 0 else 900) for case in cases),
                'failure_counts': dict(failures),
                'minimum_recorded_s': min(case['ticks'] * .02 for case in cases),
                'max_tilt_deg': max(case['max_tilt_deg'] or 0 for case in cases),
                'mean_velocity_by_seed': [case['mean_velocity'] for case in cases],
                'rmse_by_seed': [case['rmse'] for case in cases],
                'stop_samples': len(stop_rows),
                'max_stop_planar_mps': max((math.hypot(*r['velocity'][:2]) for r in stop_rows), default=None),
                'max_stop_yaw_radps': max((abs(r['velocity'][2]) for r in stop_rows), default=None),
            })
            axis = 2 if command[2] else 0
            for seed_index, case in enumerate(cases):
                rows = [r for r in case['rows'] if r['velocity'] is not None]
                x = [r['t'] for r in rows]
                y = [r['velocity'][axis] for r in rows]
                ax.plot(x, y, color=color, alpha=.55, linewidth=.85,
                        label=f'{label}: {passed}/{len(cases)} pass' if seed_index == 0 else None)
                if case['failure'] and rows:
                    ax.plot(x[-1], y[-1], 'x', color=color, markersize=7)
        summary['evaluations'].append(item)
    for index, (command, title, ax) in enumerate(zip(commands, titles, axes.flat)):
        end = 15 if index == 0 else 18
        axis = 2 if command[2] else 0
        ax.plot([0, 12, 12, end], [command[axis], command[axis], 0, 0],
                color='#333333', linestyle='--', linewidth=1.1, label='Command')
        ax.axvspan(3 if index == 0 else 15, end, color='#32945b', alpha=.07)
        ax.axhline(.08 if axis == 2 else .03, color='#999999', linewidth=.6, linestyle=':')
        ax.axhline(-.08 if axis == 2 else -.03, color='#999999', linewidth=.6, linestyle=':')
        ax.set(title=title, xlabel='Time (s)', ylabel='Yaw (rad/s)' if axis == 2 else 'Body vx (m/s)', xlim=(0, end))
        ax.grid(alpha=.2)
        ax.legend(fontsize=8, loc='upper right')
    fig.suptitle('Independent native simulation | all seeds shown | x = failure\n'
                 'Green window: scored stand/stop; pass includes all axes, tilt and completion', fontsize=13)
    args.output.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output / 'comparison.png', dpi=160)
    plt.close(fig)
    (args.output / 'comparison.json').write_text(json.dumps(summary, indent=2, allow_nan=False), 'utf-8')
    print(json.dumps(summary, indent=2))


if __name__ == '__main__':
    main()

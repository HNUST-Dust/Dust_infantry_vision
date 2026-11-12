#!/usr/bin/env python3
"""Evaluate offline MPC replay against an independent, condensed OSQP problem."""
import argparse
import csv
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys
import time
from unittest import mock

import numpy as np
from scipy import sparse
import osqp


def condense(problem):
    """Return H,g for .5*u.T*H*u+g.T*u in the exported normalized input units."""
    a = np.asarray(problem['a'], dtype=float)
    b = np.asarray(problem['b'], dtype=float).reshape(2)
    state_scale = np.asarray(problem.get('state_scale', [1., 1.]), dtype=float)
    ref = np.asarray(problem['reference'], dtype=float) / state_scale[:, None]
    q = np.diag(problem['q'])
    terminal = np.asarray(problem['terminal_hessian'], dtype=float)
    terminal_ref = np.asarray(problem['terminal_reference_weight'], dtype=float)
    n = ref.shape[1] - 1
    h = np.eye(n) * problem['r']
    g = np.zeros(n)
    state = np.asarray(problem['x0'], dtype=float) / state_scale
    mapping = np.zeros((2, n))
    for k in range(n + 1):
        weight = terminal if k == n else q
        linear = -(terminal_ref.T @ ref[:, k]) if k == n else -(q @ ref[:, k])
        h += mapping.T @ weight @ mapping
        g += mapping.T @ (weight @ state + linear)
        if k < n:
            state = a @ state
            mapping = a @ mapping
            mapping[:, k] += b
    h = (h + h.T) / 2
    if not np.isfinite(h).all() or not np.isfinite(g).all():
        raise ValueError('Nonfinite condensed QP')
    return h, g


def objective(h, g, u):
    return float(.5 * u @ h @ u + g @ u)


def kkt_error(h, g, u, limit, scale):
    gradient = (h @ u + g) * scale
    return float(np.max(np.abs(u - np.clip(u - gradient, -limit, limit))))


def oracle(h, g, limit):
    # Apply one common objective multiplier; this changes no minimizer.
    scale = 1 / max(float(np.max(np.abs(h))), float(np.max(np.abs(g))), 1e-12)
    solver = osqp.OSQP()
    solver.setup(P=sparse.csc_matrix(np.triu(h * scale)), q=g * scale,
                 A=sparse.eye(len(g), format='csc'),
                 l=np.full(len(g), -limit), u=np.full(len(g), limit),
                 eps_abs=1e-9, eps_rel=1e-9, max_iter=100000,
                 polishing=True, verbose=False)
    answer = solver.solve()
    if answer.info.status_val == osqp.constant('OSQP_SIGINT'):
        raise KeyboardInterrupt
    u = answer.x
    valid = (answer.info.status_val == 1 and u is not None and np.isfinite(u).all())
    violation = None
    kkt = None
    if valid:
        violation = float(max(0, np.max(np.abs(u)) - limit))
        kkt = kkt_error(h, g, u, limit, scale)
        valid = violation <= 1e-7 and kkt <= 1e-6
    return {'valid': bool(valid), 'status': answer.info.status,
            'inputs': u if valid else None, 'kkt': kkt, 'bound_violation': violation,
            'ms': float(answer.info.run_time * 1000), 'scale': scale}


def percentile(values, p):
    return float(np.percentile(values, p)) if values else None


def summarize(rows):
    result = []
    for axis in sorted({r['axis'] for r in rows}):
        for variant in sorted({r['variant'] for r in rows}):
            for group in ['all', 'small_error', 'low_speed', 'horizon_jump']:
                selected = [r for r in rows if r['axis'] == axis and r['variant'] == variant
                            and (group == 'all' or r[group])]
                if not selected:
                    continue
                errors = [r['acceleration_error_rad_s2'] for r in selected
                          if r['acceleration_error_rad_s2'] is not None]
                times = [r['solve_ms'] for r in selected]
                result.append({'axis': axis, 'variant': variant, 'group': group,
                               'samples': len(selected),
                               'solved': sum(r['solved'] for r in selected),
                               'oracle_failures': sum(not r['oracle_valid'] for r in selected),
                               'accuracy_samples': len(errors),
                               'p95_acceleration_error_rad_s2': percentile(errors, 95),
                               'max_acceleration_error_rad_s2': max(errors) if errors else None,
                               'p95_solve_ms': percentile(times, 95),
                               'p99_solve_ms': percentile(times, 99),
                               'max_solve_ms': max(times),
                               'over_2ms': sum(t > 2 for t in times),
                               'over_10ms': sum(t > 10 for t in times)})
    return result


def evaluate_record(record):
    if record.get('precision_schema') != 1:
        raise ValueError('Unsupported precision export schema')
    rows = []
    combined = []
    for problem in record['axes']:
        h, g = condense(problem)
        limit = problem['normalized_input_limit']
        ref = np.asarray(problem['reference'])
        error = float(ref[0, 1] - problem['x0'][0])
        reference = oracle(h, g, limit)
        for candidate in problem['candidates']:
            d = candidate['diagnostic']
            row = {'synthetic': record['synthetic'], 'session': record['session'],
                   'plan_id': record['plan_id'], 'axis': problem['axis'],
                   'variant': candidate['variant'], 'initial_error_rad': error,
                   'initial_velocity_rad_s': problem['x0'][1],
                   'small_error': abs(error) < np.deg2rad(.5),
                   'low_speed': abs(problem['x0'][1]) < .05,
                   'horizon_jump': bool(np.max(np.abs(np.diff(ref[0]))) > np.deg2rad(1)),
                   'tolerance': candidate['config']['primal_tolerance'],
                   'max_iter': candidate['config']['max_iterations'],
                   'solved': d['solved'], 'status': d['status'],
                   'iterations': d['iterations'], 'solve_ms': d['solve_time_ms'],
                   'recorded_planning_ms': record['recorded_planning_ms'],
                   'riccati_cache_residual': problem['riccati_cache_residual'],
                   'oracle_valid': reference['valid'], 'oracle_status': reference['status'],
                   'oracle_ms': reference['ms'], 'oracle_scaled_kkt_inf': reference['kkt'],
                   'acceleration_rad_s2': None, 'oracle_acceleration_rad_s2': None,
                   'acceleration_error_rad_s2': None, 'objective_gap': None,
                   'input_bound_violation_rad_s2': None, 'candidate_scaled_kkt_inf': None}
            if reference['valid']:
                row['oracle_acceleration_rad_s2'] = float(reference['inputs'][0] * problem['input_scale'])
            if d['solved']:
                inputs = np.asarray(candidate['normalized_inputs'], dtype=float).reshape(-1)
                if inputs.shape != g.shape or not np.isfinite(inputs).all():
                    raise ValueError('Invalid successful candidate control sequence')
                # State constraints in this production model are +/-1e17. Refuse to
                # compare a case in which they would no longer be nonbinding.
                state = np.asarray(problem['x0'], dtype=float) / np.asarray(problem.get('state_scale', [1., 1.]))
                a, b = np.asarray(problem['a']), np.asarray(problem['b']).reshape(2)
                for control in inputs:
                    state = a @ state + b * control
                    if np.max(np.abs(state)) >= 1e17:
                        raise ValueError('State bounds are active; condensed oracle is inapplicable')
                row['acceleration_rad_s2'] = float(inputs[0] * problem['input_scale'])
                row['input_bound_violation_rad_s2'] = float(max(0, np.max(np.abs(inputs))-limit) * problem['input_scale'])
                row['candidate_scaled_kkt_inf'] = kkt_error(h, g, inputs, limit, reference['scale'])
                if reference['valid']:
                    row['acceleration_error_rad_s2'] = abs(row['acceleration_rad_s2'] - row['oracle_acceleration_rad_s2'])
                    row['objective_gap'] = objective(h, g, inputs) - objective(h, g, reference['inputs'])
            rows.append(row)
    # These sums exclude reference construction and setup. They are not full plan timing.
    if {p['axis'] for p in record['axes']} == {'yaw', 'pitch'}:
        for variant in sorted({r['variant'] for r in rows}):
            pair = [r for r in rows if r['variant'] == variant]
            combined.append({'variant': variant, 'solve_ms': sum(r['solve_ms'] for r in pair),
                             'solved': all(r['solved'] for r in pair)})
    return rows, combined


def self_test():
    # Analytic QP solutions, including active constraints and a coupled Hessian.
    for h, g, limit, expected in [
        (np.eye(1)*2, np.array([-1.]), 1., np.array([.5])),
        (np.eye(1)*2, np.array([-4.]), .5, np.array([.5])),
        (np.array([[2., 1.], [1., 2.]]), np.array([-1., 0.]), 2., np.array([2/3, -1/3])),
    ]:
        answer = oracle(h, g, limit)
        assert answer['valid'], answer
        np.testing.assert_allclose(answer['inputs'], expected, atol=1e-7)
    # Direct stage/terminal evaluation must agree with condensation up to the
    # constant state term. Distinct terminal Hessian/reference matrices are intentional.
    problem = {'a': [[1, .01], [0, 1]], 'b': [[0], [.2]], 'x0': [-.04, .02],
               'reference': [[-.03, -.02, -.01], [0., .01, .02]],
               'q': [1, .1], 'r': .03, 'terminal_hessian': [[2, .1], [.1, 1]],
               'terminal_reference_weight': [[2.05, .1], [.1, 1.05]]}
    h, g = condense(problem)
    def direct(controls):
        state = np.asarray(problem['x0'])
        cost = 0.
        for k in range(3):
            terminal = k == 2
            w = np.asarray(problem['terminal_hessian']) if terminal else np.diag(problem['q'])
            ref_weight = np.asarray(problem['terminal_reference_weight']) if terminal else w
            target = np.asarray(problem['reference'])[:, k]
            cost += .5*state@w@state - target@ref_weight@state
            if not terminal:
                cost += .5*problem['r']*controls[k]**2
                state = np.asarray(problem['a'])@state + np.asarray(problem['b']).reshape(2)*controls[k]
        return cost
    for controls in [np.array([.2, -.3]), np.array([-.5, .1])]:
        np.testing.assert_allclose(objective(h, g, controls), direct(controls)-direct(np.zeros(2)), atol=1e-12)
    # Equivalent state coordinates must produce the same condensed physical QP.
    normalized = dict(problem)
    scales = np.diag([.02, .5])
    inverse = np.linalg.inv(scales)
    normalized.update(state_scale=[.02, .5],
                      a=(inverse@np.asarray(problem['a'])@scales).tolist(),
                      b=(inverse@np.asarray(problem['b'])).tolist(),
                      q=(np.asarray(problem['q'])*np.diag(scales)**2).tolist(),
                      terminal_hessian=(scales@np.asarray(problem['terminal_hessian'])@scales).tolist(),
                      terminal_reference_weight=(scales@np.asarray(problem['terminal_reference_weight'])@scales).tolist())
    hn, gn = condense(normalized)
    np.testing.assert_allclose(hn, h, atol=1e-12)
    np.testing.assert_allclose(gn, g, atol=1e-12)
    # A failed candidate must retain an unavailable error, never become a zero-error sample.
    problem.update(axis='pitch', riccati_cache_residual=0., input_scale=20., normalized_input_limit=.5,
                   candidates=[{'variant': 'failed', 'config': {'primal_tolerance': 1e-5, 'max_iterations': 1},
                                'diagnostic': {'solved': False, 'status': 'iteration_limit', 'iterations': 1,
                                               'solve_time_ms': .01}, 'normalized_inputs': None}])
    rows, combined = evaluate_record({'precision_schema': 1, 'synthetic': True, 'session': 'test',
                                     'plan_id': 1, 'recorded_planning_ms': None, 'axes': [problem]})
    assert rows[0]['acceleration_error_rad_s2'] is None
    assert summarize(rows)[0]['accuracy_samples'] == 0 and not combined
    # Failed oracle results cannot create valid accuracy samples, even when the
    # production candidate succeeded. This is distinct from a failed candidate.
    problem['candidates'][0]['diagnostic']['solved'] = True
    problem['candidates'][0]['normalized_inputs'] = [[0., 0.]]
    invalid = {'valid': False, 'status': 'maximum iterations reached', 'inputs': None,
               'kkt': None, 'bound_violation': None, 'ms': .1, 'scale': 1.}
    with mock.patch.dict(globals(), oracle=lambda *args: invalid):
        unavailable, _ = evaluate_record({'precision_schema': 1, 'synthetic': True,
                                         'session': 'test', 'plan_id': 2,
                                         'recorded_planning_ms': None, 'axes': [problem]})
    assert unavailable[0]['acceleration_error_rad_s2'] is None
    assert summarize(unavailable)[0]['oracle_failures'] == 1
    print('PASS analytic oracle, terminal cost condensation, failed-candidate/oracle accounting')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trace', nargs='?', help='Original planner trace JSONL; never modified')
    parser.add_argument('--replay', default='build-ninja/planner_replay_test')
    parser.add_argument('--latest', action='store_true', help='Use the newest logs/planner_trace/*.jsonl')
    parser.add_argument('--output', help='New output directory; defaults to a unique logs/planner_precision run')
    parser.add_argument('--synthetic', action='store_true', help='Use explicitly labelled replay fixtures')
    parser.add_argument('--self-test', action='store_true')
    args = parser.parse_args()
    if args.self_test:
        self_test()
        return 0
    if sum(bool(value) for value in [args.trace, args.latest, args.synthetic]) != 1:
        parser.error('请选择一个输入：TRACE、--latest 或 --synthetic；--output 可省略')
    if args.latest:
        captures = [p for p in Path('logs/planner_trace').glob('*.jsonl') if p.is_file()]
        if not captures:
            parser.error('未找到 logs/planner_trace/*.jsonl。请在下一次正常台架测试中运行应用采集完整trace，'
                         '正常退出后再执行 --latest；本工具不会启动硬件。')
        args.trace = str(max(captures, key=lambda p: (p.stat().st_mtime_ns, p.name)))
    if args.trace:
        trace = Path(args.trace).resolve()
        if not trace.is_file():
            parser.error(f'输入文件不存在：{trace}。请使用真实trace文件路径，或用 --latest 选择最新记录；'
                         '尚未采集时需先完成采集。')
        if trace.stat().st_size == 0:
            parser.error(f'输入文件为空：{trace}。请确认已采集规划记录，并在应用正常退出后重试。')
        args.trace = str(trace)
    replay = Path(args.replay).resolve()
    if not replay.is_file():
        parser.error(f'回放程序不存在：{replay}。请先构建 planner_replay_test。')
    stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    destination = Path(args.output) if args.output else Path('logs/planner_precision') / (
        ('synthetic-' if args.synthetic else 'capture-') + stamp)
    for name in ['details.csv', 'summary.json']:
        if (destination/name).exists():
            parser.error(f'{destination/name} already exists; choose a new output directory')
    destination.mkdir(parents=True, exist_ok=True)
    if args.trace:
        print(f'Trace: {args.trace}', file=sys.stderr)
    command = [str(replay)]
    command += ['--precision-self-test'] if args.synthetic else [args.trace, '--mode=precision']
    rows, combined = [], []
    total = None
    if args.trace:
        with Path(args.trace).open('rb') as stream:
            total = sum(bool(line.strip()) for line in stream)
    processed = 0
    started = last_progress = time.monotonic()
    count = f'0/{total}' if total is not None else '0'
    print(f'进度：{count} 条规划记录；开始回放和独立QP对照，Ctrl+C 可取消。', file=sys.stderr, flush=True)
    # Keep the exporter out of the terminal's process group. Ctrl+C is handled
    # here (or reported by OSQP); cleanup then terminates and reaps the exporter.
    with subprocess.Popen(command, stdout=subprocess.PIPE, text=True,
                          start_new_session=True) as process:
        try:
            for line in process.stdout:
                try:
                    record = json.loads(line)
                except json.JSONDecodeError as error:
                    raise ValueError(f'回放程序输出第{processed + 1}行JSON不完整：{error.msg}') from None
                detail, timing = evaluate_record(record)
                rows.extend(detail)
                combined.extend(timing)
                processed += 1
                now = time.monotonic()
                if processed == 1 or now - last_progress >= 2 or processed == total:
                    count = f'{processed}/{total}' if total is not None else str(processed)
                    print(f'进度：{count} 条规划记录，{len(rows)} 条候选对照，已用 {now-started:.1f}s。',
                          file=sys.stderr, flush=True)
                    last_progress = now
        except BaseException:
            process.terminate()
            process.wait()
            raise
        if process.wait() != 0:
            raise RuntimeError('回放失败，未生成完整评估；请查看上方回放程序的错误信息。')
    if not rows:
        raise ValueError('No attempted axis problems in trace')
    with (destination/'details.csv').open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    combined_summary = []
    for variant in sorted({r['variant'] for r in combined}):
        selected = [r for r in combined if r['variant'] == variant]
        times = [r['solve_ms'] for r in selected]
        combined_summary.append({'variant': variant, 'samples': len(times),
                                 'solved': sum(r['solved'] for r in selected),
                                 'p99_solve_ms': percentile(times, 99), 'max_solve_ms': max(times),
                                 'over_2ms': sum(t > 2 for t in times),
                                 'over_10ms': sum(t > 10 for t in times)})
    summary = {'synthetic': args.synthetic, 'source_trace': args.trace,
               'timing_scope': 'axis solve only; combined=sum(yaw,pitch), excluding setup/reference generation',
               'reference_scope': 'independent QP including terminal ADMM correction; finite Riccati cache residual is exported',
               'filters': {'small_error_deg': .5, 'low_speed_rad_s': .05, 'horizon_jump_deg': 1.},
               'oracle_failures': len({(r['session'], r['plan_id'], r['axis']) for r in rows if not r['oracle_valid']}),
               'axes': summarize(rows), 'combined_solve': combined_summary}
    (destination/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False)+'\n')
    print(json.dumps({'output': str(destination), 'rows': len(rows),
                      'synthetic': args.synthetic, 'oracle_failures': summary['oracle_failures']}))
    return 2 if summary['oracle_failures'] else 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print('评估已取消；未完成的运行不作为评估结果。', file=sys.stderr)
        sys.exit(130)
    except (OSError, ValueError, RuntimeError) as error:
        print(f'评估失败：{error}', file=sys.stderr)
        sys.exit(2)

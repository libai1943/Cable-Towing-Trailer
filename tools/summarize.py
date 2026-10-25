"""Publication tables computed from saved outcomes; never substitute solver flags for validation."""
from pathlib import Path
import json, statistics as st, csv, sys
import numpy as np
r = Path(__file__).resolve().parents[1]
base = r / 'results/warehouse_v6'
out = r / 'paper/tables'
out.mkdir(exist_ok=True)

def read(p):
    return json.loads(p.read_text(encoding='utf-8-sig'))
seeds = {c: read(base / f'seeds/W{c:02d}.json') for c in range(1, 10)}

def table(name, caption, label, cols, header, rows, wide=False, placement='!t'):
    assert len(caption.split()) <= 20
    env = 'table*' if wide else 'table'
    s = '\\begin{' + env + '}[' + placement + ']\n\\centering\\caption{' + caption + '}\n\\label{' + label + '}\n\\footnotesize\\begin{tabular}{' + cols + '}\n\\toprule\n' + header + '\\\\\n\\midrule\n' + '\n'.join(rows) + '\n\\bottomrule\\end{tabular}\n\\end{' + env + '}\n'
    (out / name).write_text(s, encoding='utf-8')
table('cases.tex', 'Common warehouse layouts and independently verified initial trajectories', 'tab:cases', 'lrrr', 'Case & $N$ & Objects & $d_{\\min}$ (m)', [f"W{c:02d} & {len(d['w']['S'])} & {len(d['P']['obstacles'])} & {d['a']['fineClearance']:.3f}" + '\\\\' for c, d in seeds.items()])
if '--cases-only' in sys.argv:
    sys.exit()
cfg = read(r / 'research/warehouse_v6/final_method.json')
proposed = cfg['method']
methods = ['proposed', 'max', 'logic', 'complementarity', 'fixed', 'portfolio_unique', 'minlp', 'rgr']
labels = ['Proposed', 'MAX', 'LOGIC', 'COMP', 'FIX', 'PORT', 'MINLP', 'RGR']

def rec(c, m, group=None):
    if m == 'proposed':
        m = proposed
        group = group or cfg['group']
    elif m == 'portfolio_unique':
        group = group or 'unique'
    else:
        group = group or 'main'
    return read(base / f'{group}/confined_{group}_W{c:02d}_{m}.json')

def stats(ds, ids):
    return dict(new=sum((d['rawVerified'] for d in ds)), nlp=sum((d.get('rawSolverAccepted', False) for d in ds)), retained=sum((d['retainedVerified'] for d in ds)), improved=sum((d['improved'] for d in ds)), materialImprovement=sum((d['retainedCost'] <= 0.99 * seeds[c]['w']['cost'] for c, d in zip(ids, ds))), meanObjectiveReduction=st.mean((100 * (1 - d['retainedCost'] / seeds[c]['w']['cost']) for c, d in zip(ids, ds))), meanTravelReduction=st.mean((100 * (1 - d['retainedTravel'] / seeds[c]['w']['travel']) for c, d in zip(ids, ds))), medianSeconds=st.median((d['totalSeconds'] for d in ds)), p90Seconds=float(np.quantile([d['totalSeconds'] for d in ds], 0.9)))
ids = list(range(1, 10))
lookup = {(c, m): rec(c, m) for c in ids for m in methods}
case_rows = []
for c in ids:
    d = seeds[c]
    q = lookup[c, 'proposed']
    aa = q['stageAudits']
    aa = aa if isinstance(aa, list) else [aa]
    valid = [a for a in aa if isinstance(a, dict) and a.get('accepted', False)]
    chosen = min(valid, key=lambda a: a['cost']) if q['improved'] else d['a']
    case_rows.append(f"W{c:02d} & {len(d['w']['S'])} & {len(d['P']['obstacles'])} & {d['a']['fineClearance']:.3f} & {chosen['fineClearance']:.3f}" + '\\\\')
table('cases.tex', 'Warehouse layouts, meshes, and minimum replayed body clearances', 'tab:cases', 'lrrrr', 'Case & $N$ & Objects & Seed (m) & Proposed (m)', case_rows)
summaries = [dict(method=m, label=l, **stats([lookup[c, m] for c in ids], ids)) for m, l in zip(methods, labels)]
table('main.tex', 'Common-allowance comparison on nine warehouse tasks', 'tab:main', 'lrrrrrrr', 'Method & New & NLP & $\\Delta J\\geq1\\%$ & $\\Delta J$ (\\%) & $\\Delta l$ (\\%) & $t_{50}$ (s) & $t_{90}$ (s)', [f"{s['label']} & {s['new']}/9 & {s['nlp']}/9 & {s['materialImprovement']}/9 & {s['meanObjectiveReduction']:.2f} & {s['meanTravelReduction']:.2f} & {s['medianSeconds']:.2f} & {s['p90Seconds']:.2f}" + '\\\\' for s in summaries], True)
rows = []
for c in ids:
    values = [f"{lookup[c, m]['retainedCost']:.2f}" + ('' if lookup[c, m]['improved'] else '$^*$') for m in methods]
    rows.append(f'W{c:02d} & ' + ' & '.join(values) + '\\\\')
table('per_case.tex', 'Retained objectives; asterisks mark no improvement over the common seed', 'tab:per-case', 'l' + 'r' * 8, 'Case & ' + ' & '.join(labels), rows, True)
g, m = summaries[:2]
wins = sum((lookup[c, 'proposed']['retainedCost'] < lookup[c, 'max']['retainedCost'] - 1e-05 for c in ids))
best = sum((lookup[c, 'proposed']['retainedCost'] <= min((lookup[c, q]['retainedCost'] for q in methods)) + 1e-05 for c in ids))
faster = sum((lookup[c, 'proposed']['totalSeconds'] < lookup[c, 'max']['totalSeconds'] for c in ids))
numbers = {'ProposedNew': str(g['new']) + '/9', 'ProposedGain': f"{g['meanTravelReduction']:.2f}", 'MaxNew': str(m['new']) + '/9', 'MaxGain': f"{m['meanTravelReduction']:.2f}", 'ProposedTime': f"{g['medianSeconds']:.2f}", 'MaxTime': f"{m['medianSeconds']:.2f}"}
(out / 'numbers.tex').write_text(''.join(('\\newcommand{\\' + k + '}{' + v + '}\n' for k, v in numbers.items())))
if '--primary-only' in sys.argv:
    print(json.dumps(summaries, indent=2))
    sys.exit()
abl = []

def add_abl(label, ds):
    abl.append(dict(label=label, **stats(ds, ids)))
add_abl('Proposed', [lookup[c, 'proposed'] for c in ids])
prefix = []
for c in ids:
    d = lookup[c, 'proposed']['prefix']
    prefix.append(dict(d, totalSeconds=d['seconds']))
add_abl('No final recovery (paired)', prefix)
for meth, label, group in [('no_terminal_' + proposed, 'No terminal correction (paired)', 'paired'), ('rgr', 'RGR', 'main'), ('rrp', 'No reserves (RRP)', 'ablations'), ('max_recover_priority', 'MAX + recovery', 'ablations')]:
    add_abl(label, [rec(c, meth, group) for c in ids])
first = []
for c in ids:
    aa = lookup[c, 'proposed']['stageAudits']
    a = aa[0] if isinstance(aa, list) else aa
    seed = seeds[c]['w']
    a = a if isinstance(a, dict) else {}
    accepted = a.get('accepted', False)
    improved = accepted and a['cost'] < seed['cost'] - 1e-05
    first.append(dict(rawVerified=accepted, rawSolverAccepted=accepted, retainedVerified=True, improved=improved, retainedCost=a['cost'] if improved else seed['cost'], retainedTravel=a['travel'] if improved else seed['travel'], totalSeconds=0))
add_abl('Relaxed stage only', first)
table('ablation.tex', 'Control-recovery ablations and solve-only strategy comparisons', 'tab:ablation', 'lrrrr', 'Variant & New & Impr. & $\\Delta l$ (\\%) & $t_{50}$ (s)', [f"{s['label']} & {s['new']}/9 & {s['improved']}/9 & {s['meanTravelReduction']:.2f} & " + ('--' if s['label'] in ['Relaxed stage only', 'No terminal correction (paired)'] else f"{s['medianSeconds']:.2f}") + '\\\\' for s in abl], True)
recovered = sum((lookup[c, 'proposed'].get('recoverySelected', False) for c in ids))
factorial = []
for index, (meth, group, initial, reserve) in enumerate([('recover_feasible', 'final', 'On', 'Capped'), ('fi_no_cap', 'matched', 'On', 'Uncapped'), ('fi_no_reserve', 'matched', 'On', 'Off'), ('fi_no_initial', 'matched', 'Off', 'Capped'), ('fx_no_initial_no_cap', 'factorial', 'Off', 'Uncapped'), ('fx_no_initial_no_reserve', 'factorial', 'Off', 'Off')], 1):
    factorial.append(dict(id='F' + str(index), method=meth, initialRestoration=initial, reserve=reserve, **stats([rec(c, meth, group) for c in ids], ids)))
table('factorial.tex', 'Factorial comparison of initial restoration and three reserve settings', 'tab:factorial', 'lllrrrr', 'Variant & Initial restoration & Reserves & New & $\\Delta J$ (\\%) & $\\Delta l$ (\\%) & $t_{50}$ (s)', [f"{s['id']} & {s['initialRestoration']} & {s['reserve']} & {s['new']}/9 & {s['meanObjectiveReduction']:.2f} & {s['meanTravelReduction']:.2f} & {s['medianSeconds']:.2f}" + '\\\\' for s in factorial], True)
selected = next((s for s in factorial if s['method'] == proposed))
rd = [rec(c, 'rigid', 'rigid') for c in ids]
rigid_new = sum((d['rawVerified'] for d in rd))
repeat_rows = []
repeat_stats = []
for c in [2, 5, 9]:
    for meth, label in [('proposed', 'Proposed'), ('max', 'MAX')]:
        ds = [lookup[c, meth]] + [rec(c, meth, f'repeat{j}') for j in range(1, 4)]
        times = [d['totalSeconds'] for d in ds]
        costs = [d['retainedCost'] for d in ds]
        row = dict(case=c, method=label, new=sum((d['rawVerified'] for d in ds)), minCost=min(costs), maxCost=max(costs), medianSeconds=st.median(times), iqr=float(np.quantile(times, 0.75) - np.quantile(times, 0.25)))
        repeat_stats.append(row)
        repeat_rows.append(f"W{c:02d} & {label} & {row['new']}/4 & {min(costs):.2f}--{max(costs):.2f} & {row['medianSeconds']:.2f} & {row['iqr']:.2f}" + '\\\\')
table('repeat.tex', 'Four-run sensitivity to numerical termination and process timing', 'tab:repeat', 'llrrrr', 'Case & Method & New & Objective range & $t_{50}$ (s) & IQR (s)', repeat_rows, True)
smooth = []
for beta in [100, 1000]:
    ds = [read(base / f'smoothing/confined_smoothing_W{c:02d}_lse{beta}.json') for c in [2, 5, 9]]
    smooth.append(dict(beta=beta, new=sum((d['rawVerified'] for d in ds)), improved=sum((d['improved'] for d in ds))))
shape_rows = []
shape_data = []
for i in range(1, 6):
    seed = read(base / f'shape_seeds/S{i:02d}.json')
    p = seed['P']
    a = read(base / f'shapes/confined_shapes_S{i:02d}_{proposed}.json')
    b = read(base / f'shapes/confined_shapes_S{i:02d}_max.json')
    shape_data.append(dict(shape=i, map=p['shapeMap'], name=p['shapeName'], proposed=a['retainedCost'], max=b['retainedCost'], newProposed=a['rawVerified'], newMAX=b['rawVerified']))
    vertices = np.asarray(p['vertices'][1])
    length, width = np.ptp(vertices, axis=0)
    shape_rows.append(f"S{i:02d} & ${length:.2f}\\times{width:.2f}$ & {int(a['rawVerified'])}/{int(b['rawVerified'])} & {a['retainedCost']:.2f} & {b['retainedCost']:.2f}" + '\\\\')
table('shapes.tex', 'Five cart footprints on W02', 'tab:shapes', 'llrrr', 'Case & Extents (m) & New & $J_{\\rm prop}$ & $J_{\\rm MAX}$', shape_rows, placement='!tb')
train = read(base / 'train/three_trailers.json')
assert train['R']['accepted']
budget_rows = []
for budget in [15, 30, 60]:
    for meth, label in [('proposed', 'Proposed'), ('max', 'MAX'), ('minlp', 'MINLP')]:
        actual = proposed if meth == 'proposed' else meth
        ds = [lookup[c, meth] if budget == 30 else read(base / f'budgets/confined_budget{budget}_W{c:02d}_{actual}.json') for c in [2, 5, 9]]
        budget_rows.append(dict(budget=budget, method=label, **stats(ds, [2, 5, 9])))
lowp = next((x for x in budget_rows if x['budget'] == 15 and x['method'] == 'Proposed'))
lowm = next((x for x in budget_rows if x['budget'] == 15 and x['method'] == 'MAX'))
highp = next((x for x in budget_rows if x['budget'] == 60 and x['method'] == 'Proposed'))
highm = next((x for x in budget_rows if x['budget'] == 60 and x['method'] == 'MAX'))
summary = dict(main=summaries, proposedMethod=proposed, betterThanMAX=wins, bestMain=best, fasterThanMAX=faster, ablation=abl, factorial=factorial, repeatedRuns=repeat_stats, smoothing=smooth, shapes=shape_data, budgets=budget_rows)
(base / 'summary.json').write_text(json.dumps(summary, indent=2))
with (base / 'comparison_summary.csv').open('w', newline='') as f:
    w = csv.DictWriter(f, fieldnames=list(summaries[0]))
    w.writeheader()
    w.writerows(summaries)
with (base / 'all_primary_cases.csv').open('w', newline='') as f:
    fields = ['case', 'method', 'new', 'nlp', 'improved', 'retainedCost', 'retainedTravel', 'totalSeconds']
    w = csv.DictWriter(f, fieldnames=fields)
    w.writeheader()
    for c in ids:
        for meth, label in zip(methods, labels):
            d = lookup[c, meth]
            w.writerow(dict(case=f'W{c:02d}', method=label, new=d['rawVerified'], nlp=d['rawSolverAccepted'], improved=d['improved'], **{k: d[k] for k in fields[-3:]}))
print(json.dumps(summary['main'], indent=2))
"""Verify distributed hashes and the archived numerical summary without solvers."""
from pathlib import Path
import hashlib,json,math,re
r=Path(__file__).resolve().parents[1]
manifest=json.loads((r/'MANIFEST.json').read_text())
for item in manifest:
    p=r/item['path'];assert p.is_file(),item['path']
    assert hashlib.sha256(p.read_bytes()).hexdigest()==item['sha256'],item['path']
read=lambda p:json.loads(p.read_text(encoding='utf-8-sig'))
summary=read(r/'results/warehouse_v6/summary.json')
proposed,maxrow=summary['main'][:2]
assert (proposed['new'],maxrow['new'])==(9,8)
assert abs(proposed['meanTravelReduction']-4.359387)<1e-5
assert abs(maxrow['meanTravelReduction']-.972376)<1e-5
records=read(r/'results/record_index.json');assert len(records)==214
audit=read(r/'results/warehouse_v6/verification.json');assert audit['passed'] and audit['recordCount']==214
errors=read(r/'results/terminal_errors.json')
assert len(errors)==9 and all(e['accepted'] for e in errors)
assert max(e['fine_position_m'] for e in errors)<.1551
tex=(r/'paper/main.tex').read_text()
assert 'Optimization-Based Motion Planning for Autonomous Cable-Towing Vehicles in Confined Warehouses' in tex
assert 'pdfauthor={}' in tex
print(json.dumps({'passed':True,'hashedFiles':len(manifest),'optimizationRecords':len(records),
 'maxFinePositionError_m':max(e['fine_position_m'] for e in errors),
 'maxFineHeadingError_deg':math.degrees(max(e['fine_heading_rad'] for e in errors))},indent=2))

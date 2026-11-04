#!/usr/bin/env python3
"""Render measured simulation trajectories; standard library only."""
import csv
import html
from pathlib import Path
import sys

root = Path(sys.argv[1])
colors = {"navigation": "#157fbc", "supply": "#18a577"}
parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="860" viewBox="0 0 1000 860">',
         '<rect width="1000" height="860" fill="#fafafa"/>',
         '<text x="50" y="35" font-family="sans-serif" font-size="22">pure-nav — measured simulation trajectories</text>']
def rectangle(x, y, w, h):
    return f'<rect x="{50+x*90}" y="{780-(y+h)*90}" width="{w*90}" height="{h*90}" fill="#39434d"/>'
for rect in [(0,0,10,.1),(0,7.9,10,.1),(0,0,.1,8),(9.9,0,.1,8),(4.8,0,.1,2.6),(4.8,4.7,.1,3.3),(6.5,5.1,1,1)]:
    parts.append(rectangle(*rect))
for index, (scenario, color) in enumerate(colors.items()):
    with (root / f"{scenario}.csv").open() as stream:
        rows = list(csv.DictReader(stream))
    for prefix, dash in [("true", ""), ("estimate", ' stroke-dasharray="4 4"')]:
        points = ' '.join(f'{50+float(r[prefix+"_x"])*90:.2f},{780-float(r[prefix+"_y"])*90:.2f}' for r in rows)
        parts.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="2"{dash}/>')
    log = html.escape((root / f"{scenario}.log").read_text().strip().split(' trajectory=')[0])
    parts.append(f'<text x="50" y="{810+index*24}" font-family="monospace" font-size="11" fill="{color}">{log}</text>')
parts.append('</svg>')
(root / 'trajectories.svg').write_text('\n'.join(parts))
logs = '\n'.join((root / f'{name}.log').read_text() for name in ['navigation','supply','blocked','sensor-loss','map'])
(root / 'index.html').write_text('<!doctype html><meta charset="utf-8"><title>pure-nav acceptance</title><h1>pure-nav 验收记录</h1><p>实测二维仿真闭环。实线为仿真真值，虚线为定位估计；定位模块不读取真值。</p><img src="trajectories.svg" style="max-width:100%"><pre>'+html.escape(logs)+'</pre><p>完整迁移范围和未完成项见 assets/docs/MIGRATION.md。此报告不代表 Gazebo、Point-LIO 或实车验证。</p>')
print(f'Report: {root / "index.html"}')

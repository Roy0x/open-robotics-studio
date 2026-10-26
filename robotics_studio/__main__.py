"""Run with python -m robotics_studio --x 0.6 --y 0.3."""
import argparse
import json
import math
from pathlib import Path
from .arm import Arm


def svg(result):
    reach = sum(result['links_m'])
    scale = 220 / max(reach, math.hypot(*result['target_m']), 0.01)
    def point(p):
        return f'{300+p[0]*scale:.3f},{300-p[1]*scale:.3f}'
    parts = ['<svg xmlns="http://www.w3.org/2000/svg" width="600" height="650" viewBox="0 0 600 650">',
             '<rect width="600" height="650" fill="#f8fafc"/>',
             f'<circle cx="300" cy="300" r="{reach*scale}" fill="none" stroke="#cbd5e1"/>',
             '<path d="M30 300H570 M300 30V570" stroke="#cbd5e1"/>']
    for i, solution in enumerate(result['solutions']):
        color = ('#2563eb', '#d97706')[i % 2]
        points = ' '.join(point(p) for p in ((0,0), solution['elbow_m'], solution['tip_m']))
        parts.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="7" opacity="0.7"/>')
    tx, ty = point(result['target_m']).split(',')
    parts += [f'<circle cx="{tx}" cy="{ty}" r="7" fill="#dc2626"/>',
              '<circle cx="300" cy="300" r="7" fill="#0f172a"/>',
              f'<text x="25" y="600" font-family="sans-serif" font-size="20">{result["status"]} — {len(result["solutions"])} valid poses</text>',
              '<text x="25" y="630" font-family="sans-serif" font-size="14">Red: target | Blue/orange: poses | +X right, +Y up</text></svg>']
    return '\n'.join(parts)


def main():
    parser = argparse.ArgumentParser(description='Analyze static reachability of a planar two-link arm')
    parser.add_argument('--x', type=float, default=0.6)
    parser.add_argument('--y', type=float, default=0.3)
    parser.add_argument('--links', nargs=2, type=float, default=[0.6, 0.4], metavar=('L1', 'L2'))
    parser.add_argument('--limits-deg', nargs=4, type=float, default=[-180,180,-180,180], metavar=('Q1_MIN','Q1_MAX','Q2_MIN','Q2_MAX'))
    parser.add_argument('--out', type=Path, default=Path('artifacts/reach'))
    args = parser.parse_args()
    try:
        limits = tuple(tuple(math.radians(v) for v in args.limits_deg[i:i+2]) for i in (0,2))
        result = Arm(*args.links, limits=limits).solve(args.x, args.y)
    except ValueError as exc:
        parser.error(str(exc))
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out/'result.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    (args.out/'scene.svg').write_text(svg(result), encoding='utf-8')
    print(f'{result["status"]}: {len(result["solutions"])} valid poses')
    print(f'Outputs: {args.out / "result.json"} and {args.out / "scene.svg"}')
    return 0 if result['status'] == 'reachable' else 1


if __name__ == '__main__':
    raise SystemExit(main())

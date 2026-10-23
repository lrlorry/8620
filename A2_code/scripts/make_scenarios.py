"""Create regression cases, the 3x3x3 design, generated layouts, controls and a development set."""

from itertools import product
import json
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]


def config(grid=(12,8,4), starts=((0,1,1),), goals=((11,1,1),),
           obstacles=(), altitude=True, alpha=0.25, horizon=80):
    return dict(grid_size=list(grid), start_positions=[list(p) for p in starts],
                goal_positions=[list(p) for p in goals], obstacle_cells=[list(p) for p in obstacles],
                change_altitude=altitude, step_cost=-1., collision_penalty=-100.,
                goal_reward=100., discount_factor=0.98, max_num_steps=horizon, alpha=alpha)


def main():
    folder = ROOT/'environments'
    folder.mkdir(exist_ok=True)
    cases = {
        'demo_open_2d': config((12,8,1), [(0,2,0)], [(11,5,0)], altitude=False, alpha=.15),
        'demo_four_3d': config(starts=[(0,1,1),(0,3,1),(0,5,2),(0,6,2)],
                              goals=[(11,1,2),(11,3,2),(11,5,1),(11,6,1)],
                              obstacles=[(4,1,1),(5,3,1),(6,5,2),(7,6,2),(6,4,1)], alpha=.2),
        'already_at_goals': config((3,3,1), [(0,0,0),(2,2,0)], [(0,0,0),(2,2,0)], altitude=False),
        'single_cell_goal': config((1,1,1), [(0,0,0)], [(0,0,0)], altitude=False),
        'one_reached': config((9,5,1), [(4,2,0),(0,2,0)], [(4,2,0),(8,2,0)], altitude=False, alpha=.1),
        'swap_2d': config((8,5,1), [(1,2,0),(6,2,0)], [(6,2,0),(1,2,0)], altitude=False, alpha=.1),
        'boundary_reverse': config((6,4,1), [(5,3,0)], [(0,0,0)], altitude=False, alpha=.2),
        'tiny_2d': config((2,2,1), [(0,0,0)], [(1,1,0)], altitude=False),
        'thin_1d': config((8,1,1), [(0,0,0)], [(7,0,0)], altitude=False, alpha=.1),
        'infeasible_altitude': config((6,4,2), [(0,1,0)], [(5,1,1)], altitude=False),
        'infeasible_wall': config((6,3,1), [(0,1,0)], [(5,1,0)],
                                  [(3,y,0) for y in range(3)], altitude=False),
        'infeasible_shared_goal': config((6,4,1), [(0,0,0),(0,3,0)], [(5,2,0),(5,2,0)], altitude=False),
        'horizon_one': config((8,5,1), [(0,2,0)], [(7,2,0)], altitude=False, horizon=1),
        'narrow_passage': config((12,7,1), [(0,1,0),(0,5,0)], [(11,5,0),(11,1,0)],
                                 [(5,y,0) for y in range(7) if y != 3], altitude=False, alpha=.05),
        'altitude_detour': config((10,5,4), [(0,2,0)], [(9,2,0)],
                                  [(4,y,z) for y in range(5) for z in (0,1)], alpha=.1),
        'strong_wind': config((10,5,3), [(0,2,1)], [(9,2,1)], [(4,2,1),(6,1,1)], alpha=3.),
        'three_3d': config((10,7,3), [(0,1,0),(0,3,1),(0,5,2)],
                           [(9,5,2),(9,3,1),(9,1,0)], [(4,3,1),(5,2,0),(6,4,2)], alpha=.15),
        'four_2d': config((10,8,1), [(0,0,0),(0,2,0),(0,5,0),(0,7,0)],
                          [(9,0,0),(9,2,0),(9,5,0),(9,7,0)], [(5,2,0),(4,5,0)], altitude=False, alpha=.1),
    }
    manifest = []
    for name, data in cases.items():
        path = folder/f'{name}.yaml'
        path.write_text(yaml.safe_dump(data, sort_keys=False))
        manifest.append(dict(name=name, suite='regression', path=str(path.relative_to(ROOT)), mask='full'))

    nested = [(4,1,1),(7,3,1),(6,2,1),(8,1,1),(3,3,1)]
    for height, (mask, directions), clutter in product([5,10,20], [('axial',6),('face_edge',18),('full',26)], [0,2,5]):
        name = f'factor_S{height}_A{directions}_C{clutter}'
        data = config((12,height,3), [(0,1,1),(0,3,1)], [(11,1,1),(11,3,1)],
                      nested[:clutter], alpha=.25, horizon=60)
        path = folder/f'{name}.yaml'
        path.write_text(yaml.safe_dump(data, sort_keys=False))
        free = 12*height*3-clutter
        manifest.append(dict(name=name, suite='factorial', path=str(path.relative_to(ROOT)),
                             mask=mask, S=height, A=directions, C=clutter,
                             free_cells=free, positional_state_bound=str(free**2), joint_actions=directions**2))

    # Generated layouts: drone count and noise are crossed (each pair once), not confounded.
    rng = np.random.default_rng(8620)
    for index in range(16):
        count, alpha = index % 4 + 1, [.05,.25,1.,2.][index // 4]
        yz = rng.choice(18, size=2*count, replace=False)
        starts = [(0,int(v//3),int(v%3)) for v in yz[:count]]
        goals = [(7,int(v//3),int(v%3)) for v in yz[count:]]
        obstacles = sorted({(int(rng.integers(2,6)), int(rng.integers(6)), int(rng.integers(3)))
                            for _ in range(int(rng.integers(0,6)))})
        name = f'random_{index:02d}_N{count}'
        data = config((8,6,3), starts, goals, obstacles, alpha=alpha, horizon=60)
        path = folder/f'{name}.yaml'
        path.write_text(yaml.safe_dump(data, sort_keys=False))
        manifest.append(dict(name=name, suite='random', path=str(path.relative_to(ROOT)), mask='full',
                             N=count, alpha=alpha, C=len(obstacles)))

    # Mechanism controls around the factorial S10 condition.
    for clutter in (0,2,5):
        name = f'lownoise_C{clutter}'
        data = config((12,10,3), [(0,1,1),(0,3,1)], [(11,1,1),(11,3,1)],
                      nested[:clutter], alpha=.01, horizon=60)
        path = folder/f'{name}.yaml'
        path.write_text(yaml.safe_dump(data, sort_keys=False))
        manifest.append(dict(name=name, suite='control', path=str(path.relative_to(ROOT)), mask='full',
                             S=10, A=26, C=clutter, alpha=.01))
    for clutter in (0,2,5):
        manifest.append(dict(name=f'lownoise_A6_C{clutter}', suite='control', mask='axial',
                             path=f'environments/lownoise_C{clutter}.yaml', S=10, A=6, C=clutter, alpha=.01))
    for copies in (2,4):
        manifest.append(dict(name=f'branch_x{copies}', suite='control', mask=f'dup{copies}',
                             path='environments/factor_S10_A26_C2.yaml', S=10, A=26*copies, C=2, alpha=.25))

    # Development layouts for parameter selection only; never used in reported evaluations.
    rng = np.random.default_rng(4620)
    folder_dev = folder/'dev'
    folder_dev.mkdir(exist_ok=True)
    for index in range(16):
        count, alpha = index % 4 + 1, [.25,.5,1.,2.][index // 4]
        altitude = bool((index + index // 4) % 2)
        depth = 5 if altitude else 1
        yz = rng.choice(10*depth, size=2*count, replace=False)
        starts = [(0,int(v//depth),int(v%depth)) for v in yz[:count]]
        goals = [(9,int(v//depth),int(v%depth)) for v in yz[count:]]
        obstacles = sorted({(int(rng.integers(2,8)), int(rng.integers(10)), int(rng.integers(depth)))
                            for _ in range(int(rng.integers(0,6)))})
        name = f'dev_{index:02d}_N{count}'
        data = config((10,10,depth), starts, goals, obstacles, altitude=altitude, alpha=alpha, horizon=60)
        path = folder_dev/f'{name}.yaml'
        path.write_text(yaml.safe_dump(data, sort_keys=False))
        manifest.append(dict(name=name, suite='dev', path=str(path.relative_to(ROOT)), mask='full',
                             N=count, alpha=alpha, C=len(obstacles)))
    (folder/'manifest.json').write_text(json.dumps(manifest, indent=2)+'\n')
    print(f'Wrote {len(manifest)} scenarios')


if __name__ == '__main__':
    main()

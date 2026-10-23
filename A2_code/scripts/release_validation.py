"""Final single-process checks: tests, model verification, default-budget demos, forced deadline."""

import os
for name in ('OMP_NUM_THREADS','MKL_NUM_THREADS','OPENBLAS_NUM_THREADS','NUMEXPR_NUM_THREADS'):
    os.environ[name]='1'

from datetime import date
import json
from pathlib import Path
import platform
import re
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
import matplotlib
import numpy
import scipy
import yaml
from environment import HeadlessMultiDroneUnc
from planner import MCTSPlanner
from run_u8076655 import run_episode

DEMOS=[('demo_open_2d','environments/demo_open_2d.yaml'),('example_config','example_config.yaml'),
       ('strong_wind','environments/strong_wind.yaml'),('demo_four_3d','environments/demo_four_3d.yaml')]


def main():
    with (ROOT/'results/unit_tests.log').open('w') as stream:
        subprocess.run([sys.executable,'-m','unittest','discover','-s','tests','-v'],
                       cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,check=True)
    for script in ('verify_upstream.py','verify_small_model.py'):
        subprocess.run([sys.executable,str(ROOT/'scripts'/script)],cwd=ROOT,check=True)
    demos={}
    for name,path in DEMOS:
        result=run_episode(ROOT/path,seed=0,planning_time=9.5)
        (ROOT/f'results/{name}_default.json').write_text(json.dumps(result,indent=2)+'\n')
        demos[name]={k:v for k,v in result.items() if k not in ('trajectory','searches','final_state')}
        print(name,demos[name],flush=True)
        if result['max_plan_seconds']>10:
            raise AssertionError('A default-budget decision exceeded ten seconds')
        if not result['success']:
            raise AssertionError(f'Default-budget demo {name} did not succeed')
    env=HeadlessMultiDroneUnc(ROOT/'environments/demo_four_3d.yaml',seed=8620)
    state=env.reset()
    planner=MCTSPlanner(env,max_simulations=100000)
    start=time.perf_counter();action=planner.plan(state,10.);elapsed=time.perf_counter()-start
    assert 0 <= action < 26**4
    if elapsed>10:raise AssertionError('The forced deadline check exceeded ten seconds')
    count=int(re.search(r'Ran (\d+) tests', (ROOT/'results/unit_tests.log').read_text()).group(1))
    result=dict(default_demos=demos,forced_10s_check={**planner.last_stats,'measured_elapsed':elapsed},
                unit_tests=count)
    (ROOT/'results/release_validation.json').write_text(json.dumps(result,indent=2)+'\n')
    software=dict(date=str(date.today()),os=platform.platform(terse=True),architecture=platform.machine(),
                  processor=platform.processor(),python=platform.python_version(),numpy=numpy.__version__,
                  scipy=scipy.__version__,pyyaml=yaml.__version__,matplotlib=matplotlib.__version__)
    (ROOT/'results/software_environment.json').write_text(json.dumps(software,indent=2)+'\n')
    print(json.dumps(result,indent=2),flush=True)


if __name__=='__main__':main()

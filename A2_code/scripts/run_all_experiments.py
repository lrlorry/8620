"""Run every reported suite with resumable raw outputs (simulation-capped, parallel episodes)."""

import argparse
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]

if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--workers',type=int,default=1)
    parser.add_argument('--with-tuning',action='store_true',help='Also rerun the development-set parameter study')
    args=parser.parse_args()
    suites=[('budget',20),('control',20),('factorial',20),('components',20),('random',10),('regression',20)]
    if args.with_tuning:
        subprocess.run([sys.executable,str(ROOT/'experiments.py'),'--suite','tuning','--repetitions','10',
                        '--simulations','500','--workers',str(args.workers),
                        '--output','results/dev/tuning.csv','--resume'],cwd=ROOT,check=True)
    for suite,repetitions in suites:
        subprocess.run([sys.executable,str(ROOT/'experiments.py'),'--suite',suite,
                        '--repetitions',str(repetitions),'--simulations','2000','--workers',str(args.workers),
                        '--output',f'results/{suite}.csv','--resume'],cwd=ROOT,check=True)

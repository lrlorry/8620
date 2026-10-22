"""Verify hashes, extract in isolation, and test the actual submission archive."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
from tempfile import TemporaryDirectory
import zipfile


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('archive')
    parser.add_argument('--output')
    args=parser.parse_args()
    archive_path=Path(args.archive).resolve()
    checks={}
    with TemporaryDirectory(prefix='comp8620_submission_') as temporary:
        destination=Path(temporary)
        with zipfile.ZipFile(archive_path) as archive:
            roots={Path(name).parts[0] for name in archive.namelist()}
            assert roots=={'u8076655_A2_code'}
            for name in archive.namelist():
                assert not Path(name).is_absolute() and '..' not in Path(name).parts
            archive.extractall(destination)
        root=destination/'u8076655_A2_code'
        count=0
        for line in (root/'MANIFEST.sha256').read_text().splitlines():
            digest,name=line.split('  ',1)
            assert hashlib.sha256((root/name).read_bytes()).hexdigest()==digest,name
            count+=1
        checks['verified_file_hashes']=count
        required=['run_u8076655.py','planner.py','environment.py','multi_drone.py',
                  'multi_drone_config.py','AI_prompts.txt','environments/manifest.json',
                  'report/u8076655_A2_report.tex','report/partB.tex','results/summary.json']
        assert all((root/name).exists() for name in required)
        env=os.environ.copy()
        env['MPLCONFIGDIR']=str(destination/'mpl_cache')
        env['PYTHONPATH']=''
        command=[sys.executable,str(root/'run_u8076655.py'),'--config',
                 str(root/'environments/thin_1d.yaml'),'--planning-time','0.01','--max-simulations','20']
        result=subprocess.run(command,cwd=root,env=env,capture_output=True,text=True,check=True)
        episode=json.loads(result.stdout)
        assert episode['success'] and episode['termination']=='success'
        checks['isolated_entrypoint_success']=True
        result=subprocess.run([sys.executable,'-m','unittest','discover','-s','tests','-v'],
                              cwd=root,env=env,capture_output=True,text=True,check=True)
        checks['isolated_unit_tests']=result.stderr.strip().splitlines()[-1]
        for script in ['verify_upstream.py','analyse_results.py','finish_report.py']:
            subprocess.run([sys.executable,str(root/'scripts'/script)],cwd=root,env=env,
                           stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,check=True)
        checks['isolated_analysis_and_report_inputs']=True
        subprocess.run(['latexmk','-pdf','-interaction=nonstopmode','-halt-on-error',
                        'u8076655_A2_report.tex'],cwd=root/'report',env=env,
                       stdout=subprocess.PIPE,stderr=subprocess.PIPE,text=True,check=True)
        checks['isolated_report_build']=True
    checks['archive_sha256']=hashlib.sha256(archive_path.read_bytes()).hexdigest()
    if args.output:Path(args.output).write_text(json.dumps(checks,indent=2)+'\n')
    print(json.dumps(checks,indent=2))


if __name__=='__main__':main()

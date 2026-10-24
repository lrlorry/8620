"""Build a single-root submission archive, including raw results and report sources."""

import hashlib
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parents[1]
REPORT=next((p for p in (ROOT/'report',ROOT.parent/'A2')
             if (p/'u8076655_A2_report.tex').exists()),ROOT/'report')


def main():
    files={}
    excluded={'__pycache__','.git','.venv','.pytest_cache','report'}
    for path in sorted(ROOT.rglob('*')):
        relative=path.relative_to(ROOT)
        if not path.is_file() or any(p in excluded for p in relative.parts):continue
        if path.suffix in ('.pyc','.zip') or path.name=='.DS_Store':continue
        if path.name.endswith('_progress.log') or (path.name.startswith('rerun_') and path.suffix=='.log'):continue
        files[relative.as_posix()]=path.read_bytes()
    for path in sorted(REPORT.rglob('*')):
        if path.is_file() and path.suffix in ('.tex','.pdf','.png','.md','.txt'):
            if path.name=='a2.pdf':continue
            files['report/'+path.relative_to(REPORT).as_posix()]=path.read_bytes()
    files['AI_prompts.txt']=(REPORT/'AI_prompts.txt').read_bytes()
    manifest=''.join(f'{hashlib.sha256(data).hexdigest()}  {name}\n' for name,data in sorted(files.items()))
    files['MANIFEST.sha256']=manifest.encode()
    target=REPORT/'u8076655_A2_code.zip'
    with zipfile.ZipFile(target,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=9) as archive:
        for name,data in sorted(files.items()):
            archive.writestr('u8076655_A2_code/'+name,data)
    print(f'{target}: {len(files)} files, {target.stat().st_size:,} bytes')


if __name__=='__main__':main()

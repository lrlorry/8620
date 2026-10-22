"""Verify the vendored simulator core against the archived teacher source."""
import ast
import hashlib
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]


def methods(path):
    tree=ast.parse(path.read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='MultiDroneUnc')
    return {n.name:ast.dump(n,include_attributes=False) for n in cls.body if isinstance(n,ast.FunctionDef)}


def main():
    original=ROOT/'provenance/multi_drone.original.py'
    current=ROOT/'multi_drone.py'
    provenance=json.loads((ROOT/'UPSTREAM.json').read_text())
    assert hashlib.sha256(original.read_bytes()).hexdigest()==provenance['original_sha256']['multi_drone.py']
    first,second=methods(original),methods(current)
    names=['__init__','get_config','num_actions','reset','step','simulate',
           '_transition_dynamics','_load_config','_decode_action','_encode_action']
    result={name:first[name]==second[name] for name in names}
    (ROOT/'results/model_verification.json').write_text(json.dumps(result,indent=2)+'\n')
    if not all(result.values()):raise AssertionError('Simulator core changed')
    print('All ten upstream core methods are unchanged.')


if __name__=='__main__':main()

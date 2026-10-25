"""Verify exact planner sources for all recorded experiment versions."""

import ast
from collections import defaultdict
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def main():
    index = json.loads((ROOT/'provenance/planner_sources.json').read_text())
    sources = {entry['sha256']: entry for entry in index['sources']}
    if len(sources) != len(index['sources']):
        raise ValueError('Duplicate planner hashes in source index')
    observed = defaultdict(list)
    for path in sorted((ROOT/'results').rglob('*.meta.json')):
        digest = json.loads(path.read_text())['planner_sha256']
        if digest not in sources:
            raise ValueError(f'Missing planner source for {path.relative_to(ROOT)}')
        observed[digest].append(str(path.relative_to(ROOT)))
    if set(observed) != set(sources):
        raise ValueError('Source index and experiment versions differ')
    for digest, entry in sources.items():
        source = ROOT/entry['path']
        data = source.read_bytes()
        if hashlib.sha256(data).hexdigest() != digest:
            raise ValueError(f'Planner source hash mismatch: {entry["path"]}')
        ast.parse(data, filename=entry['path'])
        if sorted(entry['used_by']) != observed[digest]:
            raise ValueError(f'Experiment references differ: {entry["path"]}')
    print(f'All {len(sources)} planner versions match '
          f'{sum(map(len, observed.values()))} experiment metadata files.')


if __name__ == '__main__':
    main()

"""Regression checks for correlated seeds and repeated evaluation conditions."""

from copy import deepcopy
import importlib.util
from pathlib import Path
import unittest


path=Path(__file__).resolve().parents[1]/'scripts/analyse_results.py'
spec=importlib.util.spec_from_file_location('analysis',path)
analysis=importlib.util.module_from_spec(spec)
spec.loader.exec_module(analysis)


def episode(seed,score,scenario='case',cap=2000):
    return dict(seed=seed,scenario=scenario,variant='full',simulation_cap=cap,
                success=True,steps=score,penalized_steps=float(score),discounted_return=-score)


class AnalysisTests(unittest.TestCase):
    def test_repeating_conditions_within_seed_does_not_narrow_interval(self):
        pairs=[(episode(s,20),episode(s,20+s-9.5)) for s in range(20)]
        original=analysis.seed_cluster_effect(pairs,'penalized_steps')
        repeated=analysis.seed_cluster_effect(pairs*7,'penalized_steps')
        self.assertEqual(original['mean'],0.)
        self.assertLess(original['lower'],0.)
        self.assertGreater(original['upper'],0.)
        self.assertEqual(original['seed_blocks'],20)
        self.assertAlmostEqual(original['lower'],repeated['lower'])
        self.assertAlmostEqual(original['upper'],repeated['upper'])

    def test_duplicate_suites_count_once_but_distinct_budgets_remain(self):
        pair=(episode(0,10),episode(0,12))
        other=(episode(0,10,cap=64),episode(0,12,cap=64))
        unique=analysis.unique_episode_pairs([pair,deepcopy(pair),other])
        self.assertEqual(len(unique),2)
        self.assertEqual({b['simulation_cap'] for _,b in unique},{64,2000})

    def test_conflicting_duplicate_results_are_rejected(self):
        pair=(episode(0,10),episode(0,12))
        changed=deepcopy(pair)
        changed[1]['success']=False
        with self.assertRaisesRegex(ValueError,'Conflicting duplicate'):
            analysis.unique_episode_pairs([pair,changed])


if __name__=='__main__':
    unittest.main()

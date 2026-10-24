"""Model semantics, interface compliance, and online-planning regressions."""

from copy import deepcopy
from pathlib import Path
from tempfile import TemporaryDirectory
import time
import unittest

import numpy as np
import yaml

from environment import ActionMask, HeadlessMultiDroneUnc
from planner import MCTSPlanner, action_vectors, decode_action, encode_action
from run_u8076655 import run_episode

ROOT = Path(__file__).resolve().parents[1]


def configuration(count=1, altitude=False):
    return dict(grid_size=[5,4,3 if altitude else 1], change_altitude=altitude,
                start_positions=[[1,i+1,0] for i in range(count)],
                goal_positions=[[4,i+1,0] for i in range(count)], obstacle_cells=[],
                step_cost=-1., collision_penalty=-100., goal_reward=100.,
                discount_factor=.98, max_num_steps=30, alpha=.5)


class FixedChoices:
    def __init__(self, choices):
        self.choices = iter(choices)

    def choice(self, count, p):
        chosen = next(self.choices)
        assert p[chosen] > 0
        return chosen


class PublicOnly:
    def __init__(self, env):
        object.__setattr__(self, '_wrapped', env)

    def get_config(self):
        return object.__getattribute__(self, '_wrapped').get_config()

    @property
    def num_actions(self):
        return object.__getattribute__(self, '_wrapped').num_actions

    def simulate(self, state, action):
        return object.__getattribute__(self, '_wrapped').simulate(state, action)

    def __getattr__(self, name):
        raise AssertionError(f'Planner accessed an unapproved simulator attribute: {name}')


class SolverTests(unittest.TestCase):
    def setUp(self):
        self.temp = TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.path = Path(self.temp.name)/'case.yaml'

    def env(self, data=None, seed=0):
        self.path.write_text(yaml.safe_dump(data or configuration()))
        env = HeadlessMultiDroneUnc(self.path, seed=seed)
        state = env.reset()
        return env, state

    def test_encoding_matches_upstream(self):
        for altitude in (False,True):
            env, _ = self.env(configuration(2,altitude))
            base = 26 if altitude else 8
            for a in [0,1,base-1,base,base**2-1,base**2//2]:
                digits = decode_action(a,base,2)
                self.assertEqual(encode_action(digits,base),a)
                self.assertEqual(env._encode_action(np.array(digits)),a)
                np.testing.assert_array_equal(env._decode_action(a),digits)

    def test_negative_action_rejected(self):
        with self.assertRaises(ValueError):
            decode_action(-1,8,2)

    def test_terminal_bandit_selects_known_best_action(self):
        env,state=self.env()
        class TerminalBandit:
            def get_config(self):return env.get_config()
            num_actions=8
            def simulate(self,current,action):
                if current[:,3].all():raise AssertionError('Terminal state was simulated again')
                successor=current.copy();successor[0,:3]=[4,1,0];successor[0,3]=1
                return successor, (100. if action==6 else -100.), True, {}
        planner=MCTSPlanner(TerminalBandit(),max_simulations=128,guided=False)
        self.assertEqual(planner.plan(state,1.),6)
        self.assertEqual(planner.last_stats['simulations'],128)

    def test_discount_endpoints_and_map_fallback(self):
        for gamma in (0.,1.):
            data=configuration();data['discount_factor']=gamma
            env,state=self.env(data)
            planner=MCTSPlanner(env,max_simulations=5,max_map_cells=0)
            self.assertTrue(0 <= planner.plan(state,.2) < 8)
            self.assertTrue(np.isfinite(planner._leaf(state,10)))

    def test_invalid_budget(self):
        env,state=self.env();planner=MCTSPlanner(env)
        for budget in (-1.,float('nan'),float('inf')):
            with self.assertRaises(ValueError):planner.plan(state,budget)

    def test_boundary_slips_renormalize_without_staying(self):
        data=configuration();data['start_positions']=[[0,0,0]]
        env,state=self.env(data)
        samples=[env.simulate(state,0)[0][0,:3] for _ in range(200)]
        self.assertTrue(all(tuple(p) in {(0,1,0),(1,0,0),(1,1,0)} for p in samples))

    def test_obstacle_collision_and_reward(self):
        data=configuration();data['obstacle_cells']=[[2,1,0]]
        env,state=self.env(data);env.rng=FixedChoices([6])
        next_state,reward,done,info=env.simulate(state,6)
        np.testing.assert_array_equal(next_state[0,:3],state[0,:3])
        self.assertEqual(reward,-101.)
        self.assertTrue(done);self.assertEqual(info['num_collisions'],1)

    def test_same_cell_collision_counts_both_drones(self):
        data=configuration(2);data['start_positions']=[[1,1,0],[3,1,0]]
        env,state=self.env(data);env.rng=FixedChoices([6,1])
        _,reward,done,info=env.simulate(state,encode_action([6,1],8))
        self.assertTrue(done);self.assertEqual(info['num_vehicle_collisions'],2)
        self.assertEqual(reward,-202.)

    def test_cell_swap_is_permitted_by_supplied_model(self):
        data=configuration(2);data['start_positions']=[[1,1,0],[2,1,0]]
        env,state=self.env(data);env.rng=FixedChoices([6,1])
        successor,_,done,info=env.simulate(state,encode_action([6,1],8))
        np.testing.assert_array_equal(successor[:,:3],[[2,1,0],[1,1,0]])
        self.assertFalse(done);self.assertEqual(info['num_vehicle_collisions'],0)

    def test_reached_drone_is_frozen_and_solid(self):
        data=configuration(2);data['start_positions']=[[2,1,0],[1,1,0]]
        data['goal_positions']=[[2,1,0],[4,2,0]]
        env,state=self.env(data);env.rng=FixedChoices([6,6])
        successor,_,done,info=env.simulate(state,encode_action([0,6],8))
        np.testing.assert_array_equal(successor[0],[2,1,0,1])
        self.assertTrue(done);self.assertEqual(info['num_vehicle_collisions'],2)

    def test_goal_reward_is_paid_only_on_first_arrival(self):
        data=configuration(2);data['start_positions']=[[1,1,0],[1,3,0]]
        data['goal_positions']=[[2,1,0],[4,3,0]]
        env,state=self.env(data);env.rng=FixedChoices([6,6])
        state,reward,done,_=env.simulate(state,encode_action([6,6],8))
        self.assertFalse(done);self.assertEqual(reward,98.)
        env.rng=FixedChoices([6,1])
        _,reward,done,_=env.simulate(state,encode_action([0,1],8))
        self.assertFalse(done);self.assertEqual(reward,-2.)

    def test_simulate_has_no_physical_state_side_effects(self):
        env,state=self.env();before=state.copy();positions=env.positions.copy()
        for _ in range(30):env.simulate(state,6)
        np.testing.assert_array_equal(state,before)
        np.testing.assert_array_equal(env.positions,positions)
        self.assertEqual(env.t,0)

    def test_altitude_is_fixed_in_2d(self):
        env,state=self.env(configuration(2))
        for a in range(64):
            next_state,_,_,_=env.simulate(state,a)
            np.testing.assert_array_equal(next_state[:,2],state[:,2])

    def test_empirical_probability_matches_interior_model(self):
        data=configuration();data['start_positions']=[[2,2,0]]
        data['obstacle_cells']=[[3,2,0]]
        env,state=self.env(data)
        collisions=sum(env.simulate(state,6)[3]['num_collisions'] for _ in range(5000))
        self.assertLess(abs(collisions/5000-2/3),.03)

    def test_action_mask_preserves_retained_transition_laws(self):
        data=configuration(2,True)
        first,state=self.env(data,seed=12);second,_=self.env(data,seed=12)
        masked=ActionMask(first,'axial')
        self.assertEqual(masked.num_actions,36)
        for action in range(36):
            x,r,d,info=masked.simulate(state,action)
            y,s,e,j=second.simulate(state,masked.map_action(action))
            np.testing.assert_array_equal(x,y)
            self.assertEqual((r,d,info),(s,e,j))

    def test_planner_uses_only_public_interfaces(self):
        env,state=self.env(configuration(2,True))
        planner=MCTSPlanner(PublicOnly(env),max_simulations=8)
        action=planner.plan(state,.2)
        self.assertTrue(0 <= action < 26**2)
        self.assertEqual(planner.last_stats['simulations'],8)

    def test_zero_budget_returns_valid_fallback_without_sampling(self):
        env,state=self.env(configuration(2,True));planner=MCTSPlanner(env)
        action=planner.plan(state,0)
        self.assertTrue(0 <= action < env.num_actions)
        self.assertEqual(planner.last_stats['generative_calls'],0)

    def test_time_budget_on_four_drone_space(self):
        data=yaml.safe_load((ROOT/'environments/demo_four_3d.yaml').read_text())
        env,state=self.env(data);planner=MCTSPlanner(env)
        start=time.perf_counter();action=planner.plan(state,.05);elapsed=time.perf_counter()-start
        self.assertTrue(0 <= action < 26**4)
        self.assertLess(elapsed,.075)

    def test_terminal_input_does_not_sample(self):
        data=configuration();data['goal_positions']=data['start_positions']
        env,state=self.env(data);planner=MCTSPlanner(env)
        planner.plan(state,.1)
        self.assertEqual(planner.last_stats['generative_calls'],0)

    def test_one_remaining_step_does_not_roll_past_time_limit(self):
        data=configuration();data['max_num_steps']=1
        env,state=self.env(data);planner=MCTSPlanner(env,max_simulations=8)
        planner.plan(state,.2)
        self.assertEqual(planner.last_stats['generative_calls'],8)
        planner.plan(state,.2)
        self.assertEqual(planner.last_stats['generative_calls'],0)

    def test_initial_success_and_static_infeasibility(self):
        for name,success,status in [('single_cell_goal',True,'success'),
                                    ('already_at_goals',True,'success'),
                                    ('infeasible_altitude',False,'infeasible_static_path'),
                                    ('infeasible_wall',False,'infeasible_static_path'),
                                    ('infeasible_shared_goal',False,'infeasible_shared_goals')]:
            with self.subTest(name=name):
                result=run_episode(ROOT/f'environments/{name}.yaml',planning_time=.01)
                self.assertEqual(result['success'],success)
                self.assertEqual(result['termination'],status)
                self.assertEqual(result['steps'],0)

    def test_fixed_iteration_reproducibility(self):
        results=[run_episode(ROOT/'environments/demo_open_2d.yaml',seed=13,planning_time=1.,
                             planner_options={'max_simulations':8}) for _ in range(2)]
        self.assertEqual(results[0]['trajectory'],results[1]['trajectory'])

    def test_yaml_seed_is_used_unless_explicitly_overridden(self):
        data=configuration();data['seed']=37
        self.path.write_text(yaml.safe_dump(data))
        default=run_episode(self.path,planning_time=0,variant='heuristic')
        explicit=run_episode(self.path,seed=37,planning_time=0,variant='heuristic')
        override=run_episode(self.path,seed=11,planning_time=0,variant='heuristic')
        self.assertEqual(default['seed'],37)
        self.assertEqual(default['trajectory'],explicit['trajectory'])
        self.assertEqual(override['seed'],11)

    def test_execution_randomness_is_independent_of_search(self):
        first,state=self.env(seed=4);second,_=self.env(seed=4);model,_=self.env(seed=1004)
        for _ in range(100):model.simulate(state,6)
        x,r,d,i=first.step(6);y,s,e,j=second.step(6)
        np.testing.assert_array_equal(x,y);self.assertEqual((r,d,i),(s,e,j))

    def test_guided_proposals_never_command_invalid_cells(self):
        data=configuration(3,True);data['grid_size']=[5,5,3]
        data['start_positions']=[[0,0,0],[0,4,2],[1,2,1]];data['goal_positions']=[[4,0,0],[4,4,2],[4,2,1]]
        data['obstacle_cells']=[[1,1,0],[1,3,2],[2,2,1]]
        env,state=self.env(data);planner=MCTSPlanner(env,seed=3)
        blocked={tuple(p) for p in data['obstacle_cells']}
        for explore in (True,False):
            for _ in range(300):
                digits=decode_action(planner._proposal(state,randomize=True,explore=explore),26,3)
                for i,d in enumerate(digits):
                    target=tuple(state[i,:3]+planner.vectors[d])
                    self.assertTrue(all(0<=target[k]<5 if k<2 else 0<=target[k]<3 for k in range(3)))
                    self.assertNotIn(target,blocked)
        # Corner drones keep 6 of 7 neighbours (one obstacle); the interior drone keeps 23 of 26.
        self.assertEqual(planner._effective_actions(state),6*6*23)

    def test_risk_map_reduces_to_bfs_and_prefers_clearance(self):
        data=configuration();data['grid_size']=[7,5,1];data['start_positions']=[[0,2,0]]
        data['goal_positions']=[[6,2,0]];data['obstacle_cells']=[[3,2,0]];data['alpha']=2.
        env,state=self.env(data)
        plain=MCTSPlanner(env,risk_weight=0.);risky=MCTSPlanner(env)
        np.testing.assert_array_equal(np.where(plain.maps[0]>=0,plain.maps[0],np.inf),plain.costs[0])
        free=np.isfinite(risky.costs[0])
        self.assertTrue((risky.costs[0][free]>=plain.costs[0][free]-1e-9).all())
        # Next to the obstacle a slip can hit it; two cells away it cannot.
        self.assertGreater(risky.departure_risk[2,2,0],0.);self.assertEqual(risky.departure_risk[3,0,0],0.)
        manual=1.*risky.land_slip[2,2,0];self.assertAlmostEqual(risky.departure_risk[2,2,0],manual)

    def test_slip_model_matches_simulator_probabilities(self):
        data=configuration();data['start_positions']=[[2,2,0]];data['obstacle_cells']=[[3,2,0]]
        env,state=self.env(data);planner=MCTSPlanner(env)
        hits=sum(env.simulate(state,1)[3]['num_collisions'] for _ in range(6000))/6000
        self.assertLess(abs(hits-planner.departure_risk[2,2,0]),.02)

    def test_anchored_choice_respects_estimated_margin(self):
        from planner import Edge
        env,_=self.env();planner=MCTSPlanner(env,switch_z=1.)
        def edges(gain,spread):
            return {0:Edge(visits=10,value=.10,spread=spread),5:Edge(visits=10,value=.10+gain,spread=spread),
                    7:Edge(visits=30,value=.05,spread=spread)}
        self.assertEqual(planner._anchored(edges(.02,.9),0,[0,5,7]),0)    # gain inside noise
        self.assertEqual(planner._anchored(edges(.20,.009),0,[0,5,7]),5)  # clear improvement
        self.assertEqual(planner._anchored(edges(-.20,.009),0,[0,5,7]),0) # never switch to worse

    def test_fixed_altitude_slip_penalty_ignores_other_layers(self):
        data=configuration(2);data['grid_size']=[6,6,2]
        data['start_positions']=[[1,1,0],[2,2,1]];data['goal_positions']=[[5,1,0],[5,2,1]]
        data['obstacle_cells']=[[3,4,0]]
        env,_=self.env(data);planner=MCTSPlanner(env)
        self.assertEqual(np.max(planner._slip_penalty((1,1,0),[(2,2,1)])),0.)  # different layer
        self.assertGreater(np.max(planner._slip_penalty((1,1,0),[(2,2,0)])),0.)  # same layer
        data['change_altitude']=True
        env,_=self.env(data);planner=MCTSPlanner(env)
        self.assertGreater(np.max(planner._slip_penalty((1,1,0),[(2,2,1)])),0.)  # 3D slips cross layers

    def test_leaf_drops_goals_cut_off_by_frozen_drones(self):
        data=configuration(2);data['grid_size']=[6,1,1]
        data['start_positions']=[[0,0,0],[3,0,0]];data['goal_positions']=[[5,0,0],[3,0,0]]
        env,state=self.env(data);planner=MCTSPlanner(env)
        self.assertEqual(planner.unreachable_drones(state),[0])
        self.assertLess(planner._leaf(state,20),0.)  # no goal reward behind the frozen drone
        data['grid_size']=[6,2,1]  # a second row keeps the goal reachable
        env,state=self.env(data);planner=MCTSPlanner(env)
        self.assertEqual(planner.unreachable_drones(state),[])
        self.assertGreater(planner._leaf(state,20),0.)

    def test_incremental_backup_matches_sample_mean_and_variance(self):
        from planner import Edge
        values=np.random.default_rng(2).normal(.3,.2,size=50);edge=Edge()
        for v in values:edge.add(float(v))
        self.assertEqual(edge.visits,50);self.assertAlmostEqual(edge.value,values.mean())
        self.assertAlmostEqual(edge.spread/(edge.visits-1),values.var(ddof=1))

    def test_duplicate_labels_keep_physical_law(self):
        data=configuration(1,True)
        first,state=self.env(data,seed=5);second,_=self.env(data,seed=5)
        masked=ActionMask(first,'dup2')
        self.assertEqual(masked.num_actions,52)
        for action in (0,7,26,33,51):
            x,r,d,info=masked.simulate(state,action)
            y,s,e,j=second.simulate(state,action%26)
            np.testing.assert_array_equal(x,y);self.assertEqual((r,d,info),(s,e,j))

    def test_progress_goes_to_stderr_only(self):
        import contextlib,io,json,sys
        from run_u8076655 import main
        out,err=io.StringIO(),io.StringIO()
        argv=sys.argv
        sys.argv=['run','--config',str(ROOT/'environments/thin_1d.yaml'),'--planning-time','0.01']
        try:
            with contextlib.redirect_stdout(out),contextlib.redirect_stderr(err):main()
        finally:
            sys.argv=argv
        self.assertTrue(json.loads(out.getvalue())['success'])
        self.assertIn('step   1/',err.getvalue());self.assertIn('success: True',err.getvalue())

    def test_outcome_is_reported_before_the_blocking_viewer(self):
        import run_u8076655
        events=[]
        class Viewer(HeadlessMultiDroneUnc):
            def show(self):events.append('show')
        original=run_u8076655.make_env
        def make(path,seed,visualize=False,action_mask='full'):
            return Viewer(path,seed=seed) if visualize else original(path,seed,action_mask=action_mask)
        run_u8076655.make_env=make
        try:
            result=run_u8076655.run_episode(ROOT/'environments/thin_1d.yaml',planning_time=.01,visualize=True,
                                            before_viewer=lambda r:events.append(('report',r['success'])))
        finally:
            run_u8076655.make_env=original
        self.assertEqual(events,[('report',True),'show'])
        self.assertTrue(result['success'])

    def test_value_guidance_is_optional_and_matches_deterministic_limit(self):
        data=configuration();data['alpha']=1e-6  # slips vanish: value = discounted steps + goal
        env,state=self.env(data)
        self.assertIsNone(MCTSPlanner(env,guidance='dijkstra').values)  # Dijkstra guidance keeps no values
        self.assertIsNotNone(MCTSPlanner(env).values)  # value-iteration guidance is the default
        planner=MCTSPlanner(env,guidance='vi');d=3;g=.98
        expected=-1.*(1-g**d)/(1-g)+100.*g**(d-1)
        self.assertAlmostEqual(planner.values[0][30][planner.value_index[1,1,0]],expected,places=3)
        self.assertEqual(planner._leaf(state,30)*planner.scale,planner.values[0][30][planner.value_index[1,1,0]])
        with self.assertRaises(ValueError):MCTSPlanner(env,guidance='astar')

    def test_value_guided_proposals_are_admissible(self):
        data=configuration(3,True);data['grid_size']=[5,5,3]
        data['start_positions']=[[0,0,0],[0,4,2],[1,2,1]];data['goal_positions']=[[4,0,0],[4,4,2],[4,2,1]]
        data['obstacle_cells']=[[1,1,0],[1,3,2],[2,2,1]]
        env,state=self.env(data);planner=MCTSPlanner(env,seed=3,guidance='vi')
        blocked={tuple(p) for p in data['obstacle_cells']}
        for explore in (True,False):
            for _ in range(200):
                digits=decode_action(planner._proposal(state,randomize=True,explore=explore),26,3)
                for i,d in enumerate(digits):
                    target=tuple(state[i,:3]+planner.vectors[d])
                    self.assertTrue(all(0<=target[k]<5 if k<2 else 0<=target[k]<3 for k in range(3)))
                    self.assertNotIn(target,blocked)

    def test_value_iteration_matches_upstream_stochastic_returns(self):
        # Obtain probabilities and outcomes through the supplied simulator, without
        # duplicating the planner's slip-law construction or transition matrix.
        class CaptureDistribution:
            def choice(self,count,p):
                self.probabilities=np.asarray(p,dtype=float).copy()
                return int(np.flatnonzero(p>0)[0])

        for gamma in (0.,.8,1.):
            for mask in ('full','axial'):
                with self.subTest(gamma=gamma,mask=mask):
                    data=configuration();data.update(grid_size=[4,3,1],
                        start_positions=[[1,1,0]],goal_positions=[[3,1,0]],
                        obstacle_cells=[[2,1,0]],max_num_steps=3,alpha=.7,
                        discount_factor=gamma)
                    env,_=self.env(data)
                    model=env if mask=='full' else ActionMask(env,mask)
                    planner=MCTSPlanner(model)
                    cells=[tuple(p) for p in np.argwhere(planner.free)]
                    goal=tuple(data['goal_positions'][0])
                    outcomes={}
                    for p in cells:
                        if p==goal:continue
                        state=np.asarray([[*p,0]],dtype=np.int32)
                        for a in range(planner.base):
                            capture=CaptureDistribution();env.rng=capture
                            model.simulate(state,a)
                            branches=[]
                            for j,probability in enumerate(capture.probabilities):
                                if probability==0:continue
                                env.rng=FixedChoices([j])
                                nxt,reward,done,_=model.simulate(state,a)
                                branches.append((probability,reward,bool(done),tuple(nxt[0,:3])))
                            outcomes[p,a]=branches
                    previous={p:0. for p in cells}
                    for h in range(1,4):
                        current={goal:0.}
                        for p in cells:
                            if p==goal:continue
                            q=[sum(prob*(reward+(0. if done else gamma*previous[nxt]))
                                   for prob,reward,done,nxt in outcomes[p,a])
                               for a in range(planner.base)]
                            index=planner.value_index[p]
                            np.testing.assert_allclose(planner.q_history[0][h][index],q,
                                                       rtol=2e-6,atol=1e-4)
                            current[p]=max(q)
                            self.assertAlmostEqual(planner.values[0][h][index],max(q),delta=1e-4)
                        self.assertEqual(planner.values[0][h][planner.value_index[goal]],0.)
                        previous=current

    def test_invalid_configurations_fail_clearly(self):
        changes=[{'alpha':0.}, {'alpha':-1.}, {'discount_factor':1.1}, {'max_num_steps':0},
                 {'grid_size':[0,4,1]}, {'start_positions':[[8,0,0]]},
                 {'goal_positions':[[1.5,0,0]]}, {'obstacle_cells':[[4,0,0]]},
                 {'obstacle_cells':[[1,1,0]]}]
        for change in changes:
            with self.subTest(change=change):
                data=configuration();data.update(change)
                with self.assertRaises(ValueError):self.env(data)


if __name__ == '__main__':
    unittest.main()

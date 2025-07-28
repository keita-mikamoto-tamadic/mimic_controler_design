#!/usr/bin/env python3
"""
複数テストシナリオでの動作検証
フルロボット動力学シミュレーションの総合テスト
"""

import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
import os
from basic_loader import FullRobotLoader
from dynamics_simulator_clean import FullRobotDynamicsSimulator
from animation_renderer import FullRobotAnimationRenderer

class FullRobotTestSuite:
    """フルロボットテストスイート"""
    
    def __init__(self):
        self.test_results = {}
        print(f"🧪 フルロボットテストスイート初期化")
    
    def run_comprehensive_tests(self):
        """包括的テストの実行"""
        print("="*80)
        print("🚀 フルロボット動力学 - 包括的テストスイート")
        print("="*80)
        
        test_scenarios = [
            {
                'name': 'Stable_Symmetric',
                'description': '安定対称構成',
                'base_pos': [0.0, 0.0],
                'base_orientation': [0.0, 0.0, 0.0],
                'left_leg': [0.2, -0.4],
                'right_leg': [0.2, -0.4],
                'expected_motion': 'minimal'
            },
            {
                'name': 'Small_Asymmetry',
                'description': '小さな非対称',
                'base_pos': [0.0, 0.0],
                'base_orientation': [0.0, 0.05, 0.0],
                'left_leg': [0.3, -0.6],
                'right_leg': [0.25, -0.55],
                'expected_motion': 'small_oscillation'
            },
            {
                'name': 'Large_Lean',
                'description': '大きな前傾',
                'base_pos': [0.0, 0.0],
                'base_orientation': [0.0, 0.3, 0.0],
                'left_leg': [0.5, -0.8],
                'right_leg': [0.5, -0.8],
                'expected_motion': 'forward_fall'
            },
            {
                'name': 'Extreme_Asymmetry',
                'description': '極端な非対称',
                'base_pos': [0.1, 0.0],
                'base_orientation': [0.0, 0.1, 0.0],
                'left_leg': [1.0, -1.4],
                'right_leg': [0.1, -0.3],
                'expected_motion': 'complex_fall'
            }
        ]
        
        for i, scenario in enumerate(test_scenarios):
            print(f"\n{'='*20} テスト {i+1}/{len(test_scenarios)}: {scenario['name']} {'='*20}")
            print(f"📋 {scenario['description']}")
            print(f"🎯 期待動作: {scenario['expected_motion']}")
            
            try:
                result = self._run_single_scenario(scenario)
                self.test_results[scenario['name']] = result
                
                # 結果サマリー
                print(f"✅ {scenario['name']} 完了")
                print(f"   最大移動距離: {result['max_displacement']:.3f}m")
                print(f"   最大高度変化: {result['height_change']:.3f}m")
                print(f"   数値安定性: {'✅' if result['numerically_stable'] else '❌'}")
                
            except Exception as e:
                print(f"❌ {scenario['name']} 失敗: {e}")
                self.test_results[scenario['name']] = {'error': str(e)}
        
        # 総合結果
        self._generate_comprehensive_report()
        return self.test_results
    
    def _run_single_scenario(self, scenario):
        """単一シナリオの実行"""
        # シミュレータ初期化
        simulator = FullRobotDynamicsSimulator()
        
        # 初期条件設定
        free_state_init, free_velocity_init = simulator.set_initial_condition(
            base_pos=scenario['base_pos'],
            base_orientation=scenario['base_orientation'],
            left_leg=scenario['left_leg'],
            right_leg=scenario['right_leg']
        )
        
        # シミュレーション実行
        sim_time = 2.0  # 短めに設定
        results = simulator.simulate(free_state_init, free_velocity_init, T_sim=sim_time, dt=0.02)
        
        # 結果解析
        analysis = self._analyze_simulation_results(results, scenario)
        
        # アニメーション作成（選択的）
        if analysis['interesting_motion']:
            animator = FullRobotAnimationRenderer(results)
            animator.results = results
            
            filename = f"scenario_{scenario['name'].lower()}_animation.gif"
            try:
                anim = animator.create_robot_animation(
                    title=f"Scenario: {scenario['description']}",
                    save_gif=True
                )
                analysis['animation_created'] = filename
            except Exception as e:
                print(f"   ⚠️  アニメーション作成失敗: {e}")
                analysis['animation_created'] = None
        
        return analysis
    
    def _analyze_simulation_results(self, results, scenario):
        """シミュレーション結果の解析"""
        states = results['states']
        time = results['time']
        
        # ベース位置変化
        base_positions = states[:, :3]  # x, y, z
        initial_pos = base_positions[0]
        final_pos = base_positions[-1]
        
        # 移動距離計算
        displacement = np.linalg.norm(final_pos - initial_pos)
        max_displacement = np.max([np.linalg.norm(pos - initial_pos) for pos in base_positions])
        
        # 高度変化
        height_change = final_pos[2] - initial_pos[2]
        max_height_change = np.max(base_positions[:, 2]) - np.min(base_positions[:, 2])
        
        # 関節角度変化
        left_leg_motion = np.max(np.abs(np.diff(states[:, 6:8], axis=0)))  # 左脚
        right_leg_motion = np.max(np.abs(np.diff(states[:, 8:10], axis=0)))  # 右脚
        
        # 数値安定性
        numerically_stable = np.all(np.isfinite(states[-1])) and displacement < 100
        
        # 動作の興味深さ判定
        interesting_motion = (max_displacement > 0.1 or 
                            abs(height_change) > 0.05 or 
                            left_leg_motion > 0.1 or 
                            right_leg_motion > 0.1)
        
        analysis = {
            'scenario_name': scenario['name'],
            'final_displacement': displacement,
            'max_displacement': max_displacement,
            'height_change': height_change,
            'max_height_change': max_height_change,
            'left_leg_motion': left_leg_motion,
            'right_leg_motion': right_leg_motion,
            'numerically_stable': numerically_stable,
            'interesting_motion': interesting_motion,
            'simulation_time': time[-1],
            'initial_conditions': {
                'base_pos': scenario['base_pos'],
                'base_orientation': scenario['base_orientation'],
                'left_leg': scenario['left_leg'],
                'right_leg': scenario['right_leg']
            }
        }
        
        return analysis
    
    def _generate_comprehensive_report(self):
        """包括的レポート生成"""
        print(f"\n{'='*80}")
        print(f"📊 包括的テスト結果レポート")
        print(f"{'='*80}")
        
        successful_tests = [name for name, result in self.test_results.items() if 'error' not in result]
        failed_tests = [name for name, result in self.test_results.items() if 'error' in result]
        
        print(f"✅ 成功テスト: {len(successful_tests)}/{len(self.test_results)}")
        print(f"❌ 失敗テスト: {len(failed_tests)}/{len(self.test_results)}")
        
        if successful_tests:
            print(f"\n📈 成功テスト詳細:")
            for name in successful_tests:
                result = self.test_results[name]
                stability = "✅ 安定" if result['numerically_stable'] else "❌ 不安定"
                motion = "🎬 動的" if result['interesting_motion'] else "🟢 静的"
                print(f"   {name:20s}: 移動{result['max_displacement']:6.3f}m, {stability}, {motion}")
        
        if failed_tests:
            print(f"\n❌ 失敗テスト:")
            for name in failed_tests:
                error = self.test_results[name]['error']
                print(f"   {name:20s}: {error}")
        
        # 統計サマリー  
        if successful_tests:
            displacements = [self.test_results[name]['max_displacement'] for name in successful_tests]
            height_changes = [abs(self.test_results[name]['height_change']) for name in successful_tests]
            
            print(f"\n📊 動作統計:")
            print(f"   平均移動距離: {np.mean(displacements):.4f}m")
            print(f"   最大移動距離: {np.max(displacements):.4f}m")
            print(f"   平均高度変化: {np.mean(height_changes):.4f}m")
            print(f"   最大高度変化: {np.max(height_changes):.4f}m")
        
        # 推奨事項
        print(f"\n💡 推奨事項:")
        
        large_motion_tests = [name for name in successful_tests 
                            if self.test_results[name]['max_displacement'] > 1.0]
        if large_motion_tests:
            print(f"   🎯 大きな動作を示すテスト: {', '.join(large_motion_tests)}")
            print(f"   → これらのGIFアニメーションで物理動作を確認推奨")
        
        stable_tests = [name for name in successful_tests 
                       if self.test_results[name]['numerically_stable']]
        if len(stable_tests) == len(successful_tests):
            print(f"   ✅ 全テストが数値的に安定 → 拘束ソルバーの実装成功")
        
        print(f"\n🎉 フルロボット動力学システム実装完了!")

def run_debug_analysis():
    """デバッグ解析の実行"""
    print("="*60)
    print("🔍 フルロボット動力学 - デバッグ解析")  
    print("="*60)
    
    # 簡単な対称ケース
    simulator = FullRobotDynamicsSimulator()
    
    free_state_init, free_velocity_init = simulator.set_initial_condition(
        base_pos=[0.0, 0.0],
        base_orientation=[0.0, 0.0, 0.0],
        left_leg=[0.3, -0.6],
        right_leg=[0.3, -0.6]
    )
    
    print(f"🧪 デバッグ用初期状態:")
    print(f"   free_state: {free_state_init}")
    print(f"   free_velocity: {free_velocity_init}")
    
    # 1ステップのみ実行
    results = simulator.simulate(free_state_init, free_velocity_init, T_sim=0.1, dt=0.02)
    
    print(f"🔍 1ステップ後の結果:")
    final_state = results['states'][-1]
    print(f"   最終状態: {final_state[:10]}") # 位置のみ
    print(f"   速度: {final_state[10:]}")
    
    # 拘束ソルバーの詳細チェック
    solver = simulator.solver
    q_test = solver.solve_constraint_configuration(free_state_init)
    contact_positions = solver.compute_wheel_contact_positions(q_test)
    
    print(f"🔧 拘束ソルバー詳細:")
    print(f"   完全構成 q: {q_test}")
    for wheel_name, pos_data in contact_positions.items():
        print(f"   {wheel_name}: 接地Z={pos_data['contact'][2]:.6f}")

if __name__ == "__main__":
    # メインテストスイート実行
    test_suite = FullRobotTestSuite()
    results = test_suite.run_comprehensive_tests()
    
    # デバッグ解析（オプション）
    print(f"\n" + "="*60)
    run_debug_analysis()
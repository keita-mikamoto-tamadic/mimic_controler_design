#!/usr/bin/env python3
"""
線形化システム統合テスト
全コンポーネントの統合動作確認
"""

import numpy as np
import os
import sys

# ローカルモジュールのインポート
from equilibrium_finder import EquilibriumFinder
from linearized_dynamics import LinearizedDynamics
from constraint_reduction import ConstraintReduction

class LinearizationIntegrationTest:
    """線形化システム統合テスト"""
    
    def __init__(self):
        """初期化"""
        print(f"🧪 線形化システム統合テスト初期化")
        
        # コンポーネント初期化
        self.eq_finder = None
        self.linearizer = None
        self.reducer = None
        
        # データ格納
        self.equilibrium_data = None
        self.A_full = None
        self.B_full = None
        self.A_reduced = None
        self.B_reduced = None
    
    def run_full_pipeline(self):
        """
        完全パイプラインの実行
        
        Returns:
            dict: 全結果データ
        """
        print(f"\n🚀 完全線形化パイプライン実行開始")
        
        # Step 1: 平衡点探索
        print(f"\n--- Step 1: 平衡点探索 ---")
        self.eq_finder = EquilibriumFinder()
        self.equilibrium_data = self.eq_finder.find_specified_equilibrium()
        
        # Step 2: 線形化行列計算
        print(f"\n--- Step 2: CasADi線形化 ---")
        self.linearizer = LinearizedDynamics(self.eq_finder.robot)
        self.A_full, self.B_full = self.linearizer.compute_linearization_matrices(
            self.equilibrium_data['q_eq'],
            self.equilibrium_data['dq_eq'], 
            self.equilibrium_data['tau_eq']
        )
        
        # Step 3: 幾何学的拘束による縮約
        print(f"\n--- Step 3: 拘束縮約 ---")
        self.reducer = ConstraintReduction()
        self.A_reduced, self.B_reduced = self.reducer.compute_reduced_system(
            self.A_full, self.B_full
        )
        
        # Step 4: 統合結果の検証
        print(f"\n--- Step 4: 統合検証 ---")
        self._verify_integration()
        
        # 結果データの整理
        results = self._compile_results()
        
        print(f"\n✅ 完全線形化パイプライン実行完了")
        return results
    
    def _verify_integration(self):
        """統合システムの検証"""
        print(f"\n🔍 統合システム検証")
        
        # 1. 次元整合性確認
        print(f"   次元整合性確認:")
        print(f"     平衡点: q={self.equilibrium_data['q_eq'].shape}, dq={self.equilibrium_data['dq_eq'].shape}")
        print(f"     フル行列: A={self.A_full.shape}, B={self.B_full.shape}")
        print(f"     縮約行列: A={self.A_reduced.shape}, B={self.B_reduced.shape}")
        
        # 2. 平衡点での動力学確認
        print(f"   平衡点動力学確認:")
        try:
            x_eq = np.concatenate([self.equilibrium_data['q_eq'], self.equilibrium_data['dq_eq']])
            f_eq = self.linearizer.dynamics_func(x_eq, self.equilibrium_data['tau_eq'])
            f_eq_norm = np.linalg.norm(f_eq)
            
            print(f"     平衡点誤差: ||f(x_eq, u_eq)|| = {f_eq_norm:.6e}")
            
            if f_eq_norm < 1e-6:
                print(f"     ✅ 平衡点条件満足")
            else:
                print(f"     ⚠️ 平衡点誤差が大きい")
                
        except Exception as e:
            print(f"     ❌ 平衡点動力学確認エラー: {e}")
        
        # 3. 拘束満足確認
        print(f"   拘束満足確認:")
        self._verify_constraints()
        
        # 4. 線形化精度確認
        print(f"   線形化精度確認:")
        self._verify_linearization_accuracy()
    
    def _verify_constraints(self):
        """拘束条件の満足確認"""
        q_eq = self.equilibrium_data['q_eq']
        
        # 幾何学的拘束確認: upper = -2 * lower
        phi_L_lower = q_eq[4]
        phi_R_lower = q_eq[5]
        phi_L_upper = q_eq[6]
        phi_R_upper = q_eq[7]
        
        constraint_L_error = phi_L_upper - (-2.0 * phi_L_lower)
        constraint_R_error = phi_R_upper - (-2.0 * phi_R_lower)
        
        print(f"     左脚拘束誤差: {constraint_L_error:.6e}")
        print(f"     右脚拘束誤差: {constraint_R_error:.6e}")
        
        if max(abs(constraint_L_error), abs(constraint_R_error)) < 1e-10:
            print(f"     ✅ 幾何学的拘束満足")
        else:
            print(f"     ⚠️ 幾何学的拘束誤差あり")
    
    def _verify_linearization_accuracy(self):
        """線形化精度の確認"""
        try:
            # 小摂動での線形近似精度テスト
            x_eq = np.concatenate([self.equilibrium_data['q_eq'], self.equilibrium_data['dq_eq']])
            u_eq = self.equilibrium_data['tau_eq']
            
            # 摂動生成
            dx = np.random.normal(0, 1e-5, x_eq.shape)
            du = np.random.normal(0, 1e-5, u_eq.shape)
            
            # 非線形応答
            f_eq = self.linearizer.dynamics_func(x_eq, u_eq)
            f_pert = self.linearizer.dynamics_func(x_eq + dx, u_eq + du)
            df_nonlinear = np.array(f_pert - f_eq).flatten()
            
            # 線形近似応答
            df_linear = self.A_full @ dx + self.B_full @ du
            
            # 誤差評価
            error = np.linalg.norm(df_nonlinear - df_linear)
            relative_error = error / (np.linalg.norm(df_nonlinear) + 1e-12)
            
            print(f"     線形化誤差: {error:.6e}")
            print(f"     相対誤差: {relative_error:.6%}")
            
            if relative_error < 0.01:  # 1%以下
                print(f"     ✅ 線形化精度良好")
            else:
                print(f"     ⚠️ 線形化精度要改善")
                
        except Exception as e:
            print(f"     ❌ 線形化精度確認エラー: {e}")
    
    def _compile_results(self):
        """結果データの整理"""
        print(f"\n📋 結果データ整理")
        
        results = {
            # 平衡点データ
            'equilibrium': self.equilibrium_data,
            
            # フルシステム
            'full_system': {
                'A': self.A_full,
                'B': self.B_full,
                'n_states': self.A_full.shape[0],
                'n_inputs': self.B_full.shape[1]
            },
            
            # 縮約システム
            'reduced_system': {
                'A': self.A_reduced,
                'B': self.B_reduced,
                'n_states': self.A_reduced.shape[0],
                'n_inputs': self.B_reduced.shape[1]
            },
            
            # 変換行列
            'transformation': self.reducer.get_transformation_info(),
            
            # システム特性
            'properties': self._analyze_system_properties()
        }
        
        print(f"   結果データ構築完了")
        return results
    
    def _analyze_system_properties(self):
        """システム特性分析"""
        properties = {}
        
        try:
            # フルシステム固有値
            eigs_full = np.linalg.eigvals(self.A_full)
            properties['eigenvalues_full'] = eigs_full
            properties['stability_full'] = {
                'stable_poles': np.sum(np.real(eigs_full) < 0),
                'unstable_poles': np.sum(np.real(eigs_full) > 0),
                'marginal_poles': np.sum(np.abs(np.real(eigs_full)) < 1e-10)
            }
            
            # 縮約システム固有値
            eigs_reduced = np.linalg.eigvals(self.A_reduced)
            properties['eigenvalues_reduced'] = eigs_reduced
            properties['stability_reduced'] = {
                'stable_poles': np.sum(np.real(eigs_reduced) < 0),
                'unstable_poles': np.sum(np.real(eigs_reduced) > 0),
                'marginal_poles': np.sum(np.abs(np.real(eigs_reduced)) < 1e-10)
            }
            
            # 制御可能性（簡易チェック）
            from scipy.linalg import matrix_rank
            
            # フルシステム制御可能性行列
            C_full = self.B_full.copy()
            for i in range(1, self.A_full.shape[0]):
                C_full = np.hstack([C_full, np.linalg.matrix_power(self.A_full, i) @ self.B_full])
            properties['controllability_full'] = {
                'rank': matrix_rank(C_full),
                'full_rank': matrix_rank(C_full) == self.A_full.shape[0]
            }
            
            # 縮約システム制御可能性行列
            C_reduced = self.B_reduced.copy()
            for i in range(1, self.A_reduced.shape[0]):
                C_reduced = np.hstack([C_reduced, np.linalg.matrix_power(self.A_reduced, i) @ self.B_reduced])
            properties['controllability_reduced'] = {
                'rank': matrix_rank(C_reduced),
                'full_rank': matrix_rank(C_reduced) == self.A_reduced.shape[0]
            }
            
        except Exception as e:
            print(f"   ⚠️ システム特性分析エラー: {e}")
            properties['analysis_error'] = str(e)
        
        return properties
    
    def print_summary(self, results):
        """結果サマリーの表示"""
        print(f"\n📊 線形化システム統合結果サマリー")
        print(f"="*60)
        
        # 平衡点情報
        eq = results['equilibrium']
        print(f"平衡点情報:")
        print(f"  上腿角度: {eq['upper_angle_deg']:.1f}deg")
        print(f"  下腿角度: {eq['lower_angle_deg']:.1f}deg") 
        print(f"  ホイール半径: {eq['wheel_radius']:.6f}m")
        
        # システム次元
        full = results['full_system']
        reduced = results['reduced_system']
        print(f"\nシステム次元:")
        print(f"  フルシステム: 状態{full['n_states']}次元, 入力{full['n_inputs']}次元")
        print(f"  縮約システム: 状態{reduced['n_states']}次元, 入力{reduced['n_inputs']}次元")
        print(f"  縮約率: 状態{reduced['n_states']/full['n_states']:.1%}, 入力{reduced['n_inputs']/full['n_inputs']:.1%}")
        
        # 安定性分析
        if 'properties' in results and 'stability_full' in results['properties']:
            stab_full = results['properties']['stability_full']
            stab_reduced = results['properties']['stability_reduced']
            
            print(f"\n安定性分析:")
            print(f"  フルシステム: 安定極{stab_full['stable_poles']}, 不安定極{stab_full['unstable_poles']}")
            print(f"  縮約システム: 安定極{stab_reduced['stable_poles']}, 不安定極{stab_reduced['unstable_poles']}")
        
        # 制御可能性
        if 'properties' in results and 'controllability_full' in results['properties']:
            ctrl_full = results['properties']['controllability_full']
            ctrl_reduced = results['properties']['controllability_reduced']
            
            print(f"\n制御可能性:")
            print(f"  フルシステム: ランク{ctrl_full['rank']}, 完全制御可能{ctrl_full['full_rank']}")
            print(f"  縮約システム: ランク{ctrl_reduced['rank']}, 完全制御可能{ctrl_reduced['full_rank']}")
        
        print(f"\n✅ 線形化システム統合テスト完了")

def main():
    """メイン実行関数"""
    print("=== 3D ロボット線形化システム統合テスト ===")
    
    # 統合テスト実行
    tester = LinearizationIntegrationTest()
    results = tester.run_full_pipeline()
    
    # 結果表示
    tester.print_summary(results)
    
    # 結果保存（オプション）
    save_results = True
    if save_results:
        import pickle
        
        output_file = os.path.join(os.path.dirname(__file__), 'linearization_results.pkl')
        with open(output_file, 'wb') as f:
            pickle.dump(results, f)
        print(f"\n💾 結果を保存しました: {output_file}")
    
    return results

if __name__ == "__main__":
    results = main()
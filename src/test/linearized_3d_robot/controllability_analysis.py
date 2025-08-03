#!/usr/bin/env python3
"""
可制御性解析システム
線形化されたロボットシステムの制御可能性・可観測性解析
"""

import numpy as np
import os
import sys
from scipy.linalg import svd, qr
from scipy.signal import place_poles

# matrix_rankはnumpyから使用（scipyのバージョン問題対応）
matrix_rank = np.linalg.matrix_rank

# ローカルモジュールのインポート
from equilibrium_finder import EquilibriumFinder
from linearized_dynamics_simple import SimpleLinearizedDynamics
from constraint_reduction import ConstraintReduction

class ControllabilityAnalysis:
    """制御可能性・可観測性解析システム"""
    
    def __init__(self):
        """初期化"""
        print(f"🎛️ 制御可能性解析システム初期化")
    
    def analyze_controllability(self, A, B, system_name="System"):
        """
        制御可能性解析
        
        Args:
            A: システム行列 (n×n)
            B: 入力行列 (n×m)
            system_name: システム名
            
        Returns:
            dict: 制御可能性解析結果
        """
        print(f"\n🔧 {system_name} 制御可能性解析")
        print(f"   システム次元: A={A.shape}, B={B.shape}")
        
        n_states = A.shape[0]
        n_inputs = B.shape[1]
        
        # 制御可能性行列構築
        print(f"   制御可能性行列構築中...")
        C_matrix = self._build_controllability_matrix(A, B)
        
        # ランク計算
        C_rank = matrix_rank(C_matrix)
        is_controllable = (C_rank == n_states)
        
        print(f"   制御可能性行列: {C_matrix.shape}")
        print(f"   ランク: {C_rank}/{n_states}")
        print(f"   制御可能: {'✅ YES' if is_controllable else '❌ NO'}")
        
        # 特異値分解による詳細解析
        svd_analysis = self._analyze_controllability_svd(C_matrix)
        
        # PBHテスト（Popov-Belevitch-Hautus test）
        pbh_analysis = self._pbh_test(A, B)
        
        # 可制御部分空間の分析
        controllable_subspace = self._analyze_controllable_subspace(A, B, C_matrix, C_rank)
        
        results = {
            'system_name': system_name,
            'n_states': n_states,
            'n_inputs': n_inputs,
            'controllability_matrix_rank': C_rank,
            'is_controllable': is_controllable,
            'controllability_deficiency': n_states - C_rank,
            'svd_analysis': svd_analysis,
            'pbh_test': pbh_analysis,
            'controllable_subspace': controllable_subspace
        }
        
        self._print_controllability_summary(results)
        return results
    
    def _build_controllability_matrix(self, A, B):
        """
        制御可能性行列構築: [B, AB, A²B, ..., A^(n-1)B]
        
        Args:
            A: システム行列
            B: 入力行列
            
        Returns:
            C_matrix: 制御可能性行列
        """
        n = A.shape[0]
        matrices = [B]
        
        A_power = np.eye(n)
        for i in range(1, n):
            A_power = A_power @ A
            matrices.append(A_power @ B)
        
        C_matrix = np.hstack(matrices)
        return C_matrix
    
    def _analyze_controllability_svd(self, C_matrix):
        """
        特異値分解による制御可能性解析
        
        Args:
            C_matrix: 制御可能性行列
            
        Returns:
            dict: SVD解析結果
        """
        U, s, Vt = svd(C_matrix)
        
        # 特異値の分析
        significant_threshold = 1e-10
        significant_sv = s[s > significant_threshold]
        condition_number = s[0] / s[-1] if s[-1] > 1e-16 else np.inf
        
        print(f"   特異値解析:")
        print(f"     最大特異値: {s[0]:.6e}")
        print(f"     最小特異値: {s[-1]:.6e}")
        print(f"     条件数: {condition_number:.6e}")
        print(f"     有意特異値数: {len(significant_sv)}")
        
        return {
            'singular_values': s,
            'significant_singular_values': significant_sv,
            'condition_number': condition_number,
            'numerical_rank': len(significant_sv)
        }
    
    def _pbh_test(self, A, B):
        """
        PBH (Popov-Belevitch-Hautus) テスト
        固有値レベルでの制御可能性確認
        
        Args:
            A: システム行列
            B: 入力行列
            
        Returns:
            dict: PBHテスト結果
        """
        print(f"   PBHテスト実行中...")
        
        eigenvalues = np.linalg.eigvals(A)
        uncontrollable_modes = []
        
        for i, lam in enumerate(eigenvalues):
            # [λI - A, B] の行列のランクをチェック
            n = A.shape[0]
            test_matrix = np.hstack([lam * np.eye(n) - A, B])
            rank_test = matrix_rank(test_matrix)
            
            if rank_test < n:
                uncontrollable_modes.append({
                    'eigenvalue': lam,
                    'index': i,
                    'rank_deficiency': n - rank_test
                })
        
        is_pbh_controllable = len(uncontrollable_modes) == 0
        
        print(f"     不可制御モード数: {len(uncontrollable_modes)}")
        print(f"     PBH制御可能: {'✅ YES' if is_pbh_controllable else '❌ NO'}")
        
        if uncontrollable_modes:
            print(f"     不可制御固有値:")
            for mode in uncontrollable_modes[:5]:  # 最初の5個のみ表示
                print(f"       λ={mode['eigenvalue']:.6f}")
        
        return {
            'is_controllable': is_pbh_controllable,
            'uncontrollable_modes': uncontrollable_modes,
            'eigenvalues': eigenvalues
        }
    
    def _analyze_controllable_subspace(self, A, B, C_matrix, rank):
        """
        可制御部分空間の解析
        
        Args:
            A, B: システム行列
            C_matrix: 制御可能性行列
            rank: 制御可能性行列のランク
            
        Returns:
            dict: 可制御部分空間解析結果
        """
        print(f"   可制御部分空間解析中...")
        
        # QR分解による可制御部分空間の基底取得
        Q, R = qr(C_matrix)
        
        # 可制御部分空間の次元
        controllable_dim = rank
        uncontrollable_dim = A.shape[0] - rank
        
        # 可制御部分空間の基底ベクトル
        controllable_basis = Q[:, :controllable_dim]
        
        print(f"     可制御部分空間次元: {controllable_dim}")
        print(f"     不可制御部分空間次元: {uncontrollable_dim}")
        
        return {
            'controllable_dimension': controllable_dim,
            'uncontrollable_dimension': uncontrollable_dim,
            'controllable_basis': controllable_basis,
            'controllable_subspace_matrix': Q
        }
    
    def _print_controllability_summary(self, results):
        """制御可能性解析結果のサマリー表示"""
        print(f"\n📊 {results['system_name']} 制御可能性サマリー:")
        print(f"   システム次元: {results['n_states']}状態 × {results['n_inputs']}入力")
        print(f"   制御可能性: {'✅ 完全制御可能' if results['is_controllable'] else '❌ 不完全制御可能'}")
        
        if not results['is_controllable']:
            deficiency = results['controllability_deficiency']
            print(f"   制御不能次元: {deficiency}")
            print(f"   可制御次元: {results['n_states'] - deficiency}")
        
        svd = results['svd_analysis']
        print(f"   数値ランク: {svd['numerical_rank']}")
        print(f"   条件数: {svd['condition_number']:.2e}")
        
        pbh = results['pbh_test']
        if not pbh['is_controllable']:
            print(f"   不可制御モード数: {len(pbh['uncontrollable_modes'])}")
    
    def analyze_observability(self, A, C, system_name="System"):
        """
        可観測性解析
        
        Args:
            A: システム行列 (n×n)
            C: 出力行列 (p×n)
            system_name: システム名
            
        Returns:
            dict: 可観測性解析結果
        """
        print(f"\n👁️ {system_name} 可観測性解析")
        print(f"   システム次元: A={A.shape}, C={C.shape}")
        
        n_states = A.shape[0]
        n_outputs = C.shape[0]
        
        # 可観測性行列構築: [C; CA; CA²; ...; CA^(n-1)]
        print(f"   可観測性行列構築中...")
        O_matrix = self._build_observability_matrix(A, C)
        
        # ランク計算
        O_rank = matrix_rank(O_matrix)
        is_observable = (O_rank == n_states)
        
        print(f"   可観測性行列: {O_matrix.shape}")
        print(f"   ランク: {O_rank}/{n_states}")
        print(f"   可観測: {'✅ YES' if is_observable else '❌ NO'}")
        
        # 特異値分解
        U, s, Vt = svd(O_matrix)
        condition_number = s[0] / s[-1] if s[-1] > 1e-16 else np.inf
        
        results = {
            'system_name': system_name,
            'n_states': n_states,
            'n_outputs': n_outputs,
            'observability_matrix_rank': O_rank,
            'is_observable': is_observable,
            'observability_deficiency': n_states - O_rank,
            'condition_number': condition_number,
            'singular_values': s
        }
        
        print(f"   条件数: {condition_number:.2e}")
        return results
    
    def _build_observability_matrix(self, A, C):
        """
        可観測性行列構築: [C; CA; CA²; ...; CA^(n-1)]
        
        Args:
            A: システム行列
            C: 出力行列
            
        Returns:
            O_matrix: 可観測性行列
        """
        n = A.shape[0]
        matrices = [C]
        
        CA_power = C.copy()
        for i in range(1, n):
            CA_power = CA_power @ A
            matrices.append(CA_power)
        
        O_matrix = np.vstack(matrices)
        return O_matrix
    
    def test_pole_placement(self, A, B, desired_poles=None):
        """
        極配置による制御器設計テスト
        
        Args:
            A, B: システム行列
            desired_poles: 希望極位置（Noneの場合は自動生成）
            
        Returns:
            dict: 極配置結果
        """
        print(f"\n🎯 極配置制御器設計テスト")
        
        n_states = A.shape[0]
        
        # 希望極の自動生成（全て左半平面の安定極）
        if desired_poles is None:
            desired_poles = -np.logspace(0, 1, n_states)  # -1, -10の間に対数分布
            print(f"   自動生成希望極: {len(desired_poles)}個")
        else:
            print(f"   指定希望極: {len(desired_poles)}個")
        
        try:
            # 極配置計算
            result = place_poles(A, B, desired_poles)
            K = result.gain_matrix
            
            # 閉ループシステム
            A_cl = A - B @ K
            actual_poles = np.linalg.eigvals(A_cl)
            
            # 極配置誤差
            pole_errors = []
            for desired in desired_poles:
                distances = np.abs(actual_poles - desired)
                min_error = np.min(distances)
                pole_errors.append(min_error)
            
            max_error = np.max(pole_errors)
            
            print(f"   ✅ 極配置成功")
            print(f"   制御器ゲイン: K={K.shape}")
            print(f"   最大極配置誤差: {max_error:.6e}")
            
            # 安定性確認
            stable_poles = np.sum(np.real(actual_poles) < 0)
            print(f"   安定極数: {stable_poles}/{len(actual_poles)}")
            
            return {
                'success': True,
                'gain_matrix': K,
                'desired_poles': desired_poles,
                'actual_poles': actual_poles,
                'pole_placement_errors': pole_errors,
                'max_pole_error': max_error,
                'is_stable': stable_poles == len(actual_poles)
            }
            
        except Exception as e:
            print(f"   ❌ 極配置失敗: {e}")
            return {
                'success': False,
                'error': str(e)
            }

def comprehensive_controllability_test():
    """包括的制御可能性テスト"""
    print("=== 包括的制御可能性解析テスト ===")
    
    # システム構築
    print("\n--- システム構築 ---")
    finder = EquilibriumFinder()
    eq_data = finder.find_specified_equilibrium()
    
    linearizer = SimpleLinearizedDynamics(finder.robot)
    print(f"線形化計算中（数値微分）...")
    A_full, B_full = linearizer.compute_linearization_matrices(
        eq_data['q_eq'], eq_data['dq_eq'], eq_data['tau_eq'], h=1e-6
    )
    
    reducer = ConstraintReduction()
    A_reduced, B_reduced = reducer.compute_reduced_system(A_full, B_full)
    
    # 解析システム初期化
    analyzer = ControllabilityAnalysis()
    
    # フルシステム解析
    print("\n" + "="*60)
    full_results = analyzer.analyze_controllability(A_full, B_full, "フルシステム")
    
    # 縮約システム解析
    print("\n" + "="*60)
    reduced_results = analyzer.analyze_controllability(A_reduced, B_reduced, "縮約システム")
    
    # 可観測性解析（状態フィードバック前提なので単位行列）
    print("\n" + "="*60)
    C_full = np.eye(A_full.shape[0])  # 全状態観測可能
    C_reduced = np.eye(A_reduced.shape[0])  # 全状態観測可能
    
    obs_full = analyzer.analyze_observability(A_full, C_full, "フルシステム")
    obs_reduced = analyzer.analyze_observability(A_reduced, C_reduced, "縮約システム")
    
    # 極配置テスト（制御可能な場合のみ）
    print("\n" + "="*60)
    if reduced_results['is_controllable']:
        pole_test = analyzer.test_pole_placement(A_reduced, B_reduced)
    else:
        print("🎯 極配置テスト: 縮約システムが不完全制御可能のためスキップ")
        pole_test = None
    
    # 総合結果表示
    print("\n" + "="*60)
    print("🏁 総合解析結果")
    print("="*60)
    
    print(f"\n📊 制御可能性比較:")
    print(f"   フルシステム: {'✅ 制御可能' if full_results['is_controllable'] else '❌ 不制御可能'}")
    print(f"   縮約システム: {'✅ 制御可能' if reduced_results['is_controllable'] else '❌ 不制御可能'}")
    
    print(f"\n📊 可観測性比較:")
    print(f"   フルシステム: {'✅ 可観測' if obs_full['is_observable'] else '❌ 不可観測'}")
    print(f"   縮約システム: {'✅ 可観測' if obs_reduced['is_observable'] else '❌ 不可観測'}")
    
    if pole_test and pole_test['success']:
        print(f"\n📊 制御器設計:")
        print(f"   極配置: ✅ 成功")
        print(f"   安定性: {'✅ 安定' if pole_test['is_stable'] else '❌ 不安定'}")
        print(f"   制御器ゲイン: {pole_test['gain_matrix'].shape}")
    
    # 制御理論的推奨事項
    print(f"\n💡 制御理論的推奨:")
    if reduced_results['is_controllable'] and obs_reduced['is_observable']:
        print(f"   ✅ LQR制御器設計可能")
        print(f"   ✅ 状態フィードバック制御適用可能")
        print(f"   ✅ 極配置制御可能")
    else:
        print(f"   ⚠️ 制御器設計に制約あり")
        if not reduced_results['is_controllable']:
            deficiency = reduced_results['controllability_deficiency']
            print(f"      - {deficiency}次元が制御不能")
        if not obs_reduced['is_observable']:
            print(f"      - 状態推定器が必要")
    
    return {
        'full_system': full_results,
        'reduced_system': reduced_results,
        'observability_full': obs_full,
        'observability_reduced': obs_reduced,
        'pole_placement': pole_test
    }

if __name__ == "__main__":
    results = comprehensive_controllability_test()
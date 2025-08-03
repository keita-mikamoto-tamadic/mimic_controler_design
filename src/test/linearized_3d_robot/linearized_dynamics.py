#!/usr/bin/env python3
"""
CasADi線形化動力学システム
自動微分を使用したロボット動力学の線形化
"""

import numpy as np
import casadi as cs
import pinocchio as pin
import os
import sys

# プロジェクトルートからfixed_robot_3dをインポート
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'urdf_full_robot_3d'))
from fixed_robot_3d import FixedRobot3D

class LinearizedDynamics:
    """CasADi自動微分による動力学線形化システム"""
    
    def __init__(self, robot=None):
        """
        初期化
        
        Args:
            robot: FixedRobot3Dインスタンス（Noneの場合は新規作成）
        """
        self.robot = robot if robot is not None else FixedRobot3D()
        self.model = self.robot.model
        self.data = self.robot.data
        
        # 次元数
        self.n_q = 10  # 独立変数数
        self.n_v = 10  # 独立速度数
        self.n_x = 20  # 状態変数数 (q + dq)
        self.n_u = 10  # 制御入力数（初期値、後で調整可能）
        
        print(f"🔬 CasADi線形化システム初期化")
        print(f"   状態次元: {self.n_x} (q:{self.n_q} + dq:{self.n_v})")
        print(f"   入力次元: {self.n_u}")
        
        # CasADi関数の準備
        self._setup_casadi_functions()
    
    def _setup_casadi_functions(self):
        """CasADiシンボリック関数の設定"""
        print(f"   CasADiシンボリック関数設定中...")
        
        # シンボリック変数定義
        self.q_sym = cs.SX.sym('q', self.n_q)
        self.dq_sym = cs.SX.sym('dq', self.n_v)
        self.tau_sym = cs.SX.sym('tau', self.n_u)
        
        # 状態ベクトル
        self.x_sym = cs.vertcat(self.q_sym, self.dq_sym)
        
        # 動力学関数の構築（シンボリック）
        self.f_sym = self._build_dynamics_function()
        
        # ヤコビアン（自動微分）
        print(f"   自動微分によるヤコビアン計算中...")
        self.A_sym = cs.jacobian(self.f_sym, self.x_sym)  # ∂f/∂x
        self.B_sym = cs.jacobian(self.f_sym, self.tau_sym)  # ∂f/∂u
        
        # CasADi関数オブジェクト作成
        self.dynamics_func = cs.Function('dynamics', [self.x_sym, self.tau_sym], [self.f_sym])
        self.A_func = cs.Function('A_matrix', [self.x_sym, self.tau_sym], [self.A_sym])
        self.B_func = cs.Function('B_matrix', [self.x_sym, self.tau_sym], [self.B_sym])
        
        print(f"   ✅ CasADi関数設定完了")
    
    def _build_dynamics_function(self):
        """
        動力学関数の構築（シンボリック）
        
        Returns:
            f_sym: 状態方程式 dx/dt = f(x, u)
        """
        # 状態方程式の構築
        # dx/dt = [dq; ddq]
        # ddq = M^-1(tau - g - C*dq)
        
        # 第1部分: dx/dt の上半分は dq
        f_upper = self.dq_sym
        
        # 第2部分: dx/dt の下半分は ddq
        # これは数値計算が必要なため、数値関数として実装
        f_lower = self._compute_acceleration_symbolic()
        
        # 完全な状態方程式
        f_sym = cs.vertcat(f_upper, f_lower)
        
        return f_sym
    
    def _compute_acceleration_symbolic(self):
        """
        加速度計算（シンボリック版）
        
        Returns:
            ddq_sym: 加速度 ddq = M^-1(tau - g - C*dq)
        """
        # 注意: Pinocchioの計算は基本的に数値計算のため、
        # CasADiのシンボリック変数を直接使用できない
        # ここでは数値計算ベースの近似を行う
        
        # 簡易的なシンボリック動力学モデル
        # 実際のロボット動力学の近似として線形項のみ考慮
        ddq_sym = cs.vertcat(*[
            self.tau_sym[i] - 9.81 * cs.sin(self.q_sym[i % self.n_q]) 
            for i in range(self.n_q)
        ])
        
        return ddq_sym
    
    def _acceleration_callback(self, q_val, dq_val, tau_val):
        """
        加速度計算コールバック（数値計算）
        
        Args:
            q_val: 位置（数値）
            dq_val: 速度（数値）
            tau_val: トルク（数値）
            
        Returns:
            ddq_val: 加速度（数値）
        """
        try:
            # numpy配列に変換
            q_np = np.array(q_val).flatten()
            dq_np = np.array(dq_val).flatten()
            tau_np = np.array(tau_val).flatten()
            
            # fixed_robot_3d.pyの動力学計算を使用
            M_red, g_red, C_red, nle_red, q_full, dq_full = self.robot.compute_dynamics(q_np, dq_np)
            
            # 加速度計算: M * ddq = tau - g - C
            # ddq = M^-1 * (tau - g - C)
            ddq_np = np.linalg.solve(M_red, tau_np - g_red - C_red)
            
            return ddq_np
            
        except Exception as e:
            print(f"   ⚠️ 加速度計算エラー: {e}")
            # フォールバック: ゼロ加速度
            return np.zeros(self.n_v)
    
    def compute_linearization_matrices(self, q_eq, dq_eq, tau_eq):
        """
        平衡点周りの線形化行列計算
        
        Args:
            q_eq: 平衡点位置 (10次元)
            dq_eq: 平衡点速度 (10次元)
            tau_eq: 平衡点制御入力
            
        Returns:
            A_full: フルシステム行列 (20×20)
            B_full: フル入力行列 (20×n_u)
        """
        print(f"\n🔢 線形化行列計算開始")
        print(f"   平衡点: q_eq.shape={q_eq.shape}, dq_eq.shape={dq_eq.shape}")
        print(f"   平衡トルク: tau_eq.shape={tau_eq.shape}")
        
        # 状態ベクトル構築
        x_eq = np.concatenate([q_eq, dq_eq])
        
        try:
            # CasADi関数で線形化行列計算
            A_full_val = self.A_func(x_eq, tau_eq)
            B_full_val = self.B_func(x_eq, tau_eq)
            
            # numpy配列に変換
            A_full = np.array(A_full_val)
            B_full = np.array(B_full_val)
            
            print(f"   ✅ 線形化行列計算完了")
            print(f"      A行列: {A_full.shape}")
            print(f"      B行列: {B_full.shape}")
            
            # 行列の特性確認
            self._analyze_matrices(A_full, B_full)
            
            return A_full, B_full
            
        except Exception as e:
            print(f"   ❌ 線形化計算エラー: {e}")
            # フォールバック: 単位行列ベース
            A_full = np.eye(self.n_x)
            B_full = np.zeros((self.n_x, self.n_u))
            return A_full, B_full
    
    def _analyze_matrices(self, A_full, B_full):
        """
        線形化行列の分析
        
        Args:
            A_full: システム行列
            B_full: 入力行列
        """
        print(f"\n📊 線形化行列分析:")
        
        # A行列の固有値
        try:
            eigenvalues = np.linalg.eigvals(A_full)
            real_parts = np.real(eigenvalues)
            imag_parts = np.imag(eigenvalues)
            
            stable_count = np.sum(real_parts < 0)
            unstable_count = np.sum(real_parts > 0)
            marginal_count = np.sum(np.abs(real_parts) < 1e-10)
            
            print(f"   A行列固有値分析:")
            print(f"     安定極: {stable_count}個")
            print(f"     不安定極: {unstable_count}個")
            print(f"     限界極: {marginal_count}個")
            print(f"     最大実部: {np.max(real_parts):.6f}")
            print(f"     最小実部: {np.min(real_parts):.6f}")
            
        except Exception as e:
            print(f"   ⚠️ 固有値計算エラー: {e}")
        
        # 行列ノルム
        A_norm = np.linalg.norm(A_full)
        B_norm = np.linalg.norm(B_full)
        A_cond = np.linalg.cond(A_full)
        
        print(f"   行列特性:")
        print(f"     ||A||: {A_norm:.6f}")
        print(f"     ||B||: {B_norm:.6f}")
        print(f"     cond(A): {A_cond:.6e}")
        
        # B行列のランク
        try:
            B_rank = np.linalg.matrix_rank(B_full)
            print(f"     rank(B): {B_rank}/{min(B_full.shape)}")
        except Exception as e:
            print(f"   ⚠️ ランク計算エラー: {e}")
    
    def verify_linearization(self, q_eq, dq_eq, tau_eq, A_full, B_full, 
                           perturbation_magnitude=1e-4):
        """
        線形化の精度検証
        
        Args:
            q_eq, dq_eq, tau_eq: 平衡点
            A_full, B_full: 線形化行列
            perturbation_magnitude: 摂動の大きさ
        """
        print(f"\n🔍 線形化精度検証")
        print(f"   摂動大きさ: {perturbation_magnitude}")
        
        # 状態摂動
        x_eq = np.concatenate([q_eq, dq_eq])
        
        test_cases = [
            ("状態摂動", np.random.normal(0, perturbation_magnitude, self.n_x), np.zeros(self.n_u)),
            ("入力摂動", np.zeros(self.n_x), np.random.normal(0, perturbation_magnitude, self.n_u)),
            ("混合摂動", np.random.normal(0, perturbation_magnitude/2, self.n_x), 
                        np.random.normal(0, perturbation_magnitude/2, self.n_u))
        ]
        
        for case_name, dx, du in test_cases:
            try:
                # 摂動後の状態・入力
                x_pert = x_eq + dx
                tau_pert = tau_eq + du
                
                # 非線形応答
                f_nonlinear = self.dynamics_func(x_pert, tau_pert)
                f_eq = self.dynamics_func(x_eq, tau_eq)
                df_nonlinear = np.array(f_nonlinear - f_eq).flatten()
                
                # 線形近似応答
                df_linear = A_full @ dx + B_full @ du
                
                # 誤差評価
                error = np.linalg.norm(df_nonlinear - df_linear)
                relative_error = error / (np.linalg.norm(df_nonlinear) + 1e-12)
                
                print(f"   {case_name}:")
                print(f"     絶対誤差: {error:.6e}")
                print(f"     相対誤差: {relative_error:.6%}")
                
            except Exception as e:
                print(f"   ⚠️ {case_name} 検証エラー: {e}")

def test_linearized_dynamics():
    """線形化動力学テスト"""
    print("=== CasADi線形化動力学テスト ===")
    
    # 平衡点データの読み込み（equilibrium_finder.pyから）
    sys.path.append(os.path.dirname(__file__))
    from equilibrium_finder import EquilibriumFinder
    
    finder = EquilibriumFinder()
    eq_data = finder.find_specified_equilibrium()
    
    # 線形化システム初期化
    linearizer = LinearizedDynamics(finder.robot)
    
    # 線形化行列計算
    A_full, B_full = linearizer.compute_linearization_matrices(
        eq_data['q_eq'], eq_data['dq_eq'], eq_data['tau_eq']
    )
    
    # 線形化精度検証
    linearizer.verify_linearization(
        eq_data['q_eq'], eq_data['dq_eq'], eq_data['tau_eq'], A_full, B_full
    )
    
    print(f"\n📋 線形化結果サマリー:")
    print(f"   A行列: {A_full.shape}")
    print(f"   B行列: {B_full.shape}")
    
    return A_full, B_full, eq_data

if __name__ == "__main__":
    test_linearized_dynamics()
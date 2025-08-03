#!/usr/bin/env python3
"""
簡易線形化動力学システム
数値微分を使用したロボット動力学の線形化（CasADi不要版）
"""

import numpy as np
import os
import sys

# プロジェクトルートからfixed_robot_3dをインポート
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'urdf_full_robot_3d'))
from fixed_robot_3d import FixedRobot3D

class SimpleLinearizedDynamics:
    """数値微分による動力学線形化システム"""
    
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
        
        # 初期化完了
    
    def dynamics_function(self, x, u):
        """
        動力学関数 dx/dt = f(x, u)
        
        Args:
            x: 状態ベクトル [q; dq] (20次元)
            u: 制御入力 (10次元)
            
        Returns:
            f: 状態方程式 dx/dt
        """
        try:
            # 状態分解
            q = x[:self.n_q]
            dq = x[self.n_q:]
            
            # 動力学計算
            M_red, g_red, C_red, nle_red, q_full, dq_full = self.robot.compute_dynamics(q, dq)
            
            # 加速度計算: M * ddq = u - g - C
            ddq = np.linalg.solve(M_red, u - g_red - C_red)
            
            # 状態方程式: dx/dt = [dq; ddq]
            f = np.concatenate([dq, ddq])
            
            return f
            
        except Exception as e:
            pass
            # フォールバック: ゼロダイナミクス
            return np.zeros(self.n_x)
    
    def compute_linearization_matrices(self, q_eq, dq_eq, tau_eq, h=1e-8):
        """
        数値微分による線形化行列計算
        
        Args:
            q_eq: 平衡点位置 (10次元)
            dq_eq: 平衡点速度 (10次元)
            tau_eq: 平衡点制御入力
            h: 微分ステップサイズ
            
        Returns:
            A_full: フルシステム行列 (20×20)
            B_full: フル入力行列 (20×10)
        """
        
        # 状態ベクトル構築
        x_eq = np.concatenate([q_eq, dq_eq])
        
        # 平衡点での動力学
        f_eq = self.dynamics_function(x_eq, tau_eq)
        
        
        # A行列計算 (∂f/∂x)
        A_full = np.zeros((self.n_x, self.n_x))
        
        for i in range(self.n_x):
            # 前進差分
            x_plus = x_eq.copy()
            x_plus[i] += h
            f_plus = self.dynamics_function(x_plus, tau_eq)
            
            # 後退差分
            x_minus = x_eq.copy()
            x_minus[i] -= h
            f_minus = self.dynamics_function(x_minus, tau_eq)
            
            # 中央差分
            A_full[:, i] = (f_plus - f_minus) / (2 * h)
        
        # B行列計算 (∂f/∂u)
        B_full = np.zeros((self.n_x, self.n_u))
        
        for i in range(self.n_u):
            # 前進差分
            u_plus = tau_eq.copy()
            u_plus[i] += h
            f_plus = self.dynamics_function(x_eq, u_plus)
            
            # 後退差分
            u_minus = tau_eq.copy()
            u_minus[i] -= h
            f_minus = self.dynamics_function(x_eq, u_minus)
            
            # 中央差分
            B_full[:, i] = (f_plus - f_minus) / (2 * h)
        
        
        # 行列の特性確認
        self._analyze_matrices(A_full, B_full)
        
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
            
            stable_count = np.sum(real_parts < -1e-10)
            unstable_count = np.sum(real_parts > 1e-10)
            marginal_count = np.sum(np.abs(real_parts) <= 1e-10)
            
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
        
        print(f"   行列特性:")
        print(f"     ||A||: {A_norm:.6f}")
        print(f"     ||B||: {B_norm:.6f}")
        
        try:
            A_cond = np.linalg.cond(A_full)
            print(f"     cond(A): {A_cond:.6e}")
        except Exception as e:
            print(f"   ⚠️ 条件数計算エラー: {e}")
        
        # B行列のランク
        try:
            B_rank = np.linalg.matrix_rank(B_full)
            print(f"     rank(B): {B_rank}/{min(B_full.shape)}")
        except Exception as e:
            print(f"   ⚠️ ランク計算エラー: {e}")
    
    def verify_linearization(self, q_eq, dq_eq, tau_eq, A_full, B_full, 
                           perturbation_magnitude=1e-6):
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
                f_eq = self.dynamics_function(x_eq, tau_eq)
                f_pert = self.dynamics_function(x_pert, tau_pert)
                df_nonlinear = f_pert - f_eq
                
                # 線形近似応答
                df_linear = A_full @ dx + B_full @ du
                
                # 誤差評価
                error = np.linalg.norm(df_nonlinear - df_linear)
                nonlinear_norm = np.linalg.norm(df_nonlinear)
                relative_error = error / (nonlinear_norm + 1e-12)
                
                print(f"   {case_name}:")
                print(f"     絶対誤差: {error:.6e}")
                print(f"     相対誤差: {relative_error:.6%}")
                print(f"     非線形応答ノルム: {nonlinear_norm:.6e}")
                
            except Exception as e:
                print(f"   ⚠️ {case_name} 検証エラー: {e}")

def test_simple_linearized_dynamics():
    """簡易線形化動力学テスト"""
    print("=== 簡易線形化動力学テスト ===")
    
    # 平衡点データの読み込み（equilibrium_finder.pyから）
    sys.path.append(os.path.dirname(__file__))
    from equilibrium_finder import EquilibriumFinder
    
    finder = EquilibriumFinder()
    eq_data = finder.find_specified_equilibrium()
    
    # 線形化システム初期化
    linearizer = SimpleLinearizedDynamics(finder.robot)
    
    # 線形化行列計算（小さなステップサイズで高精度）
    A_full, B_full = linearizer.compute_linearization_matrices(
        eq_data['q_eq'], eq_data['dq_eq'], eq_data['tau_eq'], h=1e-7
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
    test_simple_linearized_dynamics()
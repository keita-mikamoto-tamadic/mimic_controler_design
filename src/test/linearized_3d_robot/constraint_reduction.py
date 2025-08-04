#!/usr/bin/env python3
"""
幾何学的拘束による線形変換システム
theta2 = -2 * theta1 拘束とトルク拘束の線形近似による縮約
"""

import numpy as np
import os
import sys

class ConstraintReduction:
    """幾何学的拘束による状態・入力空間の縮約"""
    
    def __init__(self):
        """初期化"""
        # 次元数定義
        self.n_full_q = 10    # フル位置次元（独立変数）
        self.n_full_v = 10    # フル速度次元
        self.n_full_x = 20    # フル状態次元 (q + dq)
        self.n_full_u = 6     # フル入力次元（実アクチュエータのみ）
        
        # 縮約後の次元数（実装中に決定）
        self.n_reduced_q = None
        self.n_reduced_v = None  
        self.n_reduced_x = None
        self.n_reduced_u = None
        
        print(f"🔗 幾何学的拘束縮約システム初期化")
        print(f"   フル次元: 状態={self.n_full_x}, 入力={self.n_full_u}")
        
        # 変換行列の計算
        self.T = self._compute_state_transformation_matrix()
        self.T_u = self._compute_input_transformation_matrix()
        
        print(f"   縮約次元: 状態={self.n_reduced_x}, 入力={self.n_reduced_u}")
    
    def _compute_state_transformation_matrix(self):
        """
        状態変換行列T計算: x = T * z
        
        幾何学的拘束 theta2 = -2 * theta1 を考慮した独立変数への変換
        
        Returns:
            T: 状態変換行列 (n_full_x × n_reduced_x)
        """
        print(f"   状態変換行列T計算中...")
        
        # フル状態変数の定義（fixed_robot_3d.py準拠）
        # [x_base, y_base, pitch, yaw, phi_L_lower, phi_R_lower, phi_L_upper, phi_R_upper, wheel_L, wheel_R]
        # [dx_base, dy_base, dpitch, dyaw, dphi_L_lower, dphi_R_lower, dphi_L_upper, dphi_R_upper, dwheel_L, dwheel_R]
        
        # 拘束関係の分析（urdf/01_ApproximationModel.md準拠）
        # theta2 = -2 * theta1 → lower = -2 * upper
        # phi_L_lower = -2 * phi_L_upper (拘束あり)
        # phi_R_lower = -2 * phi_R_upper (拘束あり)
        
        # 独立変数の選択（正しい）
        # 位置: [x_base, y_base, pitch, yaw, phi_L_upper, phi_R_upper, wheel_L, wheel_R] (8変数)
        # 速度: [dx_base, dy_base, dpitch, dyaw, dphi_L_upper, dphi_R_upper, dwheel_L, dwheel_R] (8変数)
        # 縮約状態: z = 16次元
        
        self.n_reduced_q = 8
        self.n_reduced_v = 8
        self.n_reduced_x = 16
        
        # 位置変換行列 T_q (10 × 8)
        T_q = np.zeros((self.n_full_q, self.n_reduced_q))
        
        # 独立変数のマッピング（修正版）
        # z_pos = [x_base, y_base, pitch, yaw, phi_L_upper, phi_R_upper, wheel_L, wheel_R]
        # x_pos = [x_base, y_base, pitch, yaw, phi_L_lower, phi_R_lower, phi_L_upper, phi_R_upper, wheel_L, wheel_R]
        
        # 直接コピー部分
        T_q[0, 0] = 1.0  # x_base
        T_q[1, 1] = 1.0  # y_base  
        T_q[2, 2] = 1.0  # pitch
        T_q[3, 3] = 1.0  # yaw
        T_q[6, 4] = 1.0  # phi_L_upper (独立変数)
        T_q[7, 5] = 1.0  # phi_R_upper (独立変数)
        T_q[8, 6] = 1.0  # wheel_L
        T_q[9, 7] = 1.0  # wheel_R
        
        # 拘束関係部分 (lower = -2 * upper) - 正しい方向
        T_q[4, 4] = -2.0  # phi_L_lower = -2 * phi_L_upper
        T_q[5, 5] = -2.0  # phi_R_lower = -2 * phi_R_upper
        
        # 速度変換行列 T_v (10 × 8) - 位置と同じ構造
        T_v = T_q.copy()
        
        # フル状態変換行列 T (20 × 16)
        T = np.zeros((self.n_full_x, self.n_reduced_x))
        T[:self.n_full_q, :self.n_reduced_q] = T_q  # 位置部分
        T[self.n_full_q:, self.n_reduced_q:] = T_v  # 速度部分
        
        print(f"     位置縮約: {self.n_full_q} → {self.n_reduced_q}")
        print(f"     速度縮約: {self.n_full_v} → {self.n_reduced_v}")
        print(f"     状態縮約: {self.n_full_x} → {self.n_reduced_x}")
        print(f"     拘束関係適用: phi_lower = -2 * phi_upper (正しい)")
        
        return T
    
    def _compute_input_transformation_matrix(self):
        """
        入力変換行列T_u計算: u_full = T_u * u
        
        トルク拘束 tau2 = f(theta1) * tau1 の線形近似を適用
        
        Returns:
            T_u: 入力変換行列 (n_full_u × n_reduced_u)
        """
        print(f"   入力変換行列T_u計算中...")
        
        # トルク拘束の線形近似（urdf/01_ApproximationModel.md準拠）
        # f(theta1) = -0.00214*theta1^2 + 0.2636*theta1 - 8.4662
        # 平衡点 theta1 = 52deg ≈ 0.907 rad での線形化
        
        theta1_eq = np.deg2rad(52.0)  # 平衡点
        f_eq = -0.00214 * theta1_eq**2 + 0.2636 * theta1_eq - 8.4662
        
        print(f"     平衡点角度: {theta1_eq:.4f} rad ({np.rad2deg(theta1_eq):.1f} deg)")
        print(f"     トルク係数: f(theta1_eq) = {f_eq:.3f}")
        
        # 独立制御入力の選択（実アクチュエータのみ）
        # フルトルク: [tau_L_lower, tau_R_lower, tau_L_upper, tau_R_upper, tau_wheel_L, tau_wheel_R] (6変数)
        # 拘束: tau_L_lower = f_eq * tau_L_upper, tau_R_lower = f_eq * tau_R_upper (正しい)
        # 独立入力: [tau_L_upper, tau_R_upper, tau_wheel_L, tau_wheel_R] (4変数)
        
        self.n_reduced_u = 4
        
        # 入力変換行列 T_u (6 × 4)
        T_u = np.zeros((self.n_full_u, self.n_reduced_u))
        
        # 独立入力のマッピング（Pinocchio関節順序準拠）
        # u_reduced = [tau_L_upper, tau_R_upper, tau_wheel_L, tau_wheel_R]
        # u_full = [tau_L_upper, tau_L_lower, tau_wheel_L, tau_R_upper, tau_R_lower, tau_wheel_R]
        #          (Pinocchio順序: [L_upper, L_lower, wheel_L, R_upper, R_lower, wheel_R])
        
        # 直接コピー部分（独立入力）
        T_u[0, 0] = 1.0  # tau_L_upper (独立入力)
        T_u[2, 2] = 1.0  # tau_wheel_L  
        T_u[3, 1] = 1.0  # tau_R_upper (独立入力)
        T_u[5, 3] = 1.0  # tau_wheel_R
        
        # トルク拘束関係 (lower = f_eq * upper) - 正しい方向
        T_u[1, 0] = f_eq  # tau_L_lower = f_eq * tau_L_upper
        T_u[4, 1] = f_eq  # tau_R_lower = f_eq * tau_R_upper
        
        print(f"     入力縮約: {self.n_full_u} → {self.n_reduced_u}")
        print(f"     トルク拘束適用: tau_lower = {f_eq:.3f} * tau_upper (正しい)")
        
        return T_u
    
    def compute_reduced_system(self, A_full, B_full):
        """
        縮約システム行列の計算
        
        Args:
            A_full: フルシステム行列 (20×20)
            B_full: フル入力行列 (20×10)
            
        Returns:
            A_reduced: 縮約システム行列
            B_reduced: 縮約入力行列
        """
        print(f"\n🔧 縮約システム行列計算")
        print(f"   入力: A_full={A_full.shape}, B_full={B_full.shape}")
        
        # 縮約システム: dz/dt = A_reduced*z + B_reduced*u
        # A_reduced = T^T * A_full * T
        # B_reduced = T^T * B_full * T_u
        
        try:
            # 元のB_fullが10次元なので、そのまま使用して関節トルク部分を抽出
            # B_full.shape=(20, 10): 10次元独立変数に対応
            # 最後の6次元が関節トルク: [4, 5, 6, 7, 8, 9]
            joint_indices = [4, 5, 6, 7, 8, 9]  # 関節トルク部分(6次元)
            B_joints = B_full[:, joint_indices]  # (20, 6)
            
            print(f"   関節トルクインデックス: {joint_indices}")
            print(f"   B行列縮約: {B_full.shape} → {B_joints.shape} (関節トルクのみ)")
            
            A_reduced = self.T.T @ A_full @ self.T
            B_reduced = self.T.T @ B_joints @ self.T_u
            
            print(f"   出力: A_reduced={A_reduced.shape}, B_reduced={B_reduced.shape}")
            
            # 縮約の効果分析
            self._analyze_reduction_effect(A_full, B_joints, A_reduced, B_reduced)
            
            return A_reduced, B_reduced
            
        except Exception as e:
            print(f"   ❌ 縮約計算エラー: {e}")
            # フォールバック
            A_reduced = np.eye(self.n_reduced_x)
            B_reduced = np.zeros((self.n_reduced_x, self.n_reduced_u))
            return A_reduced, B_reduced
    
    def _analyze_reduction_effect(self, A_full, B_full, A_reduced, B_reduced):
        """
        縮約効果の分析
        
        Args:
            A_full, B_full: フル行列
            A_reduced, B_reduced: 縮約行列
        """
        print(f"\n📊 縮約効果分析:")
        
        # 次元縮約率
        state_reduction_ratio = self.n_reduced_x / self.n_full_x
        input_reduction_ratio = self.n_reduced_u / self.n_full_u
        
        print(f"   次元縮約:")
        print(f"     状態: {self.n_full_x} → {self.n_reduced_x} ({state_reduction_ratio:.1%})")
        print(f"     入力: {self.n_full_u} → {self.n_reduced_u} ({input_reduction_ratio:.1%})")
        
        # 行列特性比較
        try:
            # 固有値比較（A行列）
            eigs_full = np.linalg.eigvals(A_full)
            eigs_reduced = np.linalg.eigvals(A_reduced)
            
            stable_full = np.sum(np.real(eigs_full) < 0)
            stable_reduced = np.sum(np.real(eigs_reduced) < 0)
            
            print(f"   固有値分析:")
            print(f"     フルシステム安定極: {stable_full}/{len(eigs_full)}")
            print(f"     縮約システム安定極: {stable_reduced}/{len(eigs_reduced)}")
            
        except Exception as e:
            print(f"   ⚠️ 固有値分析エラー: {e}")
        
        # 行列ノルム比較
        A_norm_ratio = np.linalg.norm(A_reduced) / np.linalg.norm(A_full)
        B_norm_ratio = np.linalg.norm(B_reduced) / np.linalg.norm(B_full)
        
        print(f"   行列ノルム比:")
        print(f"     ||A_reduced||/||A_full||: {A_norm_ratio:.3f}")
        print(f"     ||B_reduced||/||B_full||: {B_norm_ratio:.3f}")
    
    def expand_state(self, z_reduced):
        """
        縮約状態をフル状態に展開
        
        Args:
            z_reduced: 縮約状態 (n_reduced_x,)
            
        Returns:
            x_full: フル状態 (n_full_x,)
        """
        return self.T @ z_reduced
    
    def reduce_state(self, x_full):
        """
        フル状態を縮約状態に変換
        
        Args:
            x_full: フル状態 (n_full_x,)
            
        Returns:
            z_reduced: 縮約状態 (n_reduced_x,)
        """
        # 疑似逆行列を使用（拘束を満たす最小ノルム解）
        T_pinv = np.linalg.pinv(self.T)
        return T_pinv @ x_full
    
    def expand_input(self, u_reduced):
        """
        縮約入力をフル入力に展開
        
        Args:
            u_reduced: 縮約入力 (n_reduced_u,)
            
        Returns:
            u_full: フル入力 (n_full_u,)
        """
        return self.T_u @ u_reduced
    
    def reduce_input(self, u_full):
        """
        フル入力を縮約入力に変換
        
        Args:
            u_full: フル入力 (n_full_u,)
            
        Returns:
            u_reduced: 縮約入力 (n_reduced_u,)
        """
        # 疑似逆行列を使用
        T_u_pinv = np.linalg.pinv(self.T_u)
        return T_u_pinv @ u_full
    
    def get_transformation_info(self):
        """
        変換行列情報の取得
        
        Returns:
            dict: 変換情報
        """
        return {
            'T': self.T,
            'T_u': self.T_u,
            'n_full_x': self.n_full_x,
            'n_reduced_x': self.n_reduced_x,
            'n_full_u': self.n_full_u,
            'n_reduced_u': self.n_reduced_u,
            'state_reduction_ratio': self.n_reduced_x / self.n_full_x,
            'input_reduction_ratio': self.n_reduced_u / self.n_full_u
        }

def test_constraint_reduction():
    """拘束縮約システムテスト"""
    print("=== 幾何学的拘束縮約システムテスト ===")
    
    # 縮約システム初期化
    reducer = ConstraintReduction()
    
    # テスト用のダミー行列
    A_full = np.random.randn(20, 20) * 0.1
    B_full = np.random.randn(20, 10) * 0.1
    
    print(f"\n🧪 ダミー行列でのテスト:")
    print(f"   A_full: {A_full.shape}")
    print(f"   B_full: {B_full.shape}")
    
    # 縮約計算
    A_reduced, B_reduced = reducer.compute_reduced_system(A_full, B_full)
    
    # 状態変換テスト
    print(f"\n🔄 状態変換テスト:")
    x_full_test = np.random.randn(20)
    z_reduced_test = reducer.reduce_state(x_full_test)
    x_full_reconstructed = reducer.expand_state(z_reduced_test)
    
    # 拘束満足確認
    phi_L_lower_orig = x_full_test[4]
    phi_L_upper_orig = x_full_test[6]
    phi_L_lower_recon = x_full_reconstructed[4]
    phi_L_upper_recon = x_full_reconstructed[6]
    
    constraint_error = phi_L_upper_recon - (-2.0 * phi_L_lower_recon)
    
    print(f"   拘束満足確認:")
    print(f"     元データ: lower={phi_L_lower_orig:.4f}, upper={phi_L_upper_orig:.4f}")
    print(f"     復元データ: lower={phi_L_lower_recon:.4f}, upper={phi_L_upper_recon:.4f}")
    print(f"     拘束誤差: {constraint_error:.6e}")
    
    # 変換情報表示
    info = reducer.get_transformation_info()
    print(f"\n📋 変換情報サマリー:")
    for key, value in info.items():
        if isinstance(value, np.ndarray):
            print(f"   {key}: shape={value.shape}")
        else:
            print(f"   {key}: {value}")
    
    return reducer, A_reduced, B_reduced

if __name__ == "__main__":
    test_constraint_reduction()
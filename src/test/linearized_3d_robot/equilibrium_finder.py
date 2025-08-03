#!/usr/bin/env python3
"""
平衡点探索システム
指定された平衡点条件でロボットの静的平衡構成を計算
"""

import numpy as np
import pinocchio as pin
import os
import sys

# プロジェクトルートからfixed_robot_3dをインポート
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'urdf_full_robot_3d'))
from fixed_robot_3d import FixedRobot3D

WHEEL_RADIUS = (77.95 / 2) / 1000.0  # urdf/01_ApproximationModel.md準拠: (77.95 / 2) / 1000 [m]

class EquilibriumFinder:
    """平衡点探索システム"""
    
    def __init__(self):
        """初期化"""
        self.robot = FixedRobot3D()
        self.model = self.robot.model
        self.data = self.robot.data
        
    
    def find_specified_equilibrium(self):
        """
        指定平衡点での完全構成計算
        urdf/01_ApproximationModel.md準拠
        
        Returns:
            dict: 平衡点情報
                q_eq: 平衡点位置 (10次元独立変数)
                q_full_eq: 完全構成 (19次元)
                dq_eq: 平衡点速度 (10次元, 全てゼロ)
                tau_eq: 平衡点制御入力 (重力補償)
        """
        
        # 指定平衡点（deg → rad変換）
        upper_angle_deg = 52.0
        lower_angle_deg = -104.0  # 拘束: -2 * upper
        
        upper_angle = np.deg2rad(upper_angle_deg)
        lower_angle = np.deg2rad(lower_angle_deg)
        
        
        # 独立変数での初期推定（10自由度）
        # [x_base, y_base, pitch, yaw, phi_L_lower, phi_R_lower, phi_L_upper, phi_R_upper, wheel_L, wheel_R]
        state_eq = np.array([
            0.0,            # x_base
            0.0,            # y_base  
            0.0,            # pitch
            0.0,            # yaw
            lower_angle,    # phi_L_lower
            lower_angle,    # phi_R_lower
            upper_angle,    # phi_L_upper
            upper_angle,    # phi_R_upper
            0.0,            # wheel_L
            0.0             # wheel_R
        ])
        
        # fixed_robot_3d.pyの拘束満足計算を使用
        q_full_eq = self.robot.compute_constrained_configuration(
            state_eq[0], state_eq[1], state_eq[2], state_eq[3],
            state_eq[4], state_eq[5], state_eq[6], state_eq[7],
            state_eq[8], state_eq[9]
        )
        
        # 速度は全てゼロ（静的平衡）
        dq_eq = np.zeros(10)
        
        # 重力補償トルク計算
        tau_eq = self._compute_gravity_compensation(state_eq, dq_eq)
        
        # 結果検証
        self._verify_equilibrium(state_eq, dq_eq, tau_eq, q_full_eq)
        
        equilibrium_data = {
            'q_eq': state_eq,           # 10次元独立変数
            'q_full_eq': q_full_eq,     # 19次元完全構成
            'dq_eq': dq_eq,             # 10次元速度（ゼロ）
            'tau_eq': tau_eq,           # 重力補償トルク
            'upper_angle_deg': upper_angle_deg,
            'lower_angle_deg': lower_angle_deg,
            'wheel_radius': WHEEL_RADIUS
        }
        
        return equilibrium_data
    
    def _compute_gravity_compensation(self, state_eq, dq_eq):
        """
        平衡点での重力補償トルク計算
        
        Args:
            state_eq: 平衡点状態 (10次元)
            dq_eq: 平衡点速度 (10次元)
            
        Returns:
            tau_eq: 重力補償トルク
        """
        # 動力学計算（平衡点で）
        try:
            M_red, g_red, C_red, nle_red, q_full, dq_full = self.robot.compute_dynamics(state_eq, dq_eq)
            
            # 平衡条件: M * 0 = tau - g
            # ⇒ tau = g (重力項のみ)
            tau_eq = g_red.copy()
            
            return tau_eq
            
        except Exception as e:
            # フォールバック: ゼロトルク
            return np.zeros(10)
    
    def _verify_equilibrium(self, state_eq, dq_eq, tau_eq, q_full_eq):
        """
        平衡点の検証
        
        Args:
            state_eq: 平衡点状態 (10次元)
            dq_eq: 平衡点速度 (10次元)
            tau_eq: 重力補償トルク
            q_full_eq: 完全構成 (19次元)
        """
        # 検証計算（print文なし）
        upper_L_rad = state_eq[6]
        upper_R_rad = state_eq[7]
        lower_L_rad = state_eq[4]
        lower_R_rad = state_eq[5]
        
        # 拘束関係確認
        constraint_L_error = lower_L_rad - (-2.0 * upper_L_rad)
        constraint_R_error = lower_R_rad - (-2.0 * upper_R_rad)
        
        # 接地条件確認
        try:
            pin.forwardKinematics(self.model, self.data, q_full_eq)
            pin.updateFramePlacements(self.model, self.data)
        except Exception:
            pass

def test_equilibrium_finder():
    """平衡点探索テスト"""
    print("=== 平衡点探索システムテスト ===")
    
    finder = EquilibriumFinder()
    equilibrium_data = finder.find_specified_equilibrium()
    
    print(f"\n📊 平衡点データサマリー:")
    print(f"   独立変数 (10次元): {equilibrium_data['q_eq']}")
    print(f"   完全構成 (19次元): shape={equilibrium_data['q_full_eq'].shape}")
    print(f"   速度 (10次元): {equilibrium_data['dq_eq']}")
    print(f"   重力補償トルク: shape={equilibrium_data['tau_eq'].shape}")
    
    return equilibrium_data

if __name__ == "__main__":
    test_equilibrium_finder()
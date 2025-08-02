#!/usr/bin/env python3
"""
修正版3Dロボット動力学
full_robot_dynamicsの正確な実装を3Dに拡張
"""

import pinocchio as pin
import numpy as np
import os

WHEEL_RADIUS = 0.075  # m

class FixedRobot3D:
    """修正版3Dロボット（full_robot_dynamicsに忠実）"""
    
    def __init__(self):
        # モデル読み込み（フローティングベース必須！）
        self.model, self.data = self.load_model()
        
        # ホイール関節の特定
        self.wheel_indices = self._find_wheel_indices()
        
        print(f"🤖 修正版3Dロボット初期化")
        print(f"   自由度: nv={self.model.nv}, nq={self.model.nq}")
        print(f"   ホイール関節: {list(self.wheel_indices.keys())}")
        print(f"   重力: {self.model.gravity.linear}")
    
    def load_model(self):
        """URDFモデルの読み込み"""
        current_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.join(current_dir, "..", "..", "..")
        urdf_path = os.path.join(project_root, "urdf", "mimic_v1.urdf")
        
        # フローティングベース必須！
        model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
        data = model.createData()
        
        # 重力設定（重要：正の値！）
        model.gravity.linear = np.array([0, 0, +9.81])
        
        return model, data
    
    def _find_wheel_indices(self):
        """ホイール関節のインデックスを特定"""
        wheel_indices = {}
        for i in range(self.model.njoints):
            joint_name = self.model.names[i]
            if 'wheel' in joint_name.lower():
                wheel_indices[joint_name] = i
        return wheel_indices
    
    def compute_constrained_configuration(self, x_base, y_base, yaw, 
                                        phi_L_upper, phi_L_lower, phi_R_upper, phi_R_lower):
        """
        full_robot_dynamicsと同じアプローチ
        拘束を満たす完全構成を計算
        """
        
        # 初期構成
        q = pin.neutral(self.model)
        
        # ベース位置・姿勢（仮設定）
        q[0] = x_base  # X
        q[1] = y_base  # Y
        # q[2] は後で拘束から計算
        q[5] = yaw     # Yaw回転
        
        # 関節角度設定（RUBY表現）
        # upper_link_L (joint index 2)
        q[7] = np.cos(phi_L_upper)
        q[8] = np.sin(phi_L_upper)
        # lower_link_L (joint index 3)
        q[9] = np.cos(phi_L_lower)
        q[10] = np.sin(phi_L_lower)
        # wheel_L (joint index 4) - 仮値
        q[11] = 1.0
        q[12] = 0.0
        
        # upper_link_R (joint index 5)
        q[13] = np.cos(phi_R_upper)
        q[14] = np.sin(phi_R_upper)
        # lower_link_R (joint index 6)
        q[15] = np.cos(phi_R_lower)
        q[16] = np.sin(phi_R_lower)
        # wheel_R (joint index 7) - 仮値
        q[17] = 1.0
        q[18] = 0.0
        
        # 順運動学でホイール位置計算
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        
        # 両ホイールの接地点高度を取得
        wheel_contact_heights = []
        for wheel_name, joint_idx in self.wheel_indices.items():
            wheel_center_z = self.data.oMi[joint_idx].translation[2]
            contact_z = wheel_center_z - WHEEL_RADIUS
            wheel_contact_heights.append(contact_z)
        
        # 接地拘束：両ホイールの平均接地点がZ=0になるようベース高度調整
        avg_contact_height = np.mean(wheel_contact_heights)
        base_height = -avg_contact_height
        
        q[2] = base_height  # ベースZ位置を拘束から決定
        
        return q
    
    def compute_dynamics(self, state, velocity):
        """
        7自由度動力学計算（full_robot_dynamicsと同じ）
        
        独立変数: [x_base, y_base, yaw, phi_L_upper, phi_L_lower, phi_R_upper, phi_R_lower]
        """
        
        # 状態変数展開
        x_base, y_base, yaw, phi_L_upper, phi_L_lower, phi_R_upper, phi_R_lower = state
        dx_base, dy_base, dyaw, dphi_L_upper, dphi_L_lower, dphi_R_upper, dphi_R_lower = velocity
        
        # 完全構成計算
        q = self.compute_constrained_configuration(x_base, y_base, yaw, 
                                                  phi_L_upper, phi_L_lower, phi_R_upper, phi_R_lower)
        
        # 完全速度構築（重要：正しいインデックス！）
        dq = np.zeros(self.model.nv)
        dq[0] = dx_base     # dx
        dq[1] = dy_base     # dy
        # dq[2] = dz_base   # 拘束から計算
        dq[5] = dyaw        # dyaw
        dq[6] = dphi_L_upper  # left upper
        dq[7] = dphi_L_lower  # left lower
        # dq[8] = wheel_L     # 従属
        dq[9] = dphi_R_upper  # right upper  
        dq[10] = dphi_R_lower # right lower
        # dq[11] = wheel_R    # 従属
        
        # 接地拘束からベースZ速度を計算
        pin.computeJointJacobians(self.model, self.data, q)
        
        # 左右ホイールの接地点Z速度=0の拘束
        constraint_jacobians = []
        for wheel_name, joint_idx in self.wheel_indices.items():
            J_wheel = pin.getFrameJacobian(self.model, self.data, joint_idx, pin.ReferenceFrame.WORLD)
            J_contact_z = J_wheel[2, :]  # Z方向
            constraint_jacobians.append(J_contact_z)
        
        # 平均拘束
        J_avg = np.mean(constraint_jacobians, axis=0)
        
        # dz_baseを計算: J_avg @ dq = 0
        if abs(J_avg[2]) > 1e-6:
            other_contribution = (J_avg[0] * dx_base + J_avg[1] * dy_base + J_avg[5] * dyaw +
                                J_avg[6] * dphi_L_upper + J_avg[7] * dphi_L_lower +
                                J_avg[9] * dphi_R_upper + J_avg[10] * dphi_R_lower)
            dq[2] = -other_contribution / J_avg[2]
        
        # ホイール速度も拘束から計算（簡略化）
        dq[8] = 0.0   # wheel_L
        dq[11] = 0.0  # wheel_R
        
        # 動力学計算
        M = pin.crba(self.model, self.data, q)
        g = pin.computeGeneralizedGravity(self.model, self.data, q)
        C = np.zeros(self.model.nv)  # コリオリ項
        if np.linalg.norm(dq) > 1e-6:
            pin.computeCoriolisMatrix(self.model, self.data, q, dq)
            C = self.data.C @ dq
        
        # 7×7縮約: [x, y, yaw, phi_L_upper, phi_L_lower, phi_R_upper, phi_R_lower]
        free_indices = [0, 1, 5, 6, 7, 9, 10]
        M_red = M[np.ix_(free_indices, free_indices)]
        g_red = g[free_indices]
        C_red = C[free_indices]
        
        return M_red, g_red, C_red, q, dq
    
    def simulate(self, initial_state, T_sim=2.0, dt=0.01):
        """動力学シミュレーション（full_robot_dynamicsと同じ）"""
        
        print(f"\n🚀 シミュレーション開始")
        print(f"   初期状態: {initial_state}")
        print(f"   時間: {T_sim}s, dt: {dt}s")
        
        # 初期化
        state = np.array(initial_state, dtype=float)
        velocity = np.zeros(7)  # 初期速度ゼロ
        
        t_array = np.arange(0, T_sim, dt)
        N = len(t_array)
        
        # 履歴
        time_history = []
        state_history = []
        velocity_history = []
        full_config_history = []
        
        print(f"   総ステップ数: {N}")
        
        # シミュレーションループ
        for i, t in enumerate(t_array):
            # 履歴記録
            time_history.append(t)
            state_history.append(state.copy())
            velocity_history.append(velocity.copy())
            
            try:
                # 動力学計算
                M_red, g_red, C_red, q_full, dq_full = self.compute_dynamics(state, velocity)
                full_config_history.append(q_full.copy())
                
                # 運動方程式: M * ddq = tau - g - C
                tau = np.zeros(7)  # 無制御（重力のみ）
                
                # 加速度計算
                acceleration = np.linalg.solve(M_red, tau - g_red - C_red)
                
                # 積分
                velocity += acceleration * dt
                state += velocity * dt
                
                # 10ステップごとに進捗表示
                if i % 10 == 0:
                    print(f"   t={t:.3f}s: 関節=[{state[3]:.2f}, {state[4]:.2f}, {state[5]:.2f}, {state[6]:.2f}]")
                
                # 発散チェック
                if np.any(np.abs(state[3:]) > 10.0):
                    print(f"   ⚠️ 発散検出 at t={t:.3f}s")
                    break
                    
            except Exception as e:
                print(f"   ❌ エラー at t={t:.3f}s: {e}")
                break
        
        print(f"   ✅ シミュレーション完了: {len(time_history)}ステップ")
        
        return {
            'time': np.array(time_history),
            'state': np.array(state_history),
            'velocity': np.array(velocity_history),
            'config': full_config_history
        }

def test_gravity_pendulum():
    """重力による振り子運動テスト"""
    print("=== 修正版3Dロボット重力テスト ===")
    
    # システム初期化
    robot = FixedRobot3D()
    
    # 初期状態（左脚を少し前に）
    initial_state = [
        0.0, 0.0, 0.0,    # x, y, yaw
        0.3, -0.8,        # 左脚: upper, lower
        0.5, -1.0         # 右脚: upper, lower
    ]
    
    # シミュレーション実行
    results = robot.simulate(initial_state, T_sim=1.0, dt=0.01)
    
    # 結果分析
    print(f"\n📊 結果分析:")
    print(f"   シミュレーション時間: {results['time'][-1]:.3f}s")
    
    # 関節角度の変化
    initial_joints = results['state'][0, 3:]
    final_joints = results['state'][-1, 3:]
    joint_change = final_joints - initial_joints
    
    print(f"   初期関節角度: {initial_joints}")
    print(f"   最終関節角度: {final_joints}")
    print(f"   関節変化量: {joint_change}")
    
    # 振り子運動の判定
    max_change = np.max(np.abs(joint_change))
    if max_change > 0.1:
        print(f"   ✅ 振り子運動検出（最大変化: {max_change:.3f} rad）")
    else:
        print(f"   ⚠️ 運動が小さすぎる（最大変化: {max_change:.3f} rad）")
    
    return results

if __name__ == "__main__":
    test_gravity_pendulum()
#!/usr/bin/env python3
"""
3Dフルロボット動力学シミュレーション
full_robot_dynamicsの成功手法を3Dに拡張
"""

import pinocchio as pin
import numpy as np
import os

WHEEL_RADIUS = 0.075  # m (3Dモデル用に更新)

class Robot3DDynamics:
    """3Dフルロボット動力学（simple_testアプローチ）"""
    
    def __init__(self):
        # モデル読み込み
        self.model, self.data = self.load_model()
        
        # ホイール関節の特定
        self.wheel_indices = self._find_wheel_indices()
        
        # 重力設定
        self.model.gravity.linear = np.array([0, 0, -9.81])
        
        print(f"🤖 3Dロボット動力学システム初期化")
        print(f"   自由度: nv={self.model.nv}, nq={self.model.nq}")
        print(f"   ホイール関節: {list(self.wheel_indices.keys())}")
    
    def load_model(self):
        """URDFモデルの読み込み"""
        current_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.join(current_dir, "..", "..", "..")
        urdf_path = os.path.join(project_root, "urdf", "mimic_v1.urdf")
        
        # 固定ベースモデルとして読み込み（フローティングベース不要）
        model = pin.buildModelFromUrdf(urdf_path)
        data = model.createData()
        
        return model, data
    
    def _find_wheel_indices(self):
        """ホイール関節のインデックスを特定"""
        wheel_indices = {}
        for i in range(self.model.njoints):
            joint_name = self.model.names[i]
            if 'wheel' in joint_name.lower():
                wheel_indices[joint_name] = i
        return wheel_indices
    
    def compute_constrained_configuration(self, x_base, y_base, z_base, roll, pitch, yaw, 
                                        phi_L_upper, phi_L_lower, phi_R_upper, phi_R_lower):
        """
        simple_testアプローチ：拘束を満たす完全構成を計算
        
        3D拡張版：
        - 独立変数: [x, y, z, roll, pitch, yaw, 4関節] (10自由度)
        - 拘束: 両ホイール接地 (2拘束)
        - 実効自由度: 8自由度
        """
        
        # 初期構成
        q = pin.neutral(self.model)
        
        # 関節角度設定
        # mimic_v1.urdfの構造に基づいて設定
        # continuous jointは自動的にcos/sin表現になる
        
        # 左脚
        if self.model.nq > 12:  # RUBY表現の場合
            q[7] = np.cos(phi_L_upper)
            q[8] = np.sin(phi_L_upper)
            q[9] = np.cos(phi_L_lower)
            q[10] = np.sin(phi_L_lower)
            # ホイール角度は後で計算
            
            # 右脚
            q[13] = np.cos(phi_R_upper)
            q[14] = np.sin(phi_R_upper)
            q[15] = np.cos(phi_R_lower)
            q[16] = np.sin(phi_R_lower)
        else:
            # 単純な角度表現の場合
            joint_idx = 0
            for i in range(1, self.model.njoints):
                if 'upper_link_L' in self.model.names[i]:
                    q[joint_idx] = phi_L_upper
                elif 'lower_link_L' in self.model.names[i]:
                    q[joint_idx] = phi_L_lower
                elif 'upper_link_R' in self.model.names[i]:
                    q[joint_idx] = phi_R_upper
                elif 'lower_link_R' in self.model.names[i]:
                    q[joint_idx] = phi_R_lower
                joint_idx += 1
        
        # 順運動学でホイール位置計算
        pin.forwardKinematics(self.model, self.data, q)
        
        # 両ホイールの位置を取得
        wheel_positions = {}
        for wheel_name, joint_idx in self.wheel_indices.items():
            wheel_pos = self.data.oMi[joint_idx].translation
            wheel_positions[wheel_name] = wheel_pos
        
        # 3D拘束: 両ホイールが地面に接地
        # ここでは簡易的に平均高さを使用
        wheel_heights = [pos[2] - WHEEL_RADIUS for pos in wheel_positions.values()]
        avg_wheel_height = np.mean(wheel_heights)
        
        # ベース変換を構築（3D版）
        # 実際のロボットではベース位置を調整する必要がある
        # ここでは簡略化のため、高さ調整のみ
        base_adjustment_z = -avg_wheel_height
        
        return q, base_adjustment_z, wheel_positions
    
    def compute_dynamics(self, state, velocity):
        """
        3D動力学計算
        
        state: [x, y, z, roll, pitch, yaw, 4関節] (10次元)
        velocity: 対応する速度 (10次元)
        """
        
        # 状態変数展開
        x, y, z, roll, pitch, yaw = state[:6]
        phi_L_upper, phi_L_lower, phi_R_upper, phi_R_lower = state[6:10]
        
        # 速度展開
        dx, dy, dz, droll, dpitch, dyaw = velocity[:6]
        dphi_L_upper, dphi_L_lower, dphi_R_upper, dphi_R_lower = velocity[6:10]
        
        # 拘束を満たす構成を計算
        q, z_adjustment, wheel_pos = self.compute_constrained_configuration(
            x, y, z, roll, pitch, yaw, 
            phi_L_upper, phi_L_lower, phi_R_upper, phi_R_lower
        )
        
        # 速度ベクトル構築
        dq = np.zeros(self.model.nv)
        # 関節速度の設定（モデル構造に依存）
        if self.model.nv >= 6:
            dq[0] = dphi_L_upper
            dq[1] = dphi_L_lower
            # dq[2] = wheel_L (従属)
            dq[3] = dphi_R_upper
            dq[4] = dphi_R_lower
            # dq[5] = wheel_R (従属)
        
        # 接地拘束からホイール速度を計算
        # 簡略化：ホイール速度は0とする
        
        # 動力学計算
        M = pin.crba(self.model, self.data, q)
        g = pin.computeGeneralizedGravity(self.model, self.data, q)
        
        # コリオリ項（速度依存）
        C = np.zeros(self.model.nv)
        if np.linalg.norm(dq) > 1e-6:
            pin.computeCoriolisMatrix(self.model, self.data, q, dq)
            C = self.data.C @ dq
        
        return M, g, C, q
    
    def simulate(self, initial_state, T_sim=2.0, dt=0.01):
        """
        動力学シミュレーション実行
        """
        print(f"\n🚀 3Dシミュレーション開始")
        print(f"   初期状態: {initial_state}")
        print(f"   シミュレーション時間: {T_sim}s")
        
        # 初期化
        state = np.array(initial_state, dtype=float)
        velocity = np.zeros(10)
        
        # 時間配列
        t_array = np.arange(0, T_sim, dt)
        N = len(t_array)
        
        # 履歴
        time_history = []
        state_history = []
        velocity_history = []
        config_history = []
        
        # シミュレーションループ
        for i, t in enumerate(t_array):
            # 記録
            time_history.append(t)
            state_history.append(state.copy())
            velocity_history.append(velocity.copy())
            
            try:
                # 動力学計算
                M, g, C, q = self.compute_dynamics(state, velocity)
                config_history.append(q.copy())
                
                # 簡略化：関節のみの動力学
                # 実際にはベース動力学も含める必要がある
                n_joints = 4
                M_joints = M[:n_joints, :n_joints]
                g_joints = g[:n_joints]
                C_joints = C[:n_joints] if len(C) >= n_joints else np.zeros(n_joints)
                
                # 運動方程式: M * ddq = tau - g - C
                tau = np.zeros(n_joints)  # 無制御
                
                # 加速度計算
                if np.linalg.cond(M_joints) < 1e10:
                    ddq_joints = np.linalg.solve(M_joints, tau - g_joints - C_joints)
                else:
                    print(f"   ⚠️ t={t:.3f}s: 特異行列")
                    ddq_joints = np.zeros(n_joints)
                
                # 積分（関節のみ）
                velocity[6:10] += ddq_joints * dt
                state[6:10] += velocity[6:10] * dt
                
                # ベース運動（簡略化：固定）
                # 実際には拘束力を考慮する必要がある
                
                # 発散チェック
                if np.any(np.abs(state[6:10]) > 10.0):
                    print(f"   ⚠️ シミュレーション発散 (t={t:.3f}s)")
                    break
                    
            except Exception as e:
                print(f"   ❌ エラー at t={t:.3f}s: {e}")
                break
        
        print(f"   ✅ シミュレーション完了: {len(time_history)}ステップ")
        
        return {
            'time': np.array(time_history),
            'state': np.array(state_history),
            'velocity': np.array(velocity_history),
            'config': config_history
        }

def test_basic_simulation():
    """基本的なシミュレーションテスト"""
    print("=== 3Dロボット動力学テスト ===")
    
    # システム初期化
    robot = Robot3DDynamics()
    
    # 初期状態（立位）
    initial_state = [
        0.0, 0.0, 0.3,  # x, y, z
        0.0, 0.0, 0.0,  # roll, pitch, yaw
        0.5, -1.0,      # 左脚: upper, lower
        0.5, -1.0       # 右脚: upper, lower
    ]
    
    # シミュレーション実行
    results = robot.simulate(initial_state, T_sim=1.0, dt=0.01)
    
    # 結果表示
    print(f"\n📊 シミュレーション結果:")
    print(f"   最終時刻: {results['time'][-1]:.3f}s")
    print(f"   最終関節角度: {results['state'][-1, 6:10]}")
    
    # 関節角度の変化量
    joint_change = results['state'][-1, 6:10] - results['state'][0, 6:10]
    print(f"   関節角度変化: {joint_change}")
    
    # 振り子運動の確認
    if np.any(np.abs(joint_change) > 0.1):
        print(f"   ✅ 振り子運動を検出")
    else:
        print(f"   ⚠️ 運動が小さい")
    
    return results

if __name__ == "__main__":
    test_basic_simulation()
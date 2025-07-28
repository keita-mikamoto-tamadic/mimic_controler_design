#!/usr/bin/env python3
"""
修正版動力学シミュレータ
single_legの成功手法をフルロボットに正しく適用
"""

import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from basic_loader import FullRobotLoader

WHEEL_RADIUS = (77.95 / 2) / 1000  # [m]

class FixedFullRobotSimulator:
    """修正版フルロボットシミュレータ（simple_testアプローチ）"""
    
    def __init__(self):
        # 基本コンポーネント初期化
        self.loader = FullRobotLoader()
        self.model, self.data = self.loader.load_model()
        
        # ホイール関節の特定
        self.wheel_indices = self._find_wheel_indices()
        
        print(f"🔧 修正版フルロボットシミュレータ初期化")
        print(f"   ホイール関節: {list(self.wheel_indices.keys())}")
    
    def _find_wheel_indices(self):
        """ホイール関節のインデックスを特定"""
        wheel_indices = {}
        for i in range(self.model.njoints):
            joint_name = self.model.names[i]
            if 'wheel' in joint_name.lower():
                wheel_indices[joint_name] = i
        return wheel_indices
    
    def compute_constrained_configuration(self, x_base, y_base, yaw, phi_L_upper, phi_L_lower, phi_R_upper, phi_R_lower):
        """simple_testアプローチ：拘束を満たす完全構成を計算"""
        
        # 初期構成
        q = pin.neutral(self.model)
        
        # ベース位置・姿勢（仮設定）
        q[0] = x_base  # X
        q[1] = y_base  # Y
        # q[2] は後で拘束から計算
        q[5] = yaw     # Yaw回転
        
        # 関節角度設定（RUBY表現）
        q[7] = np.cos(phi_L_upper)    # upper_link_L cos
        q[8] = np.sin(phi_L_upper)    # upper_link_L sin
        q[9] = np.cos(phi_L_lower)    # lower_link_L cos  
        q[10] = np.sin(phi_L_lower)   # lower_link_L sin
        q[11] = 1.0                   # wheel_L cos（仮）
        q[12] = 0.0                   # wheel_L sin（仮）
        
        q[13] = np.cos(phi_R_upper)   # upper_link_R cos
        q[14] = np.sin(phi_R_upper)   # upper_link_R sin
        q[15] = np.cos(phi_R_lower)   # lower_link_R cos
        q[16] = np.sin(phi_R_lower)   # lower_link_R sin
        q[17] = 1.0                   # wheel_R cos（仮）
        q[18] = 0.0                   # wheel_R sin（仮）
        
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
        base_height = -avg_contact_height  # 接地点をZ=0にするためのオフセット
        
        q[2] = base_height  # ベースZ位置を拘束から決定
        
        return q
    
    def compute_6dof_dynamics(self, state, velocity):
        """6自由度動力学計算（simple_testアプローチ）
        
        独立変数: [x_base, y_base, yaw, phi_L_upper, phi_L_lower, phi_R_upper, phi_R_lower] (7次元)
        → ベースZ位置は拘束により決定
        """
        
        # 状態変数展開
        x_base, y_base, yaw, phi_L_upper, phi_L_lower, phi_R_upper, phi_R_lower = state
        dx_base, dy_base, dyaw, dphi_L_upper, dphi_L_lower, dphi_R_upper, dphi_R_lower = velocity
        
        # 完全構成計算
        q = self.compute_constrained_configuration(x_base, y_base, yaw, phi_L_upper, phi_L_lower, phi_R_upper, phi_R_lower)
        
        # 完全速度構築
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
        
        # 接地拘束からベースZ速度を計算（single_legと同様）
        pin.computeJointJacobians(self.model, self.data, q)
        
        # 左右ホイールの接地点Z速度=0の拘束
        constraint_jacobians = []
        for wheel_name, joint_idx in self.wheel_indices.items():
            J_wheel = pin.getFrameJacobian(self.model, self.data, joint_idx, pin.ReferenceFrame.WORLD)
            J_contact_z = J_wheel[2, :]  # Z方向
            constraint_jacobians.append(J_contact_z)
        
        # 平均拘束（両足の接地点平均がZ=0）
        J_avg = np.mean(constraint_jacobians, axis=0)
        
        # dz_baseを計算: J_avg @ dq = 0
        if abs(J_avg[2]) > 1e-6:  # dz成分が0でない
            other_contribution = (J_avg[0] * dx_base + J_avg[1] * dy_base + J_avg[5] * dyaw +
                                J_avg[6] * dphi_L_upper + J_avg[7] * dphi_L_lower +
                                J_avg[9] * dphi_R_upper + J_avg[10] * dphi_R_lower)
            dq[2] = -other_contribution / J_avg[2]
        
        # ホイール速度も拘束から計算（簡略化）
        dq[8] = 0.0   # wheel_L
        dq[11] = 0.0  # wheel_R
        
        # 動力学計算
        pin.crba(self.model, self.data, q)
        pin.computeGeneralizedGravity(self.model, self.data, q)
        pin.computeCoriolisMatrix(self.model, self.data, q, dq)
        
        M = self.data.M
        g = self.data.g
        C = self.data.C @ dq
        
        # 7×7縮約: [x, y, yaw, phi_L_upper, phi_L_lower, phi_R_upper, phi_R_lower]
        free_indices = [0, 1, 5, 6, 7, 9, 10]
        M_red = M[np.ix_(free_indices, free_indices)]
        g_red = g[free_indices]
        C_red = C[free_indices]
        
        return M_red, g_red, C_red, q, dq
    
    def simulate_falling_robot(self, initial_state, T_sim=3.0, dt=0.02):
        """重力による倒れるロボットのシミュレーション"""
        print(f"🎬 重力落下シミュレーション開始")
        print(f"   初期状態: {initial_state}")
        print(f"   時間: {T_sim}s, dt: {dt}s")
        
        # 初期化
        state = np.array(initial_state, dtype=float)
        velocity = np.zeros(7)  # 初期速度ゼロ
        
        t_array = np.arange(0, T_sim, dt)
        N = len(t_array)
        
        # 履歴保存
        time_history = []
        state_history = []
        velocity_history = []
        full_config_history = []
        joint_positions_history = []
        
        print(f"   総ステップ数: {N}")
        
        # シミュレーションループ
        for i, t in enumerate(t_array):
            # 履歴記録
            time_history.append(t)
            state_history.append(state.copy())
            velocity_history.append(velocity.copy())
            
            # 動力学計算
            try:
                M_red, g_red, C_red, q_full, dq_full = self.compute_6dof_dynamics(state, velocity)
                full_config_history.append(q_full.copy())
                
                # 関節位置記録
                positions, _ = self.loader.get_joint_positions(q_full)
                joint_positions_history.append(positions)
                
                # 運動方程式: M * ddq = tau - g - C
                tau = np.zeros(7)  # 無制御（重力のみ）
                
                # 加速度計算
                acceleration = np.linalg.solve(M_red, tau - g_red - C_red)
                
            except Exception as e:
                print(f"   ⚠️  t={t:.3f}s: 動力学エラー {e}")
                acceleration = np.zeros(7)
            
            # 進行状況表示
            if i % 25 == 0:
                base_height = q_full[2] if 'q_full' in locals() else 0
                print(f"   t={t:.2f}s: x={state[0]:.3f}, z={base_height:.3f}, φL=({state[3]:.2f},{state[4]:.2f}), φR=({state[5]:.2f},{state[6]:.2f})")
            
            # 積分
            velocity += acceleration * dt
            state += velocity * dt
            
            # 発散チェック
            if not (np.all(np.isfinite(state)) and np.all(np.isfinite(velocity))):
                print(f"   ❌ 発散検出 at t={t:.3f}")
                break
        
        print(f"✅ シミュレーション完了")
        print(f"   最終位置: x={state[0]:.3f}, y={state[1]:.3f}")
        print(f"   最終ベース高度: {full_config_history[-1][2]:.3f}m")
        
        return {
            'time': np.array(time_history),
            'states': np.array(state_history),
            'velocities': np.array(velocity_history),
            'full_configs': np.array(full_config_history),
            'joint_positions': joint_positions_history
        }
    
    def create_falling_animation(self, results, title="Falling Robot"):
        """倒れるロボットのアニメーション作成"""
        print(f"🎨 倒れるロボットアニメーション作成: {title}")
        
        time = results['time']
        joint_positions_array = np.array(results['joint_positions'])
        n_frames = len(time)
        
        # フィギュア・軸設定
        fig, ax = plt.subplots(figsize=(12, 8))
        
        # 表示範囲設定
        all_x = joint_positions_array[:, :, 0].flatten()
        all_z = joint_positions_array[:, :, 2].flatten()
        
        x_min, x_max = np.min(all_x) - 0.2, np.max(all_x) + 0.2
        z_min, z_max = np.min(all_z) - 0.1, np.max(all_z) + 0.2
        
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(z_min, z_max)
        ax.set_xlabel('X Position [m]', fontsize=12)
        ax.set_ylabel('Z Position [m]', fontsize=12)
        ax.set_title(title, fontsize=14)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        # 地面
        ax.axhline(y=0, color='brown', linewidth=3, alpha=0.8, label='Ground')
        
        # ロボット描画要素
        robot_lines, = ax.plot([], [], 'b-', linewidth=4, marker='o', markersize=8, label='Robot')
        
        # 左右ホイール
        left_wheel = plt.Circle((0, 0), WHEEL_RADIUS, fill=False, color='blue', linewidth=2)
        right_wheel = plt.Circle((0, 0), WHEEL_RADIUS, fill=False, color='red', linewidth=2)
        ax.add_patch(left_wheel)
        ax.add_patch(right_wheel)
        
        # ベース軌跡
        base_trajectory_x, base_trajectory_z = [], []
        trajectory_line, = ax.plot([], [], 'r--', alpha=0.5, linewidth=1, label='Base Trajectory')
        
        # 時間表示
        time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, fontsize=12,
                           verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat'))
        
        ax.legend()
        
        def animate(frame):
            if frame >= n_frames:
                frame = n_frames - 1
            
            positions = joint_positions_array[frame]
            
            # ロボット骨格描画（正しい接続順序）
            # 左脚: ベース→上腿→下腿→ホイール
            left_leg_x = [positions[0][0], positions[1][0], positions[2][0], positions[3][0]]
            left_leg_z = [positions[0][2], positions[1][2], positions[2][2], positions[3][2]]
            
            # 右脚: ベース→上腿→下腿→ホイール  
            right_leg_x = [positions[0][0], positions[4][0], positions[5][0], positions[6][0]]
            right_leg_z = [positions[0][2], positions[4][2], positions[5][2], positions[6][2]]
            
            # 両脚を結合して描画
            all_x = left_leg_x + [None] + right_leg_x  # Noneで線を分割
            all_z = left_leg_z + [None] + right_leg_z
            robot_lines.set_data(all_x, all_z)
            
            # ホイール位置更新（正しいインデックス）
            # joint index: 4=wheel_L_joint, 7=wheel_R_joint
            # positions配列では: [0=root, 1=upper_L, 2=lower_L, 3=wheel_L, 4=upper_R, 5=lower_R, 6=wheel_R]
            if len(positions) >= 7:
                wheel_L_pos = positions[3]  # wheel_L_joint (index 4 in model)
                wheel_R_pos = positions[6]  # wheel_R_joint (index 7 in model)
                left_wheel.center = (wheel_L_pos[0], wheel_L_pos[2])
                right_wheel.center = (wheel_R_pos[0], wheel_R_pos[2])
            
            # ベース軌跡
            base_pos = positions[0]
            base_trajectory_x.append(base_pos[0])
            base_trajectory_z.append(base_pos[2])
            trajectory_line.set_data(base_trajectory_x, base_trajectory_z)
            
            # 時間表示
            time_text.set_text(f'Time: {time[frame]:.2f}s')
            
            return robot_lines, left_wheel, right_wheel, trajectory_line, time_text
        
        # アニメーション作成
        anim = animation.FuncAnimation(fig, animate, frames=n_frames,
                                     interval=50, blit=True, repeat=True)
        
        # 保存
        filename = 'falling_full_robot_animation.gif'
        print(f"💾 GIF保存中: {filename}")
        anim.save(filename, writer='pillow', fps=20)
        print(f"✅ 保存完了: {filename}")
        
        plt.show()
        return anim

def test_falling_robot():
    """倒れるロボットのテスト"""
    print("="*60)
    print("🤖 フルロボット重力落下テスト")
    print("="*60)
    
    try:
        # シミュレータ初期化
        simulator = FixedFullRobotSimulator()
        
        # 不安定な初期条件（single_legの成功例を参考）
        initial_state = [
            0.0,    # x_base
            0.0,    # y_base  
            0.0,    # yaw
            0.8,    # phi_L_upper (大きく曲げた左上腿)
            -1.2,   # phi_L_lower (大きく曲げた左下腿)
            0.3,    # phi_R_upper (少し曲げた右上腿)
            -0.6    # phi_R_lower (少し曲げた右下腿)
        ]
        
        print(f"🎯 不安定初期条件:")
        print(f"   ベース: x={initial_state[0]}, y={initial_state[1]}, yaw={initial_state[2]}")
        print(f"   左脚: upper={initial_state[3]:.2f}, lower={initial_state[4]:.2f}")
        print(f"   右脚: upper={initial_state[5]:.2f}, lower={initial_state[6]:.2f}")
        
        # シミュレーション実行
        results = simulator.simulate_falling_robot(initial_state, T_sim=3.0, dt=0.02)
        
        # アニメーション作成
        anim = simulator.create_falling_animation(results, "Full Robot Falling with Gravity")
        
        print(f"\n✅ 重力落下テスト完了")
        return simulator, results
        
    except Exception as e:
        print(f"❌ エラー: {e}")
        raise

if __name__ == "__main__":
    test_falling_robot()
#!/usr/bin/env python3
"""
アニメーション描画システム
フルロボットの時系列アニメーション（single_legアプローチの拡張）
"""

import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from basic_loader import FullRobotLoader
from dynamics_simulator_clean import FullRobotDynamicsSimulator

WHEEL_RADIUS = (77.95 / 2) / 1000  # [m]

class FullRobotAnimationRenderer:
    """フルロボットアニメーション描画クラス"""
    
    def __init__(self, simulation_results):
        self.results = simulation_results
        self.time = simulation_results['time']
        self.joint_positions = simulation_results['joint_positions']
        self.n_frames = len(self.time)
        
        print(f"🎬 アニメーション描画初期化")
        print(f"   フレーム数: {self.n_frames}")
        print(f"   時間範囲: {self.time[0]:.2f}s - {self.time[-1]:.2f}s")
        
    def create_robot_animation(self, title="Full Robot Dynamics", save_gif=True):
        """ロボット動力学のリアルタイムアニメーション作成"""
        print(f"🎨 アニメーション作成開始: {title}")
        
        # データ準備
        joint_positions_array = np.array(self.joint_positions)
        
        # フィギュア・軸設定
        fig, ax = plt.subplots(figsize=(14, 10))
        
        # 全フレームを考慮した表示範囲
        all_x = joint_positions_array[:, :, 0].flatten()
        all_z = joint_positions_array[:, :, 2].flatten()
        
        x_min, x_max = np.min(all_x) - 0.2, np.max(all_x) + 0.2
        z_min, z_max = np.min(all_z) - 0.1, np.max(all_z) + 0.1
        
        ax.set_xlim(x_min, x_max)
        ax.set_ylim(z_min, z_max)
        ax.set_xlabel('X Position [m]', fontsize=12)
        ax.set_ylabel('Z Position [m]', fontsize=12)
        ax.set_title(title, fontsize=14)
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        # 地面ライン
        ax.axhline(y=0, color='brown', linewidth=3, alpha=0.8, label='Ground')
        
        # ロボット描画要素
        # ベース
        base_marker, = ax.plot([], [], 'ko', markersize=12, label='Base')
        
        # 左脚
        left_leg_line, = ax.plot([], [], 'b-', linewidth=4, alpha=0.8, label='Left Leg')
        left_joints, = ax.plot([], [], 'bo', markersize=8)
        left_wheel = plt.Circle((0, 0), WHEEL_RADIUS, fill=False, color='blue', linewidth=2)
        ax.add_patch(left_wheel)
        
        # 右脚
        right_leg_line, = ax.plot([], [], 'r-', linewidth=4, alpha=0.8, label='Right Leg')
        right_joints, = ax.plot([], [], 'ro', markersize=8)
        right_wheel = plt.Circle((0, 0), WHEEL_RADIUS, fill=False, color='red', linewidth=2)
        ax.add_patch(right_wheel)
        
        # 軌跡
        base_trajectory_x, base_trajectory_z = [], []
        base_trajectory, = ax.plot([], [], 'k--', alpha=0.6, linewidth=1, label='Base Trajectory')
        
        # 時間・状態表示
        time_text = ax.text(0.02, 0.98, '', transform=ax.transAxes, fontsize=12,
                           verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        state_text = ax.text(0.02, 0.90, '', transform=ax.transAxes, fontsize=10,
                           verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.7))
        
        ax.legend(loc='upper right')
        
        def animate(frame):
            if frame >= self.n_frames:
                frame = self.n_frames - 1
            
            # 現在フレームの関節位置
            positions = joint_positions_array[frame]
            
            # ベース位置（root_joint = index 0）
            base_pos = positions[0]
            base_marker.set_data([base_pos[0]], [base_pos[2]])
            
            # 軌跡更新
            base_trajectory_x.append(base_pos[0])
            base_trajectory_z.append(base_pos[2])
            base_trajectory.set_data(base_trajectory_x, base_trajectory_z)
            
            # 左脚描画 (indices: 0=base, 1=upper_L, 2=lower_L, 3=wheel_L)
            try:
                left_x = [base_pos[0], positions[1][0], positions[2][0], positions[3][0]]
                left_z = [base_pos[2], positions[1][2], positions[2][2], positions[3][2]]
                left_leg_line.set_data(left_x, left_z)
                left_joints.set_data(left_x[1:], left_z[1:])  # 関節点のみ
                left_wheel.center = (positions[3][0], positions[3][2])
            except IndexError:
                pass
            
            # 右脚描画 (indices: 4=upper_R, 5=lower_R, 6=wheel_R)
            try:
                right_x = [base_pos[0], positions[4][0], positions[5][0], positions[6][0]]
                right_z = [base_pos[2], positions[4][2], positions[5][2], positions[6][2]]
                right_leg_line.set_data(right_x, right_z)
                right_joints.set_data(right_x[1:], right_z[1:])  # 関節点のみ
                right_wheel.center = (positions[6][0], positions[6][2])
            except IndexError:
                pass
            
            # 時間表示
            current_time = self.time[frame]
            time_text.set_text(f'Time: {current_time:.2f}s')
            
            # 状態表示
            if hasattr(self, 'results') and 'states' in self.results:
                states = self.results['states']
                if frame < len(states):
                    state = states[frame]
                    state_info = f'Base: ({state[0]:.3f}, {state[1]:.3f}, {state[2]:.3f})\\n'
                    state_info += f'L.Leg: ({state[6]:.2f}, {state[7]:.2f})\\n'
                    state_info += f'R.Leg: ({state[8]:.2f}, {state[9]:.2f})'
                    state_text.set_text(state_info)
            
            return (base_marker, left_leg_line, left_joints, left_wheel, 
                   right_leg_line, right_joints, right_wheel, 
                   base_trajectory, time_text, state_text)
        
        # アニメーション作成
        anim = animation.FuncAnimation(fig, animate, frames=self.n_frames,
                                     interval=50, blit=True, repeat=True)
        
        # GIF保存
        if save_gif:
            filename = f'full_robot_dynamics_animation.gif'
            print(f"💾 GIF保存中: {filename}")
            anim.save(filename, writer='pillow', fps=20)
            print(f"✅ GIF保存完了: {filename}")
        
        plt.show()
        return anim

def test_animation_with_unstable_initial_condition():
    """不安定な初期条件でのアニメーションテスト"""
    print("="*60)
    print("🎬 フルロボット不安定初期条件アニメーションテスト")
    print("="*60)
    
    try:
        # シミュレータ初期化
        simulator = FullRobotDynamicsSimulator()
        
        # 不安定な初期条件設定
        free_state_init, free_velocity_init = simulator.set_initial_condition(
            base_pos=[0.1, 0.0],  # 少し横にずれた位置
            base_orientation=[0.0, 0.2, 0.0],  # ピッチ角傾斜
            left_leg=[0.8, -1.2],   # 大きく曲がった左脚
            right_leg=[0.3, -0.6]   # 通常の右脚
        )
        
        # シミュレーション実行（短時間）
        results = simulator.simulate(free_state_init, free_velocity_init, T_sim=3.0, dt=0.02)
        
        # アニメーション作成
        animator = FullRobotAnimationRenderer(results)
        animator.results = results  # 状態情報をアニメータに渡す
        
        anim = animator.create_robot_animation(
            title="Full Robot - Unstable Initial Condition",
            save_gif=True
        )
        
        # 基本プロット作成
        simulator.create_basic_plots(results)
        
        print(f"\n✅ 不安定初期条件アニメーションテスト完了")
        return animator, results
        
    except Exception as e:
        print(f"❌ エラー: {e}")
        raise

def test_multiple_scenarios():
    """複数シナリオでのアニメーション比較"""
    print("="*60)
    print("🎭 複数シナリオアニメーションテスト")
    print("="*60)
    
    scenarios = [
        {
            'name': 'Small Tilt',
            'base_pos': [0.0, 0.0],
            'base_orientation': [0.0, 0.1, 0.0],
            'left_leg': [0.3, -0.6],
            'right_leg': [0.3, -0.6]
        },
        {
            'name': 'Large Asymmetry',
            'base_pos': [0.0, 0.0],
            'base_orientation': [0.0, 0.0, 0.0],
            'left_leg': [1.0, -1.4],
            'right_leg': [0.2, -0.4]
        },
        {
            'name': 'Forward Lean',
            'base_pos': [0.0, 0.0],
            'base_orientation': [0.0, 0.3, 0.0],
            'left_leg': [0.5, -0.8],
            'right_leg': [0.5, -0.8]
        }
    ]
    
    results_list = []
    
    for i, scenario in enumerate(scenarios):
        print(f"\n📺 シナリオ {i+1}: {scenario['name']}")
        
        simulator = FullRobotDynamicsSimulator()
        
        free_state_init, free_velocity_init = simulator.set_initial_condition(
            base_pos=scenario['base_pos'],
            base_orientation=scenario['base_orientation'],
            left_leg=scenario['left_leg'],
            right_leg=scenario['right_leg']
        )
        
        results = simulator.simulate(free_state_init, free_velocity_init, T_sim=2.5, dt=0.02)
        results_list.append((scenario['name'], results))
        
        # アニメーション作成
        animator = FullRobotAnimationRenderer(results)
        animator.results = results
        
        anim = animator.create_robot_animation(
            title=f"Full Robot - {scenario['name']}",
            save_gif=True
        )
    
    print(f"\n✅ 複数シナリオテスト完了: {len(scenarios)}個のアニメーション生成")
    return results_list

if __name__ == "__main__":
    # 不安定初期条件テスト
    test_animation_with_unstable_initial_condition()
    
    # 複数シナリオテスト
    # test_multiple_scenarios()
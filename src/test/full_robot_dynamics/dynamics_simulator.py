#!/usr/bin/env python3
"""
メイン動力学シミュレータ
フルロボットの拘束付き動力学シミュレーション（single_legアプローチの拡張）
"""

import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from basic_loader import FullRobotLoader
from constraint_solver import BipdealConstraintSolver

class FullRobotDynamicsSimulator:
    """フルロボット動力学シミュレータ"""
    
    def __init__(self):
        # 基本コンポーネント初期化
        self.loader = FullRobotLoader()
        self.model, self.data = self.loader.load_model()
        self.loader.compute_ground_offset()
        
        # 拘束ソルバー
        self.solver = BipdealConstraintSolver(self.model, self.data)
        
        # シミュレーション状態
        self.time_history = []
        self.state_history = []
        self.full_config_history = []
        self.joint_positions_history = []
        
        print(f"🚀 フルロボット動力学シミュレータ初期化完了")
        print(f"   独立変数数: {len(self.solver.free_indices)}")
    
    def set_initial_condition(self, base_pos=[0.0, 0.0], base_orientation=[0.0, 0.0, 0.0], 
                             left_leg=[0.3, -0.6], right_leg=[0.3, -0.6]):
        """初期条件の設定"""
        
        # 独立変数（10要素）
        # [x, y, z, roll, pitch, yaw, upper_L, lower_L, upper_R, lower_R]
        free_state = np.array([
            base_pos[0], base_pos[1], self.loader.ground_offset,  # ベース位置
            base_orientation[0], base_orientation[1], base_orientation[2],  # ベース姿勢
            left_leg[0], left_leg[1],   # 左脚関節角度
            right_leg[0], right_leg[1]  # 右脚関節角度
        ])
        
        free_velocity = np.zeros(10)  # 初期速度はゼロ
        
        print(f"🎯 初期条件設定:")
        print(f"   ベース位置: ({base_pos[0]:.3f}, {base_pos[1]:.3f}, {self.loader.ground_offset:.3f})")
        print(f"   ベース姿勢: ({base_orientation[0]:.3f}, {base_orientation[1]:.3f}, {base_orientation[2]:.3f})")
        print(f"   左脚角度: ({left_leg[0]:.3f}, {left_leg[1]:.3f})")
        print(f"   右脚角度: ({right_leg[0]:.3f}, {right_leg[1]:.3f})")
        
        return free_state, free_velocity
    
    def simulate(self, free_state_init, free_velocity_init, T_sim=3.0, dt=0.02):
        """メイン動力学シミュレーション"""
        print(f"\n🎬 動力学シミュレーション開始")
        print(f"   シミュレーション時間: {T_sim}s")
        print(f"   時間刻み: {dt}s")
        
        # 初期化
        free_state = free_state_init.copy()
        free_velocity = free_velocity_init.copy()
        
        t_array = np.arange(0, T_sim, dt)
        N = len(t_array)
        
        # 履歴配列初期化
        self.time_history = []
        self.state_history = []
        self.full_config_history = []
        self.joint_positions_history = []
        
        print(f"   総ステップ数: {N}")
        
        # シミュレーションループ
        for i, t in enumerate(t_array):
            # 現在状態を記録
            self.time_history.append(t)
            self.state_history.append(np.concatenate([free_state, free_velocity]))
            
            # 完全構成を計算
            q_full = self.solver.solve_constraint_configuration(free_state)
            self.full_config_history.append(q_full.copy())
            
            # 関節位置を記録
            positions, _ = self.loader.get_joint_positions(q_full)
            self.joint_positions_history.append(positions)
            
            # 進行状況表示
            if i % 25 == 0:  # 0.5秒ごと
                base_height = q_full[2]
                print(f"   t={t:.2f}s: ベース高度={base_height:.4f}m, 左脚=({free_state[6]:.3f},{free_state[7]:.3f}), 右脚=({free_state[8]:.3f},{free_state[9]:.3f})")
            
            # 動力学計算
            try:
                M_red, g_red, C_red, q_dynamics, dq_dynamics = self.solver.compute_reduced_dynamics(
                    free_state, free_velocity)
                
                # 運動方程式: M * ddq = tau - g - C
                tau = np.zeros(10)  # 無制御
                
                # 加速度計算
                dd_free_state = np.linalg.solve(M_red, tau - g_red - C_red)
                
            except Exception as e:
                print(f"   ⚠️  動力学計算エラー at t={t:.3f}: {e}")
                dd_free_state = np.zeros(10)
            
            # オイラー積分
            free_velocity += dd_free_state * dt
            free_state += free_velocity * dt
            
            # 発散チェック
            if not (np.all(np.isfinite(free_state)) and np.all(np.isfinite(free_velocity))):
                print(f"   ❌ 発散検出 at t={t:.3f}")
                break
        
        # 結果サマリー
        final_state = self.state_history[-1][:10]  # 位置のみ
        final_base_height = self.full_config_history[-1][2]
        
        print(f"\n✅ シミュレーション完了")
        print(f"   実行ステップ数: {len(self.time_history)}")
        print(f"   最終ベース位置: ({final_state[0]:.4f}, {final_state[1]:.4f}, {final_base_height:.4f})")
        print(f"   最終左脚角度: ({final_state[6]:.3f}, {final_state[7]:.3f})")
        print(f"   最終右脚角度: ({final_state[8]:.3f}, {final_state[9]:.3f})")
        
        return self.get_simulation_results()\n    \n    def get_simulation_results(self):\n        \"\"\"シミュレーション結果の取得\"\"\"\n        return {\n            'time': np.array(self.time_history),\n            'states': np.array(self.state_history),\n            'full_configs': np.array(self.full_config_history),\n            'joint_positions': self.joint_positions_history\n        }\n    \n    def create_basic_plots(self, results=None):\n        \"\"\"基本的なプロット作成\"\"\"\n        if results is None:\n            results = self.get_simulation_results()\n        \n        time = results['time']\n        states = results['states']\n        \n        # 4つのサブプロット\n        fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))\n        \n        # ベース位置\n        ax1.plot(time, states[:, 0], 'r-', label='X', linewidth=2)\n        ax1.plot(time, states[:, 1], 'g-', label='Y', linewidth=2)\n        ax1.plot(time, states[:, 2], 'b-', label='Z', linewidth=2)\n        ax1.set_xlabel('Time [s]')\n        ax1.set_ylabel('Base Position [m]')\n        ax1.set_title('Base Position')\n        ax1.legend()\n        ax1.grid(True)\n        \n        # ベース姿勢\n        ax2.plot(time, states[:, 3] * 180/np.pi, 'r-', label='Roll', linewidth=2)\n        ax2.plot(time, states[:, 4] * 180/np.pi, 'g-', label='Pitch', linewidth=2)\n        ax2.plot(time, states[:, 5] * 180/np.pi, 'b-', label='Yaw', linewidth=2)\n        ax2.set_xlabel('Time [s]')\n        ax2.set_ylabel('Base Orientation [deg]')\n        ax2.set_title('Base Orientation')\n        ax2.legend()\n        ax2.grid(True)\n        \n        # 左脚関節角度\n        ax3.plot(time, states[:, 6] * 180/np.pi, 'r-', label='Upper', linewidth=2)\n        ax3.plot(time, states[:, 7] * 180/np.pi, 'b-', label='Lower', linewidth=2)\n        ax3.set_xlabel('Time [s]')\n        ax3.set_ylabel('Left Leg Angles [deg]')\n        ax3.set_title('Left Leg Joint Angles')\n        ax3.legend()\n        ax3.grid(True)\n        \n        # 右脚関節角度\n        ax4.plot(time, states[:, 8] * 180/np.pi, 'r-', label='Upper', linewidth=2)\n        ax4.plot(time, states[:, 9] * 180/np.pi, 'b-', label='Lower', linewidth=2)\n        ax4.set_xlabel('Time [s]')\n        ax4.set_ylabel('Right Leg Angles [deg]')\n        ax4.set_title('Right Leg Joint Angles')\n        ax4.legend()\n        ax4.grid(True)\n        \n        plt.tight_layout()\n        plt.savefig('full_robot_simulation_results.png', dpi=150, bbox_inches='tight')\n        plt.show()\n        \n        print(f\"💾 基本プロット保存: full_robot_simulation_results.png\")\n\ndef test_dynamics_simulation():\n    \"\"\"動力学シミュレーションのテスト\"\"\"\n    print(\"=\"*60)\n    print(\"🚀 フルロボット動力学シミュレーションテスト\")\n    print(\"=\"*60)\n    \n    try:\n        # シミュレータ初期化\n        simulator = FullRobotDynamicsSimulator()\n        \n        # 初期条件設定（小さな傾斜）\n        free_state_init, free_velocity_init = simulator.set_initial_condition(\n            base_pos=[0.0, 0.0],\n            base_orientation=[0.0, 0.0, 0.0],\n            left_leg=[0.3, -0.6],\n            right_leg=[0.3, -0.6]\n        )\n        \n        # シミュレーション実行\n        results = simulator.simulate(free_state_init, free_velocity_init, T_sim=2.0, dt=0.02)\n        \n        # 基本プロット\n        simulator.create_basic_plots(results)\n        \n        print(f\"\\n✅ 動力学シミュレーションテスト完了\")\n        return simulator, results\n        \n    except Exception as e:\n        print(f\"❌ エラー: {e}\")\n        raise\n\nif __name__ == \"__main__\":\n    test_dynamics_simulation()
#!/usr/bin/env python3
"""
左右非対称の順運動学テスト - ベースRoll角度の確認
"""

import numpy as np
import matplotlib.pyplot as plt
from fixed_robot_3d import FixedRobot3D
import pinocchio as pin

def test_asymmetric_kinematics():
    """Test kinematics with different left/right leg angles"""
    print("=== Asymmetric Kinematics Test ===")
    
    robot = FixedRobot3D()
    
    # テストケース：左右で異なる膝角度
    test_cases = [
        (-0.3, -0.3),  # 対称
        (-0.3, -0.1),  # 右が高い
        (-0.1, -0.3),  # 左が高い
        (-0.5, 0.0),   # 極端な差
        (0.0, -0.5),   # 逆の極端な差
        (-0.2, -0.4),  # 中程度の差
    ]
    
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    axes = axes.flatten()
    
    for i, (phi_L_lower, phi_R_lower) in enumerate(test_cases):
        ax = axes[i]
        
        # 従属関節拘束
        phi_L_upper = -2 * phi_L_lower
        phi_R_upper = -2 * phi_R_lower
        
        print(f"\nCase {i+1}:")
        print(f"  Left: knee={phi_L_lower:.2f}, hip={phi_L_upper:.2f}")
        print(f"  Right: knee={phi_R_lower:.2f}, hip={phi_R_upper:.2f}")
        
        # 構成計算（6自由度版）
        q = robot.compute_constrained_configuration(0.0, 0.0, phi_L_lower, phi_R_lower, 0.0, 0.0)
        
        # ベースの姿勢を取得
        base_roll = q[3]
        base_z = q[2]
        print(f"  Base: Z={base_z:.3f}, Roll={base_roll:.3f} rad ({np.degrees(base_roll):.1f} deg)")
        
        # 順運動学
        pin.forwardKinematics(robot.model, robot.data, q)
        
        # 各関節位置を取得
        positions = {}
        for j in range(robot.model.njoints):
            name = robot.model.names[j]
            pos = robot.data.oMi[j].translation
            positions[name] = pos
        
        # 正面図プロット（Y-Z平面）
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        
        # 地面
        ax.axhline(y=0, color='brown', linewidth=2)
        
        # ベース
        base_pos = positions.get('root_joint', positions.get('base'))
        if base_pos is not None:
            ax.plot(base_pos[1], base_pos[2], 'ko', markersize=12, label='Base')
            # ベースの傾きを表示
            base_width = 0.2
            left_y = base_pos[1] - base_width/2 * np.cos(base_roll)
            right_y = base_pos[1] + base_width/2 * np.cos(base_roll)
            left_z = base_pos[2] - base_width/2 * np.sin(base_roll)
            right_z = base_pos[2] + base_width/2 * np.sin(base_roll)
            ax.plot([left_y, right_y], [left_z, right_z], 'k-', linewidth=3)
        
        # 左脚
        left_positions = []
        for name in ['root_joint', 'upper_link_L_joint', 'lower_link_L_joint', 'wheel_L_joint']:
            if name in positions:
                left_positions.append(positions[name])
        
        if len(left_positions) > 1:
            y_coords = [p[1] for p in left_positions]
            z_coords = [p[2] for p in left_positions]
            ax.plot(y_coords, z_coords, 'b-o', linewidth=2, markersize=6, label='Left')
        
        # 右脚
        right_positions = []
        for name in ['root_joint', 'upper_link_R_joint', 'lower_link_R_joint', 'wheel_R_joint']:
            if name in positions:
                right_positions.append(positions[name])
        
        if len(right_positions) > 1:
            y_coords = [p[1] for p in right_positions]
            z_coords = [p[2] for p in right_positions]
            ax.plot(y_coords, z_coords, 'r-o', linewidth=2, markersize=6, label='Right')
        
        # ホイール接地点
        for name, pos in positions.items():
            if 'wheel' in name.lower():
                contact_z = pos[2] - 0.075  # WHEEL_RADIUS
                ax.plot(pos[1], contact_z, 'gx', markersize=10)
                print(f"  {name} contact Z: {contact_z:.3f}")
        
        ax.set_xlabel('Y [m]')
        ax.set_ylabel('Z [m]')
        ax.set_title(f'L:({phi_L_lower:.1f},{phi_L_upper:.1f}) R:({phi_R_lower:.1f},{phi_R_upper:.1f})\nRoll={np.degrees(base_roll):.1f}°')
        ax.set_xlim([-0.4, 0.4])
        ax.set_ylim([-0.1, 0.5])
        ax.legend()
    
    plt.tight_layout()
    plt.savefig('asymmetric_kinematics_test.png', dpi=150)
    print("\nSaved: asymmetric_kinematics_test.png")

if __name__ == "__main__":
    test_asymmetric_kinematics()
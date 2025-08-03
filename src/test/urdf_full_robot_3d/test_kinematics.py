#!/usr/bin/env python3
"""
順運動学テスト - 従属関節拘束の検証
"""

import numpy as np
import matplotlib.pyplot as plt
from fixed_robot_3d import FixedRobot3D
import pinocchio as pin

def test_kinematics():
    """Test forward kinematics with dependent joint constraints"""
    print("=== Forward Kinematics Test ===")
    
    robot = FixedRobot3D()
    
    # テストケース：異なる膝関節角度
    test_angles = [-0.5, -0.3, -0.1, 0.0, 0.1, 0.3, 0.5]
    
    fig, axes = plt.subplots(2, 4, figsize=(16, 8))
    axes = axes.flatten()
    
    for i, phi_lower in enumerate(test_angles):
        ax = axes[i]
        
        # 従属関節拘束
        phi_upper = -2 * phi_lower
        
        print(f"\nCase {i+1}:")
        print(f"  Knee angle (lower): {phi_lower:.2f} rad")
        print(f"  Hip angle (upper): {phi_upper:.2f} rad (= -2 * {phi_lower:.2f})")
        
        # 構成計算（左右同じ角度、6自由度版）
        q = robot.compute_constrained_configuration(0.0, 0.0, phi_lower, phi_lower, 0.0, 0.0)
        
        # 順運動学
        pin.forwardKinematics(robot.model, robot.data, q)
        
        # 各関節位置を取得
        positions = {}
        for j in range(robot.model.njoints):
            name = robot.model.names[j]
            pos = robot.data.oMi[j].translation
            positions[name] = pos
            if 'wheel' in name.lower() or 'link' in name.lower() or 'base' in name.lower():
                print(f"  {name}: [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}]")
        
        # 側面図プロット
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        
        # 地面
        ax.axhline(y=0, color='brown', linewidth=2)
        
        
        # ベース描画
        base_names = ['root_joint', 'base']
        base_pos = None
        for name in base_names:
            if name in positions:
                base_pos = positions[name]
                break
        
        if base_pos is not None:
            ax.plot(base_pos[0], base_pos[2], 'ko', markersize=10, label='Base')
        
        # 左脚の描画
        left_upper = positions.get('upper_link_L_joint', positions.get('upper_link_L'))
        left_lower = positions.get('lower_link_L_joint', positions.get('lower_link_L'))
        left_wheel = positions.get('wheel_L_joint', positions.get('wheel_L'))
        
        if base_pos is not None and left_upper is not None:
            ax.plot([base_pos[0], left_upper[0]], [base_pos[2], left_upper[2]], 'b-', linewidth=3)
            ax.plot(left_upper[0], left_upper[2], 'bo', markersize=8)
        
        if left_upper is not None and left_lower is not None:
            ax.plot([left_upper[0], left_lower[0]], [left_upper[2], left_lower[2]], 'b-', linewidth=3)
            ax.plot(left_lower[0], left_lower[2], 'bo', markersize=8)
            
        if left_lower is not None and left_wheel is not None:
            ax.plot([left_lower[0], left_wheel[0]], [left_lower[2], left_wheel[2]], 'b-', linewidth=3)
            ax.plot(left_wheel[0], left_wheel[2], 'bo', markersize=8)
        
        # 右脚の描画
        right_upper = positions.get('upper_link_R_joint', positions.get('upper_link_R'))
        right_lower = positions.get('lower_link_R_joint', positions.get('lower_link_R'))
        right_wheel = positions.get('wheel_R_joint', positions.get('wheel_R'))
        
        if base_pos is not None and right_upper is not None:
            ax.plot([base_pos[0], right_upper[0]], [base_pos[2], right_upper[2]], 'r-', linewidth=3)
            ax.plot(right_upper[0], right_upper[2], 'ro', markersize=8)
        
        if right_upper is not None and right_lower is not None:
            ax.plot([right_upper[0], right_lower[0]], [right_upper[2], right_lower[2]], 'r-', linewidth=3)
            ax.plot(right_lower[0], right_lower[2], 'ro', markersize=8)
            
        if right_lower is not None and right_wheel is not None:
            ax.plot([right_lower[0], right_wheel[0]], [right_lower[2], right_wheel[2]], 'r-', linewidth=3)
            ax.plot(right_wheel[0], right_wheel[2], 'ro', markersize=8)
        
        # ホイール接地点確認
        for name, pos in positions.items():
            if 'wheel' in name.lower():
                contact_z = pos[2] - 0.075  # WHEEL_RADIUS
                ax.plot(pos[0], contact_z, 'rx', markersize=10)
                print(f"  {name} contact Z: {contact_z:.3f}")
        
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Z [m]')
        ax.set_title(f'phi_lower={phi_lower:.1f}, phi_upper={phi_upper:.1f}')
        ax.set_xlim([-0.5, 0.5])
        ax.set_ylim([-0.1, 0.5])
    
    # 最後のサブプロットに凡例
    axes[-1].axis('off')
    axes[-1].text(0.1, 0.5, 'Dependent joint constraint:\nphi_upper = -2 x phi_lower\n\nRed x: Wheel contact points\nBlack o: Base\nBlue o: Left leg joints\nRed o: Right leg joints', 
                  fontsize=12, transform=axes[-1].transAxes)
    
    plt.tight_layout()
    plt.savefig('kinematics_test.png', dpi=150)
    print("\nSaved: kinematics_test.png")
    # plt.show()

if __name__ == "__main__":
    test_kinematics()
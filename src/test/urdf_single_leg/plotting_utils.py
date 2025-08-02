#!/usr/bin/env python3
"""
プロッティングユーティリティ
シミュレーション結果の静止画グラフ表示機能
"""

import numpy as np
import matplotlib.pyplot as plt

def plot_noslip_results(t_array, state_history, base_positions, theta_wheel_history):
    """ノンスリップシミュレーション結果の3つのグラフを表示"""
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 5))
    
    # 関節角度
    ax1.plot(t_array, state_history[:, 1] * 180/np.pi, 'r-', label='φ1')
    ax1.plot(t_array, state_history[:, 2] * 180/np.pi, 'b-', label='φ2')
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Angle [deg]')
    ax1.set_title('Joint Angles (No-Slip)')
    ax1.legend()
    ax1.grid(True)
    
    # ベースX位置
    ax2.plot(t_array, base_positions, 'm-', linewidth=2)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Base X Position [m]')
    ax2.set_title('Base Horizontal Motion (No-Slip)')
    ax2.grid(True)
    
    # ホイール回転角度
    ax3.plot(t_array, theta_wheel_history * 180/np.pi, 'g-', linewidth=2)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Wheel Rotation [deg]')
    ax3.set_title('Wheel Rotation (No-Slip)')
    ax3.grid(True)
    
    plt.tight_layout()
    plt.savefig('2dof_noslip_result.png', dpi=150, bbox_inches='tight')
    plt.show()

def plot_simple_results(t_array, state_history, base_heights, x_positions):
    """シンプル拘束シミュレーション結果の3つのグラフを表示"""
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 5))
    
    # 関節角度
    ax1.plot(t_array, state_history[:, 1] * 180/np.pi, 'r-', label='φ1')
    ax1.plot(t_array, state_history[:, 2] * 180/np.pi, 'b-', label='φ2')
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Angle [deg]')
    ax1.set_title('Joint Angles')
    ax1.legend()
    ax1.grid(True)
    
    # ベース高度
    ax2.plot(t_array, base_heights, 'g-', linewidth=2)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Base Height [m]')
    ax2.set_title('Base Height (Should Fall!)')
    ax2.grid(True)
    
    # ベースX位置
    ax3.plot(t_array, x_positions, 'm-', linewidth=2)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Base X Position [m]')
    ax3.set_title('Base Horizontal Motion')
    ax3.grid(True)
    
    plt.tight_layout()
    plt.savefig('3dof_constrained_result.png', dpi=150, bbox_inches='tight')
    plt.show()
"""
2D可視化システム
URDFロボットモデルの関節位置を2D平面で可視化
"""

import matplotlib.pyplot as plt
import numpy as np
from load_urdf_model import load_mimic_robot_model, get_joint_positions_2d, create_configuration_vector

def define_robot_kinematic_chain():
    """
    ロボットの運動学チェーン構造を定義
    
    Returns:
        kinematic_chains: 各脚の関節接続情報
    """
    # 右脚チェーン: base -> upper -> lower -> wheel
    right_leg_chain = [
        'base_link',
        'upper_link_R', 
        'lower_link_R',
        'wheel_R'
    ]
    
    # 左脚チェーン: base -> upper -> lower -> wheel  
    left_leg_chain = [
        'base_link',
        'upper_link_L',
        'lower_link_L', 
        'wheel_L'
    ]
    
    kinematic_chains = {
        'right_leg': right_leg_chain,
        'left_leg': left_leg_chain
    }
    
    return kinematic_chains

def plot_robot_2d(joint_positions, title="URDF Robot 2D", show_both_legs=True, ax=None):
    """
    ロボットの2D可視化
    
    Args:
        joint_positions: 各関節の2D位置 {joint_name: [x, z]}
        title: プロットタイトル
        show_both_legs: 両脚表示するか
        ax: matplotlib軸（Noneの場合は新規作成）
    
    Returns:
        fig, ax: matplotlib オブジェクト
    """
    if ax is None:
        fig, ax = plt.subplots(1, 1, figsize=(10, 8))
    else:
        fig = ax.get_figure()
    
    # 運動学チェーン取得
    chains = define_robot_kinematic_chain()
    
    # 表示する脚を選択
    if show_both_legs:
        active_chains = ['right_leg', 'left_leg']
        chain_colors = {'right_leg': 'blue', 'left_leg': 'red'}
        chain_labels = {'right_leg': 'Right Leg', 'left_leg': 'Left Leg'}
    else:
        active_chains = ['right_leg']  # 右脚のみ表示
        chain_colors = {'right_leg': 'blue'}
        chain_labels = {'right_leg': 'Robot Links'}
    
    # 各脚のチェーンを描画
    for chain_name in active_chains:
        chain = chains[chain_name]
        color = chain_colors[chain_name]
        label = chain_labels[chain_name]
        
        # チェーン内の関節を線で接続
        x_coords = []
        z_coords = []
        
        for joint_name in chain:
            if joint_name in joint_positions:
                pos = joint_positions[joint_name]
                x_coords.append(pos[0])
                z_coords.append(pos[1])
        
        # リンク線を描画
        if len(x_coords) >= 2:
            ax.plot(x_coords, z_coords, '-', color=color, linewidth=3, 
                   label=label, alpha=0.8)
    
    # 関節位置をマーカーで表示
    for joint_name, pos in joint_positions.items():
        x, z = pos
        
        # 関節タイプによってマーカーを変更
        if 'base' in joint_name:
            marker = 's'  # 四角形
            size = 100
            color = 'black'
            label = 'Base'
        elif 'wheel' in joint_name:
            marker = 'o'  # 円形
            size = 80
            color = 'orange'
            label = 'Wheel'
        else:
            marker = '^'  # 三角形
            size = 60
            color = 'green'
            label = 'Joint'
        
        ax.scatter(x, z, marker=marker, s=size, c=color, 
                  edgecolors='black', linewidth=1, alpha=0.9)
        
        # 関節名をテキストで表示
        ax.annotate(joint_name.replace('_', '\\n'), (x, z), 
                   xytext=(5, 5), textcoords='offset points',
                   fontsize=8, ha='left')
    
    # 地面を表示（z=0）
    x_range = ax.get_xlim()
    ax.axhline(y=0, color='brown', linewidth=3, alpha=0.6, label='Ground')
    
    # 軸設定
    ax.set_xlabel('X [m]', fontsize=12)
    ax.set_ylabel('Z [m]', fontsize=12)
    ax.set_title(title, fontsize=14, fontweight='bold')
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')
    ax.legend()
    
    return fig, ax

def compare_configurations(joint_pos_list, config_names, title="Robot Configuration Comparison"):
    """
    複数設定での比較表示
    
    Args:
        joint_pos_list: 各設定での関節位置リスト
        config_names: 設定名リスト
        title: 比較プロットのタイトル
    """
    n_configs = len(joint_pos_list)
    fig, axes = plt.subplots(1, n_configs, figsize=(6*n_configs, 8))
    
    if n_configs == 1:
        axes = [axes]
    
    for i, (joint_positions, config_name) in enumerate(zip(joint_pos_list, config_names)):
        plot_robot_2d(joint_positions, 
                     title=f"{config_name}", 
                     show_both_legs=False,  # 右脚のみ表示で見やすく
                     ax=axes[i])
    
    plt.suptitle(title, fontsize=16, fontweight='bold')
    plt.tight_layout()
    plt.show()
    return fig

def analyze_joint_movements(joint_pos_initial, joint_pos_final):
    """
    関節移動量の分析
    
    Args:
        joint_pos_initial: 初期関節位置
        joint_pos_final: 最終関節位置
    
    Returns:
        movement_analysis: 移動量分析結果
    """
    print("\n=== Joint Movement Analysis ===")
    
    movement_analysis = {}
    
    for joint_name in joint_pos_initial:
        if joint_name in joint_pos_final:
            pos_initial = np.array(joint_pos_initial[joint_name])
            pos_final = np.array(joint_pos_final[joint_name])
            
            displacement = pos_final - pos_initial
            distance = np.linalg.norm(displacement)
            
            movement_analysis[joint_name] = {
                'initial_pos': pos_initial,
                'final_pos': pos_final,
                'displacement': displacement,
                'distance': distance
            }
            
            print(f"{joint_name}:")
            print(f"  Initial: [{pos_initial[0]:.4f}, {pos_initial[1]:.4f}]")
            print(f"  Final:   [{pos_final[0]:.4f}, {pos_final[1]:.4f}]")
            print(f"  Move:    [{displacement[0]:.4f}, {displacement[1]:.4f}] (dist: {distance:.4f})")
    
    return movement_analysis

def demo_2d_visualization():
    """
    2D可視化のデモンストレーション
    """
    print("=== 2D Visualization Demo ===")
    
    try:
        # モデル読み込み
        model, data, joint_names, link_names = load_mimic_robot_model()
        
        # 初期設定
        q_initial = create_configuration_vector(model)
        joint_pos_initial, _ = get_joint_positions_2d(model, data, q_initial)
        
        # 指定設定
        joint_angles = {
            'upper_link_R_joint': np.radians(52),   
            'lower_link_R_joint': np.radians(-104), 
            'wheel_R_joint': 0.0,                   
            'upper_link_L_joint': np.radians(52),   
            'lower_link_L_joint': np.radians(-104),
            'wheel_L_joint': 0.0
        }
        
        q_specified = create_configuration_vector(
            model, 
            base_pos=[0, 0, 0],
            joint_angles=joint_angles
        )
        joint_pos_specified, _ = get_joint_positions_2d(model, data, q_specified)
        
        # 移動量分析
        movement_analysis = analyze_joint_movements(joint_pos_initial, joint_pos_specified)
        
        # 比較表示
        compare_configurations(
            [joint_pos_initial, joint_pos_specified],
            ["Initial Configuration (0°)", "Specified Configuration (52°, -104°)"],
            "URDF Robot 2D: Configuration Comparison"
        )
        
        # 個別表示（両脚込み）
        fig, ax = plt.subplots(1, 1, figsize=(12, 10))
        plot_robot_2d(joint_pos_specified, 
                     title="URDF Robot - Both Legs (hip=52°, knee=-104°)",
                     show_both_legs=True, ax=ax)
        plt.show()
        
        return True
        
    except Exception as e:
        print(f"Error in 2D visualization demo: {e}")
        return False

if __name__ == "__main__":
    # 仮想環境確認
    try:
        import pinocchio as pin
        print("Pinocchio library loaded successfully")
    except ImportError:
        print("❌ Pinocchio library not found")
        print("Please activate virtual environment: . bin/activate")
        exit(1)
    
    # デモ実行
    success = demo_2d_visualization()
    if success:
        print("\n✅ 2D visualization demo completed successfully!")
    else:
        print("❌ 2D visualization demo failed")
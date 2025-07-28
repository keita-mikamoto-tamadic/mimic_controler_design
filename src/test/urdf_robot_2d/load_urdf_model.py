"""
URDF読み込み・モデル構築機能
URDFファイルからPinocchioモデルを構築し、順運動学計算を行う
"""

import pinocchio as pin
import numpy as np
import os

def load_mimic_robot_model(urdf_path=None, single_leg=False):
    """
    URDF から mimic ロボットモデルを構築
    
    Args:
        urdf_path: URDFファイルのパス（Noneの場合はデフォルトパス使用）
        single_leg: 単脚版URDFを使用するか
    
    Returns:
        model: Pinocchio モデル
        data: Pinocchio データ  
        joint_names: 関節名リスト
        link_names: リンク名リスト
    """
    if urdf_path is None:
        # デフォルトパス設定
        current_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.join(current_dir, "..", "..", "..")
        if single_leg:
            urdf_path = os.path.join(project_root, "urdf", "mimic_v1_single_leg.urdf")
        else:
            urdf_path = os.path.join(project_root, "urdf", "mimic_v1.urdf")
    
    print(f"Loading URDF from: {urdf_path}")
    
    # URDFファイル存在確認
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF file not found: {urdf_path}")
    
    # フローティングベースでモデル構築
    model = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
    data = model.createData()
    
    # 関節名とリンク名を取得
    joint_names = [model.names[i] for i in range(model.njoints)]
    link_names = [model.names[i] for i in range(model.njoints) if i > 0]  # universe を除外
    
    print(f"Model loaded successfully:")
    print(f"  Total DOF: {model.nq}")
    print(f"  Joints: {len(joint_names)}")
    print(f"  Links: {len(link_names)}")
    
    return model, data, joint_names, link_names

def print_model_info(model, joint_names, link_names):
    """
    モデル情報を詳細表示
    """
    print("\n=== Model Information ===")
    print(f"Model name: {model.name}")
    print(f"Configuration dimension (nq): {model.nq}")
    print(f"Velocity dimension (nv): {model.nv}")
    print(f"Number of joints: {model.njoints}")
    
    print("\nJoint Names:")
    for i, name in enumerate(joint_names):
        if i > 0:  # universe は除外
            joint_id = model.getJointId(name)
            print(f"  {i}: {name} (ID: {joint_id})")
    
    print("\nLink Names:")
    for i, name in enumerate(link_names):
        print(f"  {i+1}: {name}")

def create_configuration_vector(model, base_pos=None, base_quat=None, joint_angles=None):
    """
    設定ベクトルを構築（連続関節のクォータニオン表現に対応）
    
    Args:
        model: Pinocchio モデル
        base_pos: ベース位置 [x, y, z] (デフォルト: [0, 0, 0])
        base_quat: ベース姿勢クォータニオン [x, y, z, w] (デフォルト: [0, 0, 0, 1])
        joint_angles: 関節角度辞書 {joint_name: angle_rad}
    
    Returns:
        q: 設定ベクトル
    """
    # デフォルト値設定
    if base_pos is None:
        base_pos = [0.0, 0.0, 0.0]
    if base_quat is None:
        base_quat = [0.0, 0.0, 0.0, 1.0]  # 無回転クォータニオン
    if joint_angles is None:
        joint_angles = {}
    
    # 設定ベクトル初期化（中立設定から開始）
    q = pin.neutral(model)
    
    # フローティングベース設定 (最初の7要素)
    q[0:3] = base_pos      # 位置 [x, y, z]
    q[3:7] = base_quat     # 姿勢クォータニオン [x, y, z, w]
    
    # 関節角度設定（連続関節のクォータニオン表現に対応）
    for joint_name, angle in joint_angles.items():
        try:
            joint_id = model.getJointId(joint_name)
            joint = model.joints[joint_id]
            joint_idx = joint.idx_q
            nq = joint.nq
            
            if nq == 1:
                # 単一角度の場合
                q[joint_idx] = angle
                print(f"Set {joint_name}: {np.degrees(angle):.1f}° at index {joint_idx}")
            elif nq == 2:
                # 連続関節（クォータニオン表現）の場合
                # [cos(θ/2), sin(θ/2)] の形式で設定
                half_angle = angle / 2.0
                q[joint_idx] = np.cos(half_angle)      # cos成分
                q[joint_idx + 1] = np.sin(half_angle)  # sin成分
                print(f"Set {joint_name}: {np.degrees(angle):.1f}° at indices [{joint_idx}:{joint_idx+2}] as quaternion")
            else:
                print(f"Warning: Unsupported joint dimension nq={nq} for {joint_name}")
                
        except Exception as e:
            print(f"Warning: Could not set joint {joint_name}: {e}")
    
    return q

def get_joint_positions_2d(model, data, q_config):
    """
    指定された設定での全関節位置を計算（2D射影）
    
    Args:
        model: Pinocchio モデル
        data: Pinocchio データ
        q_config: 完全な関節設定
    
    Returns:
        joint_positions: 各関節の2D位置 {joint_name: [x, z]}
        link_info: 各リンクの詳細情報
    """
    # 順運動学計算
    pin.forwardKinematics(model, data, q_config)
    
    joint_positions = {}
    link_info = {}
    
    print("\n=== Joint Positions (2D Projection) ===")
    
    for i in range(1, model.njoints):  # universe(0) を除外
        joint_name = model.names[i]
        
        # 関節位置取得
        joint_placement = data.oMi[i]
        pos_3d = joint_placement.translation
        
        # 2D射影 (Y軸無視: X-Z平面)
        pos_2d = [pos_3d[0], pos_3d[2]]
        joint_positions[joint_name] = pos_2d
        
        # リンク情報保存
        link_info[joint_name] = {
            'position_3d': pos_3d.copy(),
            'position_2d': pos_2d,
            'rotation': joint_placement.rotation.copy()
        }
        
        print(f"  {joint_name}: [{pos_2d[0]:.4f}, {pos_2d[1]:.4f}] (3D: [{pos_3d[0]:.4f}, {pos_3d[1]:.4f}, {pos_3d[2]:.4f}])")
    
    return joint_positions, link_info

def demo_urdf_loading():
    """
    URDF読み込みのデモンストレーション
    """
    print("=== URDF Loading Demo ===")
    
    try:
        # モデル読み込み
        model, data, joint_names, link_names = load_mimic_robot_model()
        
        # モデル情報表示
        print_model_info(model, joint_names, link_names)
        
        # 初期設定での順運動学
        print("\n--- Initial Configuration (All joints at 0°) ---")
        q_initial = pin.neutral(model)
        joint_pos_initial, link_info_initial = get_joint_positions_2d(model, data, q_initial)
        
        # 指定角度での設定
        print("\n--- Specified Configuration (hip=52°, knee=-104°) ---")
        joint_angles = {
            'upper_link_R_joint': np.radians(52),   # 52° → rad
            'lower_link_R_joint': np.radians(-104), # -104° → rad
            'wheel_R_joint': 0.0,                   # 0°
        }
        
        # 両脚版の場合は左脚も設定
        if 'upper_link_L_joint' in [model.names[i] for i in range(model.njoints)]:
            joint_angles.update({
                'upper_link_L_joint': np.radians(52),   # 左脚も同様
                'lower_link_L_joint': np.radians(-104),
                'wheel_L_joint': 0.0
            })
        
        q_specified = create_configuration_vector(
            model, 
            base_pos=[0, 0, 0],
            joint_angles=joint_angles
        )
        
        joint_pos_specified, link_info_specified = get_joint_positions_2d(model, data, q_specified)
        
        return model, data, joint_pos_initial, joint_pos_specified, q_specified
        
    except Exception as e:
        print(f"Error in URDF loading demo: {e}")
        return None

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
    result = demo_urdf_loading()
    if result:
        print("\n✅ URDF loading demo completed successfully!")
    else:
        print("❌ URDF loading demo failed")
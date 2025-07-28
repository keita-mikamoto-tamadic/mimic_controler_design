"""
URDF順運動学テスト実行
指定された関節角度での順運動学計算と2D可視化のメインテストプログラム
"""

import numpy as np
import matplotlib.pyplot as plt
from load_urdf_model import load_mimic_robot_model, create_configuration_vector, get_joint_positions_2d, print_model_info
from visualize_2d_robot import plot_robot_2d, compare_configurations, analyze_joint_movements

def test_basic_urdf_loading():
    """
    基本的なURDF読み込みテスト
    """
    print("=== Test 1: Basic URDF Loading ===")
    
    try:
        model, data, joint_names, link_names = load_mimic_robot_model()
        print("✅ URDF loaded successfully")
        
        print_model_info(model, joint_names, link_names)
        return model, data, joint_names, link_names
        
    except Exception as e:
        print(f"❌ URDF loading failed: {e}")
        return None

def test_neutral_configuration(model, data):
    """
    中立設定での順運動学テスト
    """
    print("\n=== Test 2: Neutral Configuration Forward Kinematics ===")
    
    try:
        # 中立設定
        q_neutral = create_configuration_vector(model)
        print(f"Neutral configuration vector (nq={len(q_neutral)}): {q_neutral}")
        
        # 順運動学計算
        joint_positions, link_info = get_joint_positions_2d(model, data, q_neutral)
        print("✅ Neutral configuration FK completed")
        
        return joint_positions, q_neutral
        
    except Exception as e:
        print(f"❌ Neutral configuration FK failed: {e}")
        return None, None

def test_specified_configuration(model, data):
    """
    指定設定での順運動学テスト (hip=52°, knee=-104°)
    """
    print("\n=== Test 3: Specified Configuration Forward Kinematics ===")
    print("Target: hip_pitch = 52°, knee = -104°")
    
    try:
        # 指定角度
        hip_angle = np.radians(52)   # 52° → rad
        knee_angle = np.radians(-104) # -104° → rad
        
        joint_angles = {
            'upper_link_R_joint': hip_angle,
            'lower_link_R_joint': knee_angle,
            'wheel_R_joint': 0.0,
            'upper_link_L_joint': hip_angle,   # 左脚も同様
            'lower_link_L_joint': knee_angle,
            'wheel_L_joint': 0.0
        }
        
        print(f"Joint angles [rad]: {joint_angles}")
        
        # 設定ベクトル作成
        q_specified = create_configuration_vector(
            model, 
            base_pos=[0.0, 0.0, 0.0],  # ベース位置
            joint_angles=joint_angles
        )
        
        print(f"Configuration vector: {q_specified}")
        
        # 順運動学計算
        joint_positions, link_info = get_joint_positions_2d(model, data, q_specified)
        print("✅ Specified configuration FK completed")
        
        return joint_positions, q_specified
        
    except Exception as e:
        print(f"❌ Specified configuration FK failed: {e}")
        return None, None

def test_physical_validation(joint_positions_neutral, joint_positions_specified):
    """
    物理的妥当性の検証
    """
    print("\n=== Test 4: Physical Validation ===")
    
    try:
        # 右脚の関節チェーン検証
        right_leg_joints = ['base_link', 'upper_link_R', 'lower_link_R', 'wheel_R']
        
        print("Right leg chain validation:")
        for i, joint_name in enumerate(right_leg_joints):
            if joint_name in joint_positions_specified:
                pos = joint_positions_specified[joint_name]
                print(f"  {i}: {joint_name} -> [{pos[0]:.4f}, {pos[1]:.4f}]")
        
        # リンク長計算（概算）
        if all(joint in joint_positions_specified for joint in right_leg_joints):
            base_pos = np.array(joint_positions_specified['base_link'])
            upper_pos = np.array(joint_positions_specified['upper_link_R'])
            lower_pos = np.array(joint_positions_specified['lower_link_R'])
            wheel_pos = np.array(joint_positions_specified['wheel_R'])
            
            upper_length = np.linalg.norm(upper_pos - base_pos)
            lower_length = np.linalg.norm(lower_pos - upper_pos)
            wheel_offset = np.linalg.norm(wheel_pos - lower_pos)
            
            print(f"\nEstimated link lengths:")
            print(f"  Base to Upper: {upper_length:.4f} m")
            print(f"  Upper to Lower: {lower_length:.4f} m")
            print(f"  Lower to Wheel: {wheel_offset:.4f} m")
            
            # ホイール地面接触確認
            wheel_height = wheel_pos[1]  # Z座標
            print(f"  Wheel height: {wheel_height:.4f} m")
            
            if wheel_height < 0:
                print("  ⚠️  Warning: Wheel is below ground level")
            else:
                print("  ✅ Wheel is above ground level")
        
        print("✅ Physical validation completed")
        return True
        
    except Exception as e:
        print(f"❌ Physical validation failed: {e}")
        return False

def test_visualization(joint_positions_neutral, joint_positions_specified):
    """
    可視化テスト
    """
    print("\n=== Test 5: Visualization ===")
    
    try:
        # 比較可視化
        print("Generating comparison visualization...")
        compare_configurations(
            [joint_positions_neutral, joint_positions_specified],
            ["Neutral (0°)", "Specified (52°, -104°)"],
            "URDF Robot: Forward Kinematics Test"
        )
        
        # 移動量分析
        movement_analysis = analyze_joint_movements(joint_positions_neutral, joint_positions_specified)
        
        print("✅ Visualization test completed")
        return True
        
    except Exception as e:
        print(f"❌ Visualization test failed: {e}")
        return False

def run_comprehensive_test():
    """
    包括的なテスト実行
    """
    print("🚀 Starting URDF Forward Kinematics Comprehensive Test")
    print("=" * 60)
    
    test_results = {}
    
    # Test 1: URDF読み込み
    result = test_basic_urdf_loading()
    if result is None:
        print("❌ Critical failure: Cannot proceed without URDF model")
        return False
    
    model, data, joint_names, link_names = result
    test_results['urdf_loading'] = True
    
    # Test 2: 中立設定
    joint_pos_neutral, q_neutral = test_neutral_configuration(model, data)
    if joint_pos_neutral is None:
        print("❌ Critical failure: Neutral configuration failed")
        return False
    test_results['neutral_config'] = True
    
    # Test 3: 指定設定
    joint_pos_specified, q_specified = test_specified_configuration(model, data)
    if joint_pos_specified is None:
        print("❌ Critical failure: Specified configuration failed")
        return False
    test_results['specified_config'] = True
    
    # Test 4: 物理的妥当性
    test_results['physical_validation'] = test_physical_validation(joint_pos_neutral, joint_pos_specified)
    
    # Test 5: 可視化
    test_results['visualization'] = test_visualization(joint_pos_neutral, joint_pos_specified)
    
    # 結果サマリー
    print("\n" + "=" * 60)
    print("🏁 Test Summary")
    print("=" * 60)
    
    all_passed = True
    for test_name, passed in test_results.items():
        status = "✅ PASS" if passed else "❌ FAIL"
        print(f"  {test_name}: {status}")
        if not passed:
            all_passed = False
    
    if all_passed:
        print("\n🎉 All tests passed successfully!")
        print("URDF forward kinematics system is working correctly.")
    else:
        print("\n⚠️  Some tests failed. Please check the implementation.")
    
    return all_passed

def demo_urdf_forward_kinematics():
    """
    ユーザー向けデモンストレーション
    """
    print("=== URDF Forward Kinematics Demo ===")
    print("This demo shows URDF robot model loading and forward kinematics")
    print("with specified joint angles: hip_pitch=52°, knee=-104°")
    print()
    
    success = run_comprehensive_test()
    
    if success:
        print("\n✨ Demo completed successfully!")
        print("Check the generated plots to see the robot configurations.")
    else:
        print("\n💥 Demo encountered issues. Please check the error messages above.")
    
    return success

if __name__ == "__main__":
    # 仮想環境確認
    try:
        import pinocchio as pin
        print("Pinocchio library loaded successfully")
        print()
    except ImportError:
        print("❌ Pinocchio library not found")
        print("Please activate virtual environment: . bin/activate")
        exit(1)
    
    # デモ実行
    demo_urdf_forward_kinematics()
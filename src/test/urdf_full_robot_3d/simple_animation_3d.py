#!/usr/bin/env python3
"""
新しいアニメーションシステムのテスト用ファイル
animation_3d.py を使用した統合テスト
"""

from fixed_robot_3d import FixedRobot3D, run_simulation_with_animation

def test_new_animation_system():
    """新しいアニメーションシステムのテスト"""
    print("=== 新アニメーションシステムテスト ===")
    
    # 基本シミュレーション&アニメーション（8自由度対応）
    results, anim = run_simulation_with_animation(
        initial_state=[0.0, 0.0, -0.15, -0.1, 0.2, 0.2, 0.0, 0.0],
        T_sim=1.0,
        dt=0.005
    )
    
    if results is not None:
        print(f"✅ シミュレーション成功: {len(results['time'])} ステップ")
        
        # 結果サマリ
        final_state = results['state'][-1]
        print(f"最終状態:")
        print(f"  位置: [{final_state[0]:.3f}, {final_state[1]:.3f}]")
        print(f"  膝関節: [{final_state[2]:.3f}, {final_state[3]:.3f}]")
        print(f"  腰関節: [{final_state[4]:.3f}, {final_state[5]:.3f}]")
        print(f"  ホイール: [{final_state[6]:.3f}, {final_state[7]:.3f}]")
    
    if anim is not None:
        print("✅ アニメーション作成成功")
    
    return results, anim

def test_asymmetric_scenario():
    """非対称シナリオのテスト"""
    print("\n=== 非対称シナリオテスト ===")
    
    # 左右で異なる初期角度（8自由度対応）
    asymmetric_state = [0.0, 0.0, -0.2, -0.05, 0.3, 0.1, 0.1, -0.1]
    
    results, anim = run_simulation_with_animation(
        initial_state=asymmetric_state,
        T_sim=1.5,
        dt=0.005
    )
    
    return results, anim

if __name__ == "__main__":
    # 基本テスト
    test_new_animation_system()
    
    # 非対称テスト  
    test_asymmetric_scenario()
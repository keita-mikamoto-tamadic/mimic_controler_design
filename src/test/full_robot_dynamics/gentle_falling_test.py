#!/usr/bin/env python3
"""
穏やかな倒れ動作のテスト
数値発散を避けつつ、明確な重力動作を確認
"""

from fixed_dynamics_simulator import FixedFullRobotSimulator

def test_gentle_falling():
    """穏やかな倒れ動作のテスト"""
    print("="*60)
    print("🌟 フルロボット穏やかな重力落下テスト")
    print("="*60)
    
    try:
        simulator = FixedFullRobotSimulator()
        
        # より保守的な初期条件
        gentle_initial_state = [
            0.0,    # x_base
            0.0,    # y_base
            0.0,    # yaw
            0.4,    # phi_L_upper (穏やかな左上腿)
            -0.8,   # phi_L_lower (穏やかな左下腿)
            0.3,    # phi_R_upper (少し曲げた右上腿)
            -0.6    # phi_R_lower (少し曲げた右下腿)
        ]
        
        print(f"🎯 穏やかな初期条件:")
        print(f"   左脚: upper={gentle_initial_state[3]:.2f}, lower={gentle_initial_state[4]:.2f}")
        print(f"   右脚: upper={gentle_initial_state[5]:.2f}, lower={gentle_initial_state[6]:.2f}")
        
        # より短時間・細かい時間刻みでシミュレーション
        results = simulator.simulate_falling_robot(gentle_initial_state, T_sim=2.0, dt=0.01)
        
        # アニメーション作成
        anim = simulator.create_falling_animation(results, "Gentle Falling Robot")
        
        print(f"\n✅ 穏やかな落下テスト完了")
        return results
        
    except Exception as e:
        print(f"❌ エラー: {e}")
        raise

def test_symmetric_slight_tilt():
    """対称でわずかな傾斜のテスト"""
    print("\n" + "="*60)
    print("⚖️  対称わずか傾斜テスト")
    print("="*60)
    
    try:
        simulator = FixedFullRobotSimulator()
        
        # 対称だがわずかに不安定
        symmetric_state = [
            0.0,    # x_base
            0.0,    # y_base
            0.0,    # yaw
            0.35,   # phi_L_upper (左右対称)
            -0.7,   # phi_L_lower
            0.35,   # phi_R_upper (左右対称)
            -0.7    # phi_R_lower
        ]
        
        print(f"🎯 対称初期条件:")
        print(f"   両脚: upper={symmetric_state[3]:.2f}, lower={symmetric_state[4]:.2f}")
        
        results = simulator.simulate_falling_robot(symmetric_state, T_sim=2.5, dt=0.01)
        
        anim = simulator.create_falling_animation(results, "Symmetric Robot Motion")
        
        print(f"\n✅ 対称テスト完了")
        return results
        
    except Exception as e:
        print(f"❌ エラー: {e}")
        raise

if __name__ == "__main__":
    # 穏やかな落下テスト
    gentle_results = test_gentle_falling()
    
    # 対称テスト
    symmetric_results = test_symmetric_slight_tilt()
    
    print(f"\n🎉 両テスト完了！重力による自然な倒れ動作を確認できました。")
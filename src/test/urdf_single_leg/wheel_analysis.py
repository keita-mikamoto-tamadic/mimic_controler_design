#!/usr/bin/env python3
import numpy as np

def estimate_wheel_radius_from_inertia():
    """
    URDFの慣性データからホイール半径を推定
    
    ホイールの慣性データ:
    - mass = 0.643498 kg
    - ixx = 0.001232 kg⋅m²
    - iyy = 0.00212 kg⋅m²  
    - izz = 0.001234 kg⋅m²
    
    円柱状ホイールの慣性モーメント:
    - Ixx = Izz = (1/12)m(3r² + h²) (軸に垂直方向)
    - Iyy = (1/2)mr² (軸方向)
    """
    
    mass = 0.643498  # kg
    ixx = 0.001232   # kg⋅m²
    iyy = 0.00212    # kg⋅m²
    izz = 0.001234   # kg⋅m²
    
    print("ホイール慣性データ分析:")
    print(f"質量: {mass:.6f} kg")
    print(f"Ixx: {ixx:.6f} kg⋅m²")
    print(f"Iyy: {iyy:.6f} kg⋅m²")
    print(f"Izz: {izz:.6f} kg⋅m²")
    
    # 軸方向慣性から半径推定: Iyy = (1/2)mr²
    radius_from_iyy = np.sqrt(2 * iyy / mass)
    
    print(f"\n軸方向慣性からの半径推定:")
    print(f"r = sqrt(2*Iyy/m) = {radius_from_iyy:.6f} m = {radius_from_iyy*1000:.2f} mm")
    
    # 垂直方向慣性から推定（高さを仮定）
    # Ixx = (1/12)m(3r² + h²) より r² = (12*Ixx/m - h²)/3
    
    assumed_heights = [0.02, 0.03, 0.04, 0.05]  # 20-50mm
    print(f"\n垂直方向慣性からの半径推定（高さ仮定）:")
    
    for h in assumed_heights:
        discriminant = 12 * ixx / mass - h**2
        if discriminant > 0:
            radius_from_ixx = np.sqrt(discriminant / 3)
            print(f"高さ {h*1000:.0f}mm仮定時: r = {radius_from_ixx:.6f} m = {radius_from_ixx*1000:.2f} mm")
        else:
            print(f"高さ {h*1000:.0f}mm仮定時: 負の値（不適切）")
    
    print(f"\n推奨ホイール半径: {radius_from_iyy*1000:.1f} mm")
    return radius_from_iyy

def check_wheel_contact_geometry():
    """
    URDFの構造からホイール接地点を確認
    """
    print("\nホイール接地点の幾何学的確認:")
    
    # Joint positions from URDF
    base_to_upper = np.array([0.0, -0.1325, 0.0])
    upper_to_lower = np.array([0.0, 0.003, -0.15]) 
    lower_to_wheel = np.array([0.0, -0.0545, -0.15])
    
    print("関節位置（親リンク座標系）:")
    print(f"base → upper: {base_to_upper}")
    print(f"upper → lower: {upper_to_lower}")
    print(f"lower → wheel: {lower_to_wheel}")
    
    # 中立姿勢でのホイール位置（概算）
    wheel_pos_neutral = base_to_upper + upper_to_lower + lower_to_wheel
    print(f"\n中立姿勢でのホイール中心位置: {wheel_pos_neutral}")
    print(f"ホイール中心の高さ: {wheel_pos_neutral[2]:.3f} m")
    
    return wheel_pos_neutral

if __name__ == "__main__":
    wheel_radius_estimated = estimate_wheel_radius_from_inertia()
    wheel_center = check_wheel_contact_geometry()
    
    # 実際の値（model_info.mdより）
    wheel_radius_actual = (77.95 / 2) / 1000  # [m]
    
    print(f"\n=== 結論 ===")
    print(f"推定ホイール半径: {wheel_radius_estimated:.4f} m ({wheel_radius_estimated*1000:.1f} mm)")
    print(f"実際のホイール半径: {wheel_radius_actual:.4f} m ({wheel_radius_actual*1000:.1f} mm)")
    print(f"推定誤差: {((wheel_radius_estimated - wheel_radius_actual) / wheel_radius_actual * 100):.1f}%")
    print(f"中立姿勢でのホイール中心高さ: {wheel_center[2]:.3f} m")
    print(f"地面からホイール中心までの距離: {abs(wheel_center[2]) + wheel_radius_actual:.3f} m")
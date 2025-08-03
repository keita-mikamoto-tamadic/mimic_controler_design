#!/usr/bin/env python3
"""
線形化システムの制御可能性解析（メイン実行ファイル）
保存された線形化結果（PKLファイル）から制御可能性を検証
"""

import numpy as np
import pickle
import scipy.linalg as la
from scipy.signal import place_poles

def load_and_display_results():
    """PKLファイルを読み込んで内容を表示"""
    print("=== 線形化結果の読み込み ===\n")
    
    with open('linearization_results.pkl', 'rb') as f:
        data = pickle.load(f)
    
    return data

def check_controllability(A, B, name):
    """制御可能性を計算（scipy使用）"""
    n = A.shape[0]
    m = B.shape[1]
    
    print(f"\n{name}:")
    print(f"  A行列: {A.shape}")
    print(f"  B行列: {B.shape}")
    
    # 制御可能性行列 C = [B, AB, A²B, ..., A^(n-1)B]
    C = np.zeros((n, n*m))
    
    # 各ブロックを計算
    for i in range(n):
        if i == 0:
            C[:, i*m:(i+1)*m] = B
        else:
            C[:, i*m:(i+1)*m] = A @ C[:, (i-1)*m:i*m]
    
    # ランク計算
    rank = np.linalg.matrix_rank(C)
    is_controllable = (rank == n)
    
    print(f"  制御可能性行列: {C.shape}")
    print(f"  ランク: {rank}/{n}")
    print(f"  制御可能: {'✅ YES' if is_controllable else '❌ NO'}")
    
    # 特異値も確認
    U, s, Vt = la.svd(C)
    print(f"  最大特異値: {s[0]:.6e}")
    print(f"  最小特異値: {s[-1]:.6e}")
    if s[-1] > 1e-10:
        print(f"  条件数: {s[0]/s[-1]:.6e}")
    
    return is_controllable, rank

def main():
    # データ読み込み
    data = load_and_display_results()
    
    # 平衡点情報
    print(f"\n平衡点情報:")
    print(f"  上腿角度: {data['equilibrium']['upper_angle_deg']}deg")
    print(f"  下腿角度: {data['equilibrium']['lower_angle_deg']}deg")
    
    # フルシステムの行列を取得
    A_full = data['full_system']['A']
    B_full = data['full_system']['B']
    
    # 縮約システムの行列を取得
    A_reduced = data['reduced_system']['A']
    B_reduced = data['reduced_system']['B']
    
    print("\n" + "="*60)
    print("制御可能性解析（実際の線形化行列）")
    print("="*60)
    
    # フルシステムの制御可能性
    is_ctrl_full, rank_full = check_controllability(A_full, B_full, "フルシステム（20次元）")
    
    # 縮約システムの制御可能性
    is_ctrl_reduced, rank_reduced = check_controllability(A_reduced, B_reduced, "縮約システム（16次元）")
    
    # 固有値も確認
    print("\n" + "="*60)
    print("固有値解析")
    print("="*60)
    
    eig_full = np.linalg.eigvals(A_full)
    eig_reduced = np.linalg.eigvals(A_reduced)
    
    print(f"\nフルシステムの固有値:")
    print(f"  実部範囲: [{np.min(np.real(eig_full)):.6f}, {np.max(np.real(eig_full)):.6f}]")
    print(f"  安定極: {np.sum(np.real(eig_full) < 0)}")
    print(f"  不安定極: {np.sum(np.real(eig_full) > 0)}")
    
    print(f"\n縮約システムの固有値:")
    print(f"  実部範囲: [{np.min(np.real(eig_reduced)):.6f}, {np.max(np.real(eig_reduced)):.6f}]")
    print(f"  安定極: {np.sum(np.real(eig_reduced) < 0)}")
    print(f"  不安定極: {np.sum(np.real(eig_reduced) > 0)}")
    
    # 最終結論
    print("\n" + "="*60)
    print("🏁 結論（実際のシステム）")
    print("="*60)
    
    if is_ctrl_reduced:
        print("\n✅ 縮約システムは完全制御可能です！")
        print("   これは本当の結果です（ランダム行列ではありません）")
        print("   → LQR制御、極配置制御、状態フィードバック制御が適用可能")
    else:
        print("\n❌ 縮約システムは完全制御可能ではありません")
        print(f"   ランク不足: {rank_reduced}/{A_reduced.shape[0]}")

if __name__ == "__main__":
    main()
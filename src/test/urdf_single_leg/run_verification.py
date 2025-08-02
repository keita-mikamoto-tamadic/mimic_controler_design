#!/usr/bin/env python3
"""
ノンスリップ拘束検証スクリプト
シミュレーション結果の事後検証を独立して実行
"""

import pickle
import numpy as np
from noslip_verification import comprehensive_constraint_verification, plot_constraint_verification_results

def load_simulation_results(filename):
    """シミュレーション結果を読み込み"""
    with open(filename, 'rb') as f:
        return pickle.load(f)

def verify_noslip_simulation(results):
    """ノンスリップシミュレーション結果の検証"""
    print("🔍 拘束精度検証を実行中...")
    
    if 'q_history' not in results or len(results['q_history']) == 0:
        print("❌ 検証用データ（q_history）が見つかりません")
        return
    
    # 検証実行
    verification_results = comprehensive_constraint_verification(
        results['t_array'], 
        results['q_history']
    )
    
    # 結果表示
    plot_constraint_verification_results(
        results['t_array'], 
        verification_results
    )

def run_verification_from_file(filename):
    """ファイルから結果を読み込んで検証実行"""
    try:
        results = load_simulation_results(filename)
        verify_noslip_simulation(results)
    except FileNotFoundError:
        print(f"❌ ファイルが見つかりません: {filename}")
    except Exception as e:
        print(f"❌ エラーが発生しました: {e}")

def run_verification_interactive():
    """インタラクティブに検証を実行"""
    from noslip_constrained import simulate_noslip_dynamics
    
    print("🧪 ノンスリップ拘束検証システム")
    print("シミュレーションを実行して検証します...")
    
    # 小さな傾斜でテスト
    print("\n1. 小さな傾斜のシミュレーション:")
    results1 = simulate_noslip_dynamics(0.3, -0.6)
    verify_noslip_simulation(results1)
    
    # 大きな傾斜でテスト
    print("\n2. 大きな傾斜のシミュレーション:")
    results2 = simulate_noslip_dynamics(0.8, -1.2)
    verify_noslip_simulation(results2)

if __name__ == "__main__":
    import sys
    
    if len(sys.argv) > 1:
        # ファイルから検証
        filename = sys.argv[1]
        run_verification_from_file(filename)
    else:
        # インタラクティブ検証
        run_verification_interactive()
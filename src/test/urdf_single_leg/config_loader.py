#!/usr/bin/env python3
"""
YAML設定ファイルから初期値を読み込む機能
"""

import yaml
import numpy as np
from pathlib import Path

def load_config(config_path="conf.yaml"):
    """YAML設定ファイルを読み込む"""
    config_file = Path(config_path)
    if not config_file.exists():
        raise FileNotFoundError(f"設定ファイルが見つかりません: {config_path}")
    
    with open(config_file, 'r', encoding='utf-8') as file:
        config = yaml.safe_load(file)
    
    return config

def get_simulation_params(config):
    """シミュレーションパラメータを取得"""
    sim_config = config['simulation']
    
    # 時間設定
    T_sim = sim_config['time']['T_sim']
    dt = sim_config['time']['dt']
    
    # 初期状態（度をラジアンに変換）
    initial = sim_config['initial_state']
    x_base = initial['x_base']
    pitch_base = np.deg2rad(initial['pitch_base_deg'])
    phi1 = initial['phi1']
    phi2 = initial['phi2']
    
    return {
        'T_sim': T_sim,
        'dt': dt,
        'x_base': x_base,
        'pitch_base': pitch_base,
        'phi1': phi1,
        'phi2': phi2
    }

def get_test_case(config, case_name):
    """指定されたテストケースの設定を取得"""
    if 'test_cases' not in config:
        raise KeyError("test_casesが設定ファイルに定義されていません")
    
    if case_name not in config['test_cases']:
        available_cases = list(config['test_cases'].keys())
        raise KeyError(f"テストケース '{case_name}' が見つかりません。利用可能: {available_cases}")
    
    case = config['test_cases'][case_name]
    
    return {
        'name': case['name'],
        'x_base': case['x_base'],
        'pitch_base': np.deg2rad(case['pitch_base_deg']),
        'phi1': case['phi1'],
        'phi2': case['phi2']
    }

def list_test_cases(config):
    """利用可能なテストケース一覧を表示"""
    if 'test_cases' not in config:
        print("テストケースが定義されていません")
        return
    
    print("利用可能なテストケース:")
    for case_name, case_data in config['test_cases'].items():
        print(f"  {case_name}: {case_data['name']} (pitch={case_data['pitch_base_deg']}°, φ1={case_data['phi1']}, φ2={case_data['phi2']})")

def get_output_settings(config):
    """出力設定を取得"""
    if 'output' not in config:
        # デフォルト設定
        return {
            'save_animation': True,
            'save_plots': True,
            'animation_fps': 20,
            'plot_dpi': 150
        }
    
    output_config = config['output']
    return {
        'save_animation': output_config.get('save_animation', True),
        'save_plots': output_config.get('save_plots', True),
        'animation_fps': output_config.get('animation_fps', 20),
        'plot_dpi': output_config.get('plot_dpi', 150)
    }

if __name__ == "__main__":
    # テスト用
    print("🔧 設定ファイルテスト")
    
    try:
        config = load_config()
        print("✅ conf.yaml読み込み成功")
        
        # 基本シミュレーション設定
        sim_params = get_simulation_params(config)
        print(f"シミュレーション設定: T={sim_params['T_sim']}s, dt={sim_params['dt']}s")
        print(f"初期状態: x={sim_params['x_base']}, pitch={np.rad2deg(sim_params['pitch_base']):.1f}°, φ1={sim_params['phi1']}, φ2={sim_params['phi2']}")
        
        # テストケース一覧
        print()
        list_test_cases(config)
        
        # 特定のテストケース
        print()
        case = get_test_case(config, 'case3')
        print(f"case3: {case['name']} - pitch={np.rad2deg(case['pitch_base']):.1f}°")
        
        # 出力設定
        output = get_output_settings(config)
        print(f"出力設定: アニメーション={output['save_animation']}, グラフ={output['save_plots']}")
        
    except Exception as e:
        print(f"❌ エラー: {e}")
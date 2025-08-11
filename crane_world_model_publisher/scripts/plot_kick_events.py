#!/usr/bin/env python3

"""
キックイベント可視化スクリプト - ROS2版
JSONデータファイルからキックイベントのグラフを生成する汎用スクリプト
"""

import argparse
import json
import sys
import os
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np


def load_kick_data(data_file_path: str) -> dict:
    """JSONデータファイルを読み込み"""
    if not os.path.exists(data_file_path):
        raise FileNotFoundError(f"データファイルが見つかりません: {data_file_path}")
    
    with open(data_file_path, 'r') as f:
        data = json.load(f)
    
    # データ構造の検証
    required_keys = ['event_info', 'data']
    for key in required_keys:
        if key not in data:
            raise KeyError(f"データファイルに必要なキー '{key}' が見つかりません")
    
    return data


def create_kick_plot(data: dict, output_path: str = None) -> None:
    """キックイベントのプロットを生成"""
    # データ抽出
    event_info = data['event_info']
    time = data['data']['time']
    pos_x = data['data']['position']['x']
    pos_y = data['data']['position']['y']
    vel_x = data['data']['velocity']['x']
    vel_y = data['data']['velocity']['y']
    speed = data['data']['speed']
    
    kick_pos_x = event_info['kick_position']['x']
    kick_pos_y = event_info['kick_position']['y']
    event_idx = event_info['event_index']
    
    # グラフ生成
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle(f'Kick Event {event_idx} Analysis', fontsize=16)
    
    # ボール軌道 (XY)
    ax1.plot(pos_x, pos_y, 'b-', linewidth=2, label='Ball trajectory')
    ax1.scatter([kick_pos_x], [kick_pos_y], color='red', s=100, marker='*', label='Kick position', zorder=5)
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_title('Ball Trajectory (XY)')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.set_aspect('equal')
    
    # 位置 vs 時間
    ax2.plot(time, pos_x, 'b-', label='X position', linewidth=2)
    ax2.plot(time, pos_y, 'g-', label='Y position', linewidth=2)
    ax2.axvline(x=0, color='red', linestyle='--', alpha=0.7, label='Kick time')
    ax2.set_xlabel('Time relative to kick (s)')
    ax2.set_ylabel('Position (m)')
    ax2.set_title('Position vs Time')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    # 速度 vs 時間
    ax3.plot(time, vel_x, 'b-', label='X velocity', linewidth=2)
    ax3.plot(time, vel_y, 'g-', label='Y velocity', linewidth=2)
    ax3.plot(time, speed, 'r-', label='Speed', linewidth=3)
    ax3.axvline(x=0, color='red', linestyle='--', alpha=0.7, label='Kick time')
    ax3.set_xlabel('Time relative to kick (s)')
    ax3.set_ylabel('Velocity (m/s)')
    ax3.set_title('Velocity vs Time')
    ax3.grid(True, alpha=0.3)
    ax3.legend()
    
    # 速度ベクトル（データ点数に応じて間引き）
    step = max(1, len(pos_x) // 20)  # 最大20個のベクトルを表示
    ax4.quiver(pos_x[::step], pos_y[::step], vel_x[::step], vel_y[::step],
               angles='xy', scale_units='xy', scale=1, alpha=0.7)
    ax4.plot(pos_x, pos_y, 'b-', alpha=0.5, linewidth=1)
    ax4.scatter([kick_pos_x], [kick_pos_y], color='red', s=100, marker='*', label='Kick position', zorder=5)
    ax4.set_xlabel('X Position (m)')
    ax4.set_ylabel('Y Position (m)')
    ax4.set_title('Ball Velocity Vectors')
    ax4.grid(True, alpha=0.3)
    ax4.set_aspect('equal')
    ax4.legend()
    
    plt.tight_layout()
    
    # 出力ファイル名を決定
    if output_path is None:
        data_path = Path(sys.argv[1] if len(sys.argv) > 1 else 'kick_data.json')
        output_path = str(data_path.with_suffix('.png').with_name(data_path.stem.replace('_data', '_plot')))
    
    plt.savefig(output_path, dpi=300, bbox_inches='tight')
    print(f'グラフ保存: {output_path}')
    
    return output_path


def print_data_summary(data: dict) -> None:
    """データの概要を表示"""
    event_info = data['event_info']
    data_count = event_info.get('data_points_count', len(data['data']['time']))
    max_speed = max(data['data']['speed'])
    avg_speed = np.mean(data['data']['speed'])
    
    print(f"キックイベント {event_info['event_index']} の概要:")
    print(f"  キック位置: ({event_info['kick_position']['x']:.3f}, {event_info['kick_position']['y']:.3f})")
    print(f"  データ点数: {data_count}")
    print(f"  最大速度: {max_speed:.3f} m/s")
    print(f"  平均速度: {avg_speed:.3f} m/s")
    print(f"  時間窓: ±{event_info.get('time_window_seconds', 3.0):.1f}秒")


def main():
    parser = argparse.ArgumentParser(
        description='キックイベントデータの可視化',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
使用例:
  ros2 run crane_world_model_publisher plot_kick_events.py kick_event_visualization_0_data.json
  ros2 run crane_world_model_publisher plot_kick_events.py data.json --output custom_plot.png
  ros2 run crane_world_model_publisher plot_kick_events.py data.json --summary-only
        ''')
    
    parser.add_argument('data_file', help='JSONデータファイルのパス')
    parser.add_argument('--output', '-o', help='出力画像ファイルのパス（指定しない場合は自動生成）')
    parser.add_argument('--summary-only', action='store_true', help='グラフ生成をスキップしてデータ概要のみ表示')
    parser.add_argument('--no-display', action='store_true', help='プロットを表示しない（ファイル保存のみ）')
    
    args = parser.parse_args()
    
    try:
        # データ読み込み
        data = load_kick_data(args.data_file)
        
        # データ概要表示
        print_data_summary(data)
        
        if not args.summary_only:
            # プロット生成
            output_path = create_kick_plot(data, args.output)
            print(f'データファイル: {args.data_file}')
            
            if not args.no_display:
                try:
                    plt.show()
                except Exception as e:
                    print(f'ディスプレイエラー（ファイルは保存済み）: {e}')
        
    except Exception as e:
        print(f'エラー: {e}', file=sys.stderr)
        sys.exit(1)


if __name__ == '__main__':
    main()
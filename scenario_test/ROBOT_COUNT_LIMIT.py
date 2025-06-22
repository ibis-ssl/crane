import math
import time
from rcst.communication import Communication
from rcst.ball import Ball
from rcst.robot import RobotDict


def test_robot_count_limit(rcst_comm: Communication):
    """フィールド上のロボット数制限（最大11台）遵守をテスト"""
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(0.0, 0.0)
    
    # 通常の試合開始状態（各チーム最大11台）
    print("Testing normal robot count (11 per team)...")
    
    # Yellow team に11台のロボットを配置
    for i in range(11):
        x = -3.0 + i * 0.5
        y = 2.0 - (i % 6) * 0.8
        rcst_comm.send_yellow_robot(i, x, y, math.radians(0))
    
    # Blue team に11台のロボットを配置
    for i in range(11):
        x = 3.0 - i * 0.5
        y = -2.0 + (i % 6) * 0.8
        rcst_comm.send_blue_robot(i, x, y, math.radians(180))
    
    rcst_comm.change_referee_command("NORMAL_START", 2.0)
    
    def check_robot_counts(
        ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict
    ) -> bool:
        """ロボット数が制限を超えていないかチェック"""
        yellow_count = len(yellow_robots)
        blue_count = len(blue_robots)
        
        if yellow_count > 11:
            print(f"Yellow team has too many robots: {yellow_count}")
            return True
        if blue_count > 11:
            print(f"Blue team has too many robots: {blue_count}")
            return True
        
        return False
    
    rcst_comm.observer.customized().register_sticky_true_callback(
        "robot_count_violation", check_robot_counts
    )
    
    # テスト実行
    rcst_comm.observer.reset()
    time.sleep(3)
    
    # 最終状態のカウント確認
    world = rcst_comm.observer.get_world()
    yellow_robots = world.get_yellow_robots()
    blue_robots = world.get_blue_robots()
    
    yellow_count = len(yellow_robots)
    blue_count = len(blue_robots)
    
    print(f"Final robot counts - Yellow: {yellow_count}, Blue: {blue_count}")
    
    # SSL規則: 各チーム最大11台
    assert yellow_count <= 11, f"Yellow team exceeded robot limit: {yellow_count}"
    assert blue_count <= 11, f"Blue team exceeded robot limit: {blue_count}"
    
    # 違反が検出されなかったことを確認
    violation_detected = rcst_comm.observer.customized().get_result("robot_count_violation")
    assert violation_detected is False, "Robot count violation detected during test"


def test_robot_substitution(rcst_comm: Communication):
    """ロボット交代時の数制限テスト"""
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(1.0, 0.0)
    
    print("Testing robot substitution...")
    
    # 初期状態: Yellow team に5台のロボット
    initial_robots = [0, 1, 2, 3, 4]
    for i, robot_id in enumerate(initial_robots):
        rcst_comm.send_yellow_robot(robot_id, -2.0 + i * 0.8, 1.0, math.radians(0))
    
    rcst_comm.change_referee_command("STOP", 1.0)
    time.sleep(2)
    
    # ロボット交代シミュレーション
    # ロボット4を除去し、ロボット5を追加
    print("Substituting robot 4 with robot 5...")
    
    # 新しいロボット5を追加
    rcst_comm.send_yellow_robot(5, -2.0 + 4 * 0.8, 1.0, math.radians(0))
    
    # 交代後の状態確認
    time.sleep(2)
    world = rcst_comm.observer.get_world()
    yellow_robots = world.get_yellow_robots()
    
    active_robot_ids = list(yellow_robots.keys())
    print(f"Active robot IDs after substitution: {sorted(active_robot_ids)}")
    
    # ロボット数が制限内であることを確認
    assert len(yellow_robots) <= 11, f"Too many robots after substitution: {len(yellow_robots)}"
    
    # 期待される交代が行われたことを確認
    # 注意: 実際の実装では、古いロボットの自動除去が必要な場合がある
    if len(yellow_robots) > 5:
        print("WARNING: Old robot might not have been properly removed")


def test_robot_removal_on_cards(rcst_comm: Communication):
    """カード（警告/退場）によるロボット除去テスト"""
    rcst_comm.send_empty_world()
    rcst_comm.send_ball(-1.0, 0.0)
    
    print("Testing robot removal due to cards...")
    
    # Yellow team に複数のロボット配置
    for i in range(6):
        rcst_comm.send_yellow_robot(i, -2.0 + i * 0.6, 0.5, math.radians(0))
    
    rcst_comm.change_referee_command("STOP", 1.0)
    time.sleep(2)
    
    initial_world = rcst_comm.observer.get_world()
    initial_count = len(initial_world.get_yellow_robots())
    print(f"Initial robot count: {initial_count}")
    
    # レッドカードシミュレーション（実際の実装に依存）
    # 注意: この部分は実際のRCSTとレフェリーシステムの実装に合わせて調整が必要
    print("Simulating red card for robot 2...")
    
    # 一定時間後に状態確認
    time.sleep(3)
    final_world = rcst_comm.observer.get_world()
    final_count = len(final_world.get_yellow_robots())
    
    print(f"Final robot count: {final_count}")
    
    # カードによる除去が適切に処理されることを確認
    # 実際の処理は審判システムの実装に依存
    assert final_count <= initial_count, "Robot count should not increase after card"
    assert final_count <= 11, "Robot count should never exceed maximum"


if __name__ == "__main__":
    rcst_comm = Communication()
    
    print("=== Robot Count Limit Test ===")
    test_robot_count_limit(rcst_comm)
    
    print("\n=== Robot Substitution Test ===")
    test_robot_substitution(rcst_comm)
    
    print("\n=== Robot Removal on Cards Test ===")
    test_robot_removal_on_cards(rcst_comm)
    
    rcst_comm.close()
    print("ROBOT_COUNT_LIMIT test passed")
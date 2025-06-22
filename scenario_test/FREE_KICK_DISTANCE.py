import math
import time
from rcst.communication import Communication
from rcst import calc
from rcst.ball import Ball
from rcst.robot import RobotDict


def test_free_kick_distance(rcst_comm: Communication):
    """フリーキック時に相手ロボットが規定距離（500mm）を保つことをテスト"""
    rcst_comm.send_empty_world()
    
    # ボールをフリーキック位置に配置
    ball_x, ball_y = 2.0, 1.0
    rcst_comm.send_ball(ball_x, ball_y)
    
    # 味方ロボット（Yellow）をフリーキック実行位置に配置
    rcst_comm.send_yellow_robot(0, ball_x - 0.2, ball_y, math.radians(0))
    
    # 相手ロボット（Blue）を様々な距離に配置
    rcst_comm.send_blue_robot(0, ball_x + 0.3, ball_y, math.radians(180))  # 300mm - 違反
    rcst_comm.send_blue_robot(1, ball_x + 0.4, ball_y + 0.2, math.radians(180))  # 400mm - 違反
    rcst_comm.send_blue_robot(2, ball_x + 0.6, ball_y, math.radians(180))  # 600mm - 合法
    rcst_comm.send_blue_robot(3, ball_x, ball_y + 0.7, math.radians(270))  # 700mm - 合法
    
    # Yellow team のフリーキックを設定
    rcst_comm.change_referee_command("DIRECT_FREE_YELLOW", 3.0)
    
    def blue_robots_too_close(
        ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict
    ) -> bool:
        """相手ロボットがボールから500mm以内にいるかチェック"""
        violating_robots = []
        for robot_id, robot in blue_robots.items():
            distance = calc.distance_robot_and_ball(robot, ball)
            if distance < 0.5:  # 500mm
                violating_robots.append((robot_id, distance))
        
        if violating_robots:
            print(f"Blue robots violating free kick distance: {violating_robots}")
            return True
        return False
    
    rcst_comm.observer.customized().register_sticky_true_callback(
        "blue_robots_too_close", blue_robots_too_close
    )
    
    # テスト実行 - 相手ロボットが適切に離れるかを確認
    rcst_comm.observer.reset()
    time.sleep(4)  # システムが反応する時間を与える
    
    # 最終状態をチェック
    world = rcst_comm.observer.get_world()
    ball = world.get_ball()
    blue_robots = world.get_blue_robots()
    
    violating_count = 0
    distances = {}
    for robot_id, robot in blue_robots.items():
        distance = calc.distance_robot_and_ball(robot, ball)
        distances[robot_id] = distance
        if distance < 0.5:
            violating_count += 1
    
    print("Blue robot distances from ball during free kick:")
    for robot_id, distance in distances.items():
        status = "VIOLATION" if distance < 0.5 else "OK"
        print(f"  Robot {robot_id}: {distance:.3f}m ({status})")
    
    # SSL規則: フリーキック時は相手ロボットがボールから500mm以上離れる必要がある
    assert violating_count == 0, f"Blue robots ({violating_count}) violated free kick distance rule"
    
    # 違反が検出されなかったことを確認
    violation_detected = rcst_comm.observer.customized().get_result("blue_robots_too_close")
    assert violation_detected is False, "Blue robots did not maintain proper distance during free kick"


def test_free_kick_distance_blue(rcst_comm: Communication):
    """Blue team のフリーキック時のテスト"""
    rcst_comm.send_empty_world()
    
    # ボールを別の位置に配置
    ball_x, ball_y = -1.5, -0.8
    rcst_comm.send_ball(ball_x, ball_y)
    
    # Blue team のフリーキック実行ロボット
    rcst_comm.send_blue_robot(0, ball_x + 0.2, ball_y, math.radians(180))
    
    # Yellow team の相手ロボットを配置
    rcst_comm.send_yellow_robot(0, ball_x - 0.35, ball_y, math.radians(0))  # 350mm - 違反
    rcst_comm.send_yellow_robot(1, ball_x - 0.55, ball_y + 0.3, math.radians(0))  # 550mm - 合法
    
    # Blue team のフリーキックを設定
    rcst_comm.change_referee_command("DIRECT_FREE_BLUE", 3.0)
    
    def yellow_robots_too_close(
        ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict
    ) -> bool:
        """Yellow team ロボットがボールから500mm以内にいるかチェック"""
        violating_robots = []
        for robot_id, robot in yellow_robots.items():
            distance = calc.distance_robot_and_ball(robot, ball)
            if distance < 0.5:
                violating_robots.append((robot_id, distance))
        
        if violating_robots:
            print(f"Yellow robots violating free kick distance: {violating_robots}")
            return True
        return False
    
    rcst_comm.observer.customized().register_sticky_true_callback(
        "yellow_robots_too_close", yellow_robots_too_close
    )
    
    # テスト実行
    rcst_comm.observer.reset()
    time.sleep(4)
    
    # 最終状態をチェック
    world = rcst_comm.observer.get_world()
    ball = world.get_ball()
    yellow_robots = world.get_yellow_robots()
    
    violating_count = 0
    for robot_id, robot in yellow_robots.items():
        distance = calc.distance_robot_and_ball(robot, ball)
        if distance < 0.5:
            violating_count += 1
        print(f"  Yellow Robot {robot_id}: {distance:.3f}m")
    
    assert violating_count == 0, f"Yellow robots violated free kick distance rule"
    assert rcst_comm.observer.customized().get_result("yellow_robots_too_close") is False


if __name__ == "__main__":
    rcst_comm = Communication()
    
    print("Testing Yellow team free kick...")
    test_free_kick_distance(rcst_comm)
    
    print("Testing Blue team free kick...")
    test_free_kick_distance_blue(rcst_comm)
    
    rcst_comm.close()
    print("FREE_KICK_DISTANCE test passed")
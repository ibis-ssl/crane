import math
import time
from rcst.communication import Communication
from rcst import calc
from rcst.ball import Ball
from rcst.robot import RobotDict


def test_ball_possession_control(rcst_comm: Communication):
    """ボール保持制御のテスト"""
    rcst_comm.send_empty_world()
    
    # ボールとロボットを近接配置
    ball_x, ball_y = 0.0, 0.0
    robot_x, robot_y = ball_x - 0.15, ball_y  # ロボットをボールの近くに配置
    
    rcst_comm.send_ball(ball_x, ball_y)
    rcst_comm.send_yellow_robot(0, robot_x, robot_y, math.radians(0))
    
    rcst_comm.change_referee_command("FORCE_START", 2.0)
    
    def ball_under_control(
        ball: Ball, blue_robots: RobotDict, yellow_robots: RobotDict
    ) -> bool:
        """ボールがロボットの制御下にあるかチェック"""
        if 0 not in yellow_robots:
            return False
        
        robot = yellow_robots[0]
        distance = calc.distance_robot_and_ball(robot, ball)
        
        # ボールがロボットから200mm以内にあり、かつ安定している場合を制御下と判定
        return distance <= 0.2
    
    rcst_comm.observer.customized().register_sticky_true_callback(
        "ball_under_control", ball_under_control
    )
    
    # テスト実行
    rcst_comm.observer.reset()
    possession_time = 0
    test_duration = 5.0
    
    for i in range(int(test_duration * 10)):  # 0.1秒間隔で監視
        time.sleep(0.1)
        
        world = rcst_comm.observer.get_world()
        ball = world.get_ball()
        yellow_robots = world.get_yellow_robots()
        
        if ball_under_control(ball, {}, yellow_robots):
            possession_time += 0.1
    
    possession_rate = possession_time / test_duration
    print(f"Ball possession time: {possession_time:.1f}s / {test_duration}s ({possession_rate:.1%})")
    
    # 最低60%の時間でボールを制御下に置くことを要求
    assert possession_rate >= 0.6, f"Ball possession rate insufficient: {possession_rate:.1%}"
    
    return possession_rate


def test_ball_dribbling_control(rcst_comm: Communication):
    """ボールドリブリング制御のテスト"""
    rcst_comm.send_empty_world()
    
    # ドリブリング経路の設定
    start_x, start_y = -2.0, 0.0
    end_x, end_y = 2.0, 0.0
    
    rcst_comm.send_ball(start_x, start_y)
    rcst_comm.send_yellow_robot(0, start_x - 0.15, start_y, math.radians(0))
    
    rcst_comm.change_referee_command("FORCE_START", 1.0)
    
    # ドリブリング開始
    start_time = time.time()
    max_dribble_time = 8.0
    
    ball_positions = []
    robot_positions = []
    
    while time.time() - start_time < max_dribble_time:
        time.sleep(0.2)
        
        world = rcst_comm.observer.get_world()
        ball = world.get_ball()
        yellow_robots = world.get_yellow_robots()
        
        ball_positions.append((ball.x, ball.y, time.time()))
        
        if 0 in yellow_robots:
            robot = yellow_robots[0]
            robot_positions.append((robot.x, robot.y, time.time()))
            
            # 目標に到達したかチェック
            distance_to_goal = calc.distance(ball.x, ball.y, end_x, end_y)
            if distance_to_goal < 0.3:  # 目標から300mm以内
                break
    
    # ドリブリング評価
    if len(ball_positions) >= 10 and len(robot_positions) >= 10:
        # ボールとロボットの距離を評価
        close_control_count = 0
        total_measurements = min(len(ball_positions), len(robot_positions))
        
        for i in range(total_measurements):
            ball_pos = ball_positions[i]
            robot_pos = robot_positions[i]
            
            distance = calc.distance(ball_pos[0], ball_pos[1], robot_pos[0], robot_pos[1])
            if distance <= 0.25:  # 250mm以内で制御
                close_control_count += 1
        
        control_rate = close_control_count / total_measurements
        print(f"Dribbling control rate: {close_control_count}/{total_measurements} ({control_rate:.1%})")
        
        # 最終位置の評価
        final_ball_pos = ball_positions[-1]
        distance_to_target = calc.distance(final_ball_pos[0], final_ball_pos[1], end_x, end_y)
        print(f"Final distance to target: {distance_to_target:.2f}m")
        
        # 評価基準
        assert control_rate >= 0.7, f"Dribbling control rate insufficient: {control_rate:.1%}"
        assert distance_to_target <= 0.5, f"Did not reach target location: {distance_to_target:.2f}m"
        
        return control_rate, distance_to_target
    else:
        assert False, "Insufficient dribbling data collected"


def test_ball_passing_accuracy(rcst_comm: Communication):
    """ボールパスの精度テスト"""
    rcst_comm.send_empty_world()
    
    # パサーとレシーバーを配置
    passer_x, passer_y = -1.0, 0.0
    receiver_x, receiver_y = 1.5, 0.8
    
    rcst_comm.send_ball(passer_x, passer_y)
    rcst_comm.send_yellow_robot(0, passer_x - 0.15, passer_y, math.radians(0))  # パサー
    rcst_comm.send_yellow_robot(1, receiver_x, receiver_y, math.radians(180))    # レシーバー
    
    rcst_comm.change_referee_command("FORCE_START", 2.0)
    
    # パス実行の監視
    pass_start_time = time.time()
    ball_initial_pos = (passer_x, passer_y)
    
    ball_moving = False
    pass_completed = False
    max_pass_time = 5.0
    
    while time.time() - pass_start_time < max_pass_time:
        time.sleep(0.1)
        
        world = rcst_comm.observer.get_world()
        ball = world.get_ball()
        
        # ボールが初期位置から移動したかチェック
        if not ball_moving:
            distance_from_start = calc.distance(ball.x, ball.y, ball_initial_pos[0], ball_initial_pos[1])
            if distance_from_start > 0.3:  # 300mm以上移動
                ball_moving = True
                print("Ball started moving (pass initiated)")
        
        # レシーバーの近くにボールが到達したかチェック
        if ball_moving:
            distance_to_receiver = calc.distance(ball.x, ball.y, receiver_x, receiver_y)
            if distance_to_receiver <= 0.4:  # 400mm以内
                pass_completed = True
                print("Pass completed (ball reached receiver area)")
                break
    
    # パス精度の評価
    world = rcst_comm.observer.get_world()
    ball = world.get_ball()
    final_distance_to_receiver = calc.distance(ball.x, ball.y, receiver_x, receiver_y)
    
    print(f"Pass execution time: {time.time() - pass_start_time:.1f}s")
    print(f"Final distance to receiver: {final_distance_to_receiver:.2f}m")
    print(f"Ball moving: {ball_moving}")
    print(f"Pass completed: {pass_completed}")
    
    # 評価基準
    assert ball_moving, "Ball did not move (pass not initiated)"
    assert final_distance_to_receiver <= 0.6, f"Pass accuracy insufficient: {final_distance_to_receiver:.2f}m"
    
    return pass_completed, final_distance_to_receiver


def test_ball_shooting_power(rcst_comm: Communication):
    """ボールシュートの力加減テスト"""
    rcst_comm.send_empty_world()
    
    # シューターをゴール前に配置
    shooter_x, shooter_y = 4.0, 0.0
    goal_x, goal_y = 6.0, 0.0  # ゴール中央
    
    rcst_comm.send_ball(shooter_x, shooter_y)
    rcst_comm.send_yellow_robot(0, shooter_x - 0.2, shooter_y, math.radians(0))
    
    rcst_comm.change_referee_command("FORCE_START", 2.0)
    
    # シュート実行の監視
    shoot_start_time = time.time()
    ball_initial_pos = (shooter_x, shooter_y)
    
    max_speed = 0.0
    ball_speeds = []
    
    while time.time() - shoot_start_time < 4.0:
        time.sleep(0.05)
        
        world = rcst_comm.observer.get_world()
        ball = world.get_ball()
        
        # ボール速度の推定（簡易版）
        if len(ball_speeds) > 0:
            prev_pos = ball_speeds[-1]
            dt = 0.05
            dx = ball.x - prev_pos[0]
            dy = ball.y - prev_pos[1]
            speed = math.sqrt(dx*dx + dy*dy) / dt
            max_speed = max(max_speed, speed)
        
        ball_speeds.append((ball.x, ball.y))
        
        # ゴールラインを越えたかチェック
        if ball.x >= goal_x:
            break
    
    # シュート評価
    world = rcst_comm.observer.get_world()
    ball = world.get_ball()
    
    distance_from_start = calc.distance(ball.x, ball.y, ball_initial_pos[0], ball_initial_pos[1])
    distance_to_goal = calc.distance(ball.x, ball.y, goal_x, goal_y)
    
    print(f"Max estimated ball speed: {max_speed:.2f} m/s")
    print(f"Distance traveled: {distance_from_start:.2f}m")
    print(f"Final distance to goal: {distance_to_goal:.2f}m")
    
    # 評価基準
    assert distance_from_start >= 1.0, "Ball did not move significantly (shot not executed)"
    assert max_speed >= 1.0, f"Ball speed too low: {max_speed:.2f} m/s"
    assert max_speed <= 8.0, f"Ball speed too high (dangerous): {max_speed:.2f} m/s"
    
    return max_speed, distance_to_goal


if __name__ == "__main__":
    rcst_comm = Communication()
    
    print("=== Ball Possession Control Test ===")
    possession_rate = test_ball_possession_control(rcst_comm)
    
    print("\n=== Ball Dribbling Control Test ===")
    dribble_rate, dribble_accuracy = test_ball_dribbling_control(rcst_comm)
    
    print("\n=== Ball Passing Accuracy Test ===")
    pass_success, pass_accuracy = test_ball_passing_accuracy(rcst_comm)
    
    print("\n=== Ball Shooting Power Test ===")
    shot_speed, shot_accuracy = test_ball_shooting_power(rcst_comm)
    
    print(f"\n=== Summary ===")
    print(f"Possession rate: {possession_rate:.1%}")
    print(f"Dribbling control: {dribble_rate:.1%}")
    print(f"Pass accuracy: {pass_accuracy:.2f}m")
    print(f"Shot speed: {shot_speed:.2f} m/s")
    
    rcst_comm.close()
    print("BALL_HANDLING_CONTROL test passed")
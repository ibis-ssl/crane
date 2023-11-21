#!/usr/bin/env python3

import os
import sys
import time
from crane_msgs.msg import WorldModel, RobotCommands

import rclpy
import numpy as np

from rclpy.node import Node

from rclpy.qos import QoSProfile

from rclpy.qos import qos_profile_sensor_data


# class for training the network with 9 ∗ 10 = 90 values, in an
# angle interval between 10 degree and 170 degree, with a step of 20◦ ,
# and a velocity interval between 200 and 2000mm/s, with
# a step of 200mm/s. We also measure a group of 9 ∗ 10
# data for checking the accuracy of the network like the data
# above, except an angle interval between 20 degree and 180 degree.

#   robot controller class for collecting dataset
class RobotControllerForDatasetCollector(Node):
    def __init__(self):
        super().__init__('robot_controller_for_dataset_collector')
        self.pub_robot_commands = self.create_publisher(
            RobotCommands,
            '/robot_commands',
            10)
        self.robot_commands = RobotCommands()
        self.robot_commands.header.stamp = self.get_clock().now().to_msg()
        self.robot_commands.robot_commands = []
        for i in range(12):
            robot_command = RobotCommands.RobotCommand()
            robot_command.robot_id = i
            robot_command.target_velocity.x = 0
            robot_command.target_velocity.y = 0
            self.robot_commands.robot_commands.append(robot_command)
        self.pub_robot_commands.publish(self.robot_commands)
        self.timer = self.create_timer(0.01, self.on_timer)

    def on_timer(self):
        self.robot_commands.header.stamp = self.get_clock().now().to_msg()
        self.pub_robot_commands.publish(self.robot_commands)


        




class RobotDataCollector:
    def __init__(self, id):
        self.id = id
        self.observed_velocity = []
        self.observed_velocity_stamp = []
        self.command_velocity = []
        self.command_velocity_stamp = []

    def on_world_model(self, msg):
        for robot in msg.robot_info_ours:
            if robot.id == self.id:
                self.observed_velocity.append(robot.velocity)
                self.observed_velocity_stamp.append(time.time())

    def on_robot_commands(self, msg):
        for command in msg.robot_commands:
            if command.robot_id == self.id:
                self.command_velocity.append(command.target_velocity)
                self.command_velocity_stamp.append(time.time())

    def get_synced_data(self):
        observed_velocity = []
        command_velocity = []
        for i in range(len(self.observed_velocity_stamp)):
            for j in range(len(self.command_velocity_stamp)):
                if abs(self.observed_velocity_stamp[i] - self.command_velocity_stamp[j]) < 0.01:
                    observed_velocity.append(self.observed_velocity[i])
                    command_velocity.append(self.command_velocity[j])
        return observed_velocity, command_velocity


class DataSetCollector(Node):
    def __init__(self):
        super().__init__('dataset_collector')
        self.sub_world_model = self.create_subscription(
            WorldModel,
            '/world_model',
            self.on_world_model,
            10)
        self.sub_robot_commands = self.create_subscription(
            RobotCommands,
            '/robot_commands',
            self.on_robot_commands,
            10)
        self.world_model = None
        self.robot_commands = None

        self.robot_data_collectors = []
        for i in range(12):
            self.robot_data_collectors.append(RobotDataCollector(i))

        self.file = open('dataset.txt', 'w')
        self.file.write('world_model,robot_commands\n')
        self.file.close()
        self.file = open('dataset.txt', 'a')

    def on_world_model(self, msg):
        print("on_world_model")
        self.world_model = msg
        for i in range(len(self.robot_data_collectors)):
            self.robot_data_collectors[i].on_world_model(msg)

    def on_robot_commands(self, msg):
        print("on_robot_commands")
        self.robot_commands = msg
        for i in range(len(self.robot_data_collectors)):
            self.robot_data_collectors[i].on_robot_commands(msg)

    def make_pytorch_dataset_for_robot(self, id):
        observed_velocity, command_velocity = self.robot_data_collectors[id].get_synced_data()
        if len(observed_velocity) == 0:
            return None, None
        observed_velocity = np.array(observed_velocity)
        command_velocity = np.array(command_velocity)
        return observed_velocity, command_velocity

    def make_pytorch_dataset(self):
        observed_velocity = []
        command_velocity = []
        for i in range(len(self.robot_data_collectors)):
            observed_velocity_i, command_velocity_i = self.make_pytorch_dataset_for_robot(i)
            if observed_velocity_i is not None:
                observed_velocity.append(observed_velocity_i)
                command_velocity.append(command_velocity_i)
        observed_velocity = np.concatenate(observed_velocity, axis=0)
        command_velocity = np.concatenate(command_velocity, axis=0)
        return observed_velocity, command_velocity

    def save_dataset(self):
        observed_velocity, command_velocity = self.make_pytorch_dataset()
        for i in range(len(observed_velocity)):
            self.file.write(str(observed_velocity[i].x) + ',' + str(observed_velocity[i].y) + ',' + str(
                command_velocity[i].x) + ',' + str(command_velocity[i].y) + '\n')
        self.file.close()
        # print full path of the file
        print(os.path.realpath(self.file.name))


#   class for training with dataset
class DataSetTrainer:
    def __init__(self):
        self.observed_velocity = []
        self.command_velocity = []
        self.load_dataset()

    def load_dataset(self):
        self.file = open('dataset.txt', 'r')
        for line in self.file:
            data = line.split(',')
            # Converts string to float before appending
            self.observed_velocity.append(np.array([float(data[0]), float(data[1])]))
            self.command_velocity.append(np.array([float(data[2]), float(data[3])]))
        self.file.close()
        self.observed_velocity = np.array(self.observed_velocity)
        self.command_velocity = np.array(self.command_velocity)

    def get_batch(self, batch_size):
        indices = np.random.randint(0, len(self.observed_velocity), batch_size)
        return self.observed_velocity[indices], self.command_velocity[indices]

    def get_size(self):
        return len(self.observed_velocity)


import torch
import torch.nn as nn


class RobotMotionPrediction(nn.Module):
    def __init__(self):
        super(RobotMotionPrediction, self).__init__()
        self.fc1 = nn.Linear(2, 5)
        # self.fc2 = nn.Linear(5, 5)
        self.fc2 = nn.Linear(5, 2)

    def forward(self, x):
        x = torch.sigmoid(self.fc1(x))
        # x = torch.sigmoid(self.fc2(x))
        x = self.fc2(x)
        return x


def train():
    dataset_trainer = DataSetTrainer()
    model = RobotMotionPrediction()
    criterion = nn.MSELoss()
    optimizer = torch.optim.Adam(model.parameters(), lr=0.001)
    for epoch in range(100000):
        batch_observed_velocity, batch_command_velocity = dataset_trainer.get_batch(100)
        batch_observed_velocity = torch.from_numpy(batch_observed_velocity).float()
        batch_command_velocity = torch.from_numpy(batch_command_velocity).float()

        optimizer.zero_grad()
        output = model(batch_observed_velocity)
        loss = criterion(output, batch_command_velocity)
        loss.backward()
        optimizer.step()
        if epoch % 100 == 0:
            print(epoch, loss.item())
    torch.save(model.state_dict(), 'model.pth')
    print("Model saved")


def test():
    model = RobotMotionPrediction()
    model.load_state_dict(torch.load('model.pth'))
    model.eval()
    print("Model loaded")
    dataset_trainer = DataSetTrainer()
    batch_observed_velocity, batch_command_velocity = dataset_trainer.get_batch(100)
    batch_observed_velocity = torch.from_numpy(batch_observed_velocity).float()
    batch_command_velocity = torch.from_numpy(batch_command_velocity).float()
    output = model(batch_observed_velocity)
    print(output)
    print(batch_command_velocity)
    print(output.shape)
    print(batch_command_velocity.shape)
    print(torch.mean((output - batch_command_velocity) ** 2))
    print(torch.std((output - batch_command_velocity) ** 2))

    #   plot how well the model works
    import matplotlib.pyplot as plt
    plt.scatter(batch_command_velocity.detach().numpy()[:, 0], output.detach().numpy()[:, 0])
    plt.title("x")
    plt.show()
    plt.scatter(batch_command_velocity.detach().numpy()[:, 1], output.detach().numpy()[:, 1])
    plt.title("y")
    plt.show()
    plt.scatter(output.detach().numpy()[:, 0], output.detach().numpy()[:, 1])
    plt.title("x,y")
    plt.show()
    plt.scatter(batch_command_velocity.detach().numpy()[:, 0], batch_command_velocity.detach().numpy()[:, 1])
    plt.title("x,y")
    plt.show()


class RobotCommandCorrector(Node):
    def __init__(self):
        super().__init__('robot_command_corrector')
        self.sub_robot_commands = self.create_subscription(
            RobotCommands,
            '/robot_commands',
            self.on_robot_commands,
            10)
        self.pub_robot_commands = self.create_publisher(
            RobotCommands,
            '/robot_commands_corrected',
            10)
        self.robot_commands = None
        self.robot_motion_prediction = RobotMotionPrediction()
        self.robot_motion_prediction.load_state_dict(torch.load('model.pth'))
        self.robot_motion_prediction.eval()
        print("Model loaded")

    def on_robot_commands(self, msg):
        # correct robot commands with robot motion prediction and publish
        self.robot_commands = msg
        robot_commands_corrected = RobotCommands()
        robot_commands_corrected.header = msg.header
        robot_commands_corrected.robot_commands = []
        for command in msg.robot_commands:
            command_corrected = command
            command_corrected.target_velocity = self.robot_motion_prediction(torch.from_numpy(
                np.array([command.target_velocity.x, command.target_velocity.y])).float()).detach().numpy()
            robot_commands_corrected.robot_commands.append(command_corrected)
        self.pub_robot_commands.publish(robot_commands_corrected)
        # print(robot_commands_corrected)


def collect(args=None):
    try:
        rclpy.init(args=args)
        dataset_collector = DataSetCollector()
        rclpy.spin(dataset_collector)
    except KeyboardInterrupt:
        print("KeyboardInterrupt")
    finally:
        dataset_collector.save_dataset()
        dataset_collector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # collect()
    train()
    # test()

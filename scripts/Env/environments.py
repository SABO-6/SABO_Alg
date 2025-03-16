import copy
from typing import Optional

import numpy as np
from gosafeopt.experiments.environment import Environment
from gosafeopt.experiments.experiment import Experiment
import torch

import rospy
from std_msgs.msg import Float64MultiArray
import os

class LeggedRobotEnv(Environment):
    def __init__(self, config, LeggedRobotConfig):

        self.c = config
        self.i = 0

        self.env = LeggedRobot(config)

        self.metadata = self.env.metadata

    def _get_obs(self):
        return self.env._get_obs()

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        observation, _ = self.env.reset()
        super().reset()
        self.i = 0
        return observation, {}

    def backup(self, params):
        self.env.backup(params)

    def step(self, k):
        observation, reward, violation, terminated, truncated, info = self.env.step(k)
        self.i += 1

        rewards = np.array([reward, violation])
        return observation, rewards, terminated, truncated, info


class LeggedRobot(Environment):
    def __init__(self, config):
        
        self.config = config

        self.n_rollout = config["n_rollout"]
        self.n = 0
        
        self.backup_params = None
        self.obs = np.array([1, 1])

        self.reward, self.vio = None, None
        self.reward_list, self.vio_list = [], []

        path = '/home/hresvelgr/Downloads/legged/src/leggedcontrol_go2/legged_controllers/scripts/Comparison/Trot/Txt'      # 输入文件夹地址
        files = os.listdir(path)   # 读入文件夹
        self.num_png = len(files)       # 统计文件夹中的文件个数

        rospy.init_node('kp_kd_optimizer', anonymous=False)
        rospy.Subscriber("/controllers/legged_controller/kp_kd_total_diff_topic", Float64MultiArray, self.kp_kd_total_diff_callback)
        self.kp_kd_pub = rospy.Publisher("/controllers/legged_controller/kp_kd_opt", Float64MultiArray, queue_size=10)
        rospy.loginfo("Optimizer initializes.")

        self.new_data_received = False

    def kp_kd_total_diff_callback(self, msg):
        kp, kd, obj, vio = msg.data[0], msg.data[1], msg.data[2], msg.data[3]

        self.reward = -obj
        self.vio = vio
        rospy.loginfo("Optimizer receives kp {%f}, kd {%f} reward {%f} and violation {%f}", kp, kd, self.reward, self.vio)
        self.new_data_received = True

    def _get_obs(self):
        return self.obs

    def backup(self, params):
        self.backup_params = params

    def reset(self, *, seed: Optional[int] = None, options: Optional[dict] = None):
        super().reset()
        self.n = 0
        self.backup_params = None
        return self._get_obs(), {}

    def step(self, k):
        
        msg = Float64MultiArray()
        msg.data = [k[0], k[1]]
        self.kp_kd_pub.publish(msg)
        rospy.loginfo("Optimizer sends kp {%f} and kd {%f} to the controller.", k[0], k[1])

        self.new_data_received = False
        while not self.new_data_received:
            self.kp_kd_pub.publish(msg)
            rospy.sleep(1)

        self.n += 1

        self.obs = np.array([k[0], k[1]])
        done = True
        truncated = True
        info = {}

        rospy.loginfo("Optimizer returns reward {%f} and violation {%f}", self.reward, self.vio)

        # self.vio = max(self.vio - self.config["threshold"], 0)
        self.vio = self.vio

        result = [[k[0], k[1], -self.reward, self.vio]]

        with open(f'/home/hresvelgr/Downloads/legged/src/leggedcontrol_go2/legged_controllers/scripts/Comparison/Trot/Txt/GoSafeOPT_Standing_Trot_Index_{self.num_png + 1}_Upper_{self.config["domain_end"][0]}_Lengthscale_{self.config["model"]["lenghtscale"][0]}_Threshold_{self.config["threshold"]}_Opt_solver_{self.config["Opt"]}', 'a') as f:
            for row in result:
                f.write("\t".join(map(str, row)) + "\n")

        return self.obs, self.reward, self.vio, done, truncated, info
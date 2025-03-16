#!/usr/bin python3

import json
import random
from pathlib import Path
import sys
sys.path.append('/home/hresvelgr/Downloads/legged/src/leggedcontrol_go2/legged_controllers/scripts')
sys.path.append('/home/hresvelgr/Downloads/legged/src/leggedcontrol_go2/legged_controllers/scripts/Env')


import numpy as np
import torch
import typer
# import wandb
from gosafeopt.aquisitions.go_safe_opt import GoSafeOpt
from gosafeopt.aquisitions.ucb import UCB
from gosafeopt.experiments.backup import GoSafeOptBackup
from gosafeopt.experiments.experiment import Experiment
from gosafeopt.models.model import ModelGenerator
from gosafeopt.optim.swarm_opt import SwarmOpt
from gosafeopt.optim.grid_opt import GridOpt
from gosafeopt.tools.data import Data
# from gosafeopt.tools.data_logger import WandbLogger
from gosafeopt.tools.logger import Logger
from gosafeopt.trainer import Trainer
from environments import LeggedRobotEnv
from torch import Tensor
import rospy



def get_configs(
    # config_path: str = f"{Path().absolute()}/examples/config.json",
    config_path: str = "/home/hresvelgr/Downloads/legged/src/leggedcontrol_go2/legged_controllers/scripts/Env/config.json",
    config_path_fx: str = "/home/hresvelgr/Downloads/legged/src/leggedcontrol_go2/legged_controllers/scripts/Env/config_robot.json",
    # config_path_fx: str = f"{Path().absolute()}/examples/config_robot.json",
):
    with open(config_path) as f:
        config = json.load(f)
    with open(config_path_fx) as f:
        config_fx = json.load(f)

    return config, config_fx

def train():
    config, config_fx = get_configs()
    config.update(config_fx)
    Logger.set_verbosity(4)

    torch.manual_seed(config["seed"])
    np.random.seed(config["seed"])
    random.seed(config["seed"])

    data = Data()

    safeopt = True
    context = torch.tensor([1.0])
    x_safe = torch.tensor([[1, 1, context]])

    lower_bound = 0.5
    upper_bound_list = [9]
    threshold_list = [0.3]
    ls_list = [0.1]
    Opt_list = ["Swarm"]

    for i in range(1, 11):
        for upper in upper_bound_list:
            config["domain_end"] = [upper, upper, 5]
            config["domain_start"] = [lower_bound, lower_bound, 1]
            for threshold in threshold_list:
                config["threshold"] = threshold
                for length_scale in ls_list:
                    config["model"]["lenghtscale"] = [length_scale, length_scale, 1.0]
                    for Opt in Opt_list:
                        config["Opt"] = Opt
                        if safeopt:
                            backup = GoSafeOptBackup(data=data, **config["GoSafeOptBackupStrategy"])
                            aquisition = GoSafeOpt(**config["GoSafeOpt"], data=data, context=context, dim_obs=config["dim_obs"])
                        else:
                            backup = None
                            aquisition = UCB(scale_beta=1.0, beta=1.0, context=context, dim_obs=config["dim_obs"])

                        environment = LeggedRobotEnv(config, config_fx)
                        experiment = Experiment(config, environment, data=data, backup=backup)

                        trainer = Trainer(
                            **config["Trainer"], dim_obs=config["dim_obs"], dim_params=config["dim_params"], data=data)

                        model = ModelGenerator(
                            **config["model"],
                            domain_start=Tensor(config["domain_start"]),
                            domain_end=Tensor(config["domain_end"]),
                            dim_obs=config["dim_obs"],
                            dim_model=config["dim_model"],
                        )
                        if config["Opt"] == "Swarm":
                            optimizer = SwarmOpt(
                                aquisition,
                                **config["Optimization"],
                                context=context,
                                domain_start=Tensor(config["domain_start"]),
                                domain_end=Tensor(config["domain_end"]),
                                dim_params=config["dim_params"],
                                dim_context=config["dim_context"],
                                data=data,
                            )
                        else:
                            optimizer = GridOpt(
                                aquisition,
                                **config["Optimization"],
                                context=context,
                                domain_start=Tensor(config["domain_start"]),
                                domain_end=Tensor(config["domain_end"]),
                                dim_params=config["dim_params"],
                                dim_context=config["dim_context"],
                                data=data,
                            )

                        trainer.train(experiment, model, optimizer, aquisition, x_safe)

    # print('Best k:', trainer.best_k, 'Reward max:', trainer.reward_max)

if __name__ == "__main__":
    train()

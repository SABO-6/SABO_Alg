from abc import ABC, abstractmethod
from typing import Optional, Tuple

import gpytorch
import torch
from botorch.acquisition.proximal import ModelListGP
from botorch.models.pairwise_gp import GPyTorchPosterior
from torch import Tensor

import gosafeopt
from gosafeopt.tools.data import Data


class BaseAquisition(ABC):
    def __init__(
        self,
        dim_obs: int,
        scale_beta: float,
        beta: float,
        context: Optional[Tensor] = None,
        data: Optional[Data] = None,
        n_steps: int = 1,
    ):
        self.dim_obs = dim_obs
        self.scale_beta = scale_beta
        self.beta = beta
        self.context = context
        self.data = data
        self.steps = n_steps
        self.model: None | ModelListGP = None
        self.fmin = torch.zeros(self.dim_obs).to(gosafeopt.device)

    def update_model(self, model: ModelListGP):
        self.model = model

    def model_posterior(self, x: Tensor) -> GPyTorchPosterior:
        if self.model is not None:
            self.model.eval()
            with torch.no_grad(), gpytorch.settings.fast_pred_var(), gpytorch.settings.fast_pred_samples():
                posterior = self.model.posterior(x)
                return posterior  # type: ignore
        else:
            raise Exception("Model is not initialized")

    @abstractmethod
    def evaluate(self, x: Tensor, step: int = 0) -> Tensor:
        pass

    def after_optimization(self) -> None:  # noqa: B027
        pass

    def before_optimization(self) -> None:  # noqa: B027
        pass

    # def override_set_initialization(self) -> bool | str:
    def override_set_initialization(self):
        """With this method an aquisition function can override the set initialization."""
        return False

    def is_internal_step(self, step: int = 0):
        """Return if the result of the aquisition step should be appended to the possible maximizers."""

        if step == 0:
            return True
        else:
            raise NotImplementedError

    def get_confidence_interval(self, posterior: GPyTorchPosterior) -> Tuple[Tensor, Tensor]:
        mean = posterior.mean.reshape(-1, self.dim_obs)
        var = posterior.variance.reshape(-1, self.dim_obs)

        # Upper and lower confidence bound
        l = mean - self.scale_beta * torch.sqrt(self.beta * var)  # noqa: E741
        u = mean + self.scale_beta * torch.sqrt(self.beta * var)

        return l, u

    def safe_set(self, x: Tensor) -> Tensor:
        posterior = self.model_posterior(x)

        l, _ = self.get_confidence_interval(posterior)  # noqa: E741

        safe_set = torch.all(l[:, 1:] > self.fmin[1:], axis=1)  # type: ignore
        return safe_set

    def has_safe_points(self, x: Tensor) -> Tensor:
        return torch.any(self.safe_set(x))

    def soft_penalty(self, slack: Tensor) -> Tensor:
        penalties = torch.clip(slack, None, 0)

        penalties[(slack < 0) & (slack > -0.001)] *= 2
        penalties[(slack <= -0.001) & (slack > -0.1)] *= 5
        penalties[(slack <= -0.1) & (slack > -1)] *= 10

        slack_id = slack < -1
        penalties[slack_id] = -300 * penalties[slack_id] ** 2
        return torch.sum(penalties[:, 1:], axis=1)  # type: ignore

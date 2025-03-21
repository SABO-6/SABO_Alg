from typing import Optional

import torch
from botorch.models.pairwise_gp import GPyTorchPosterior
from torch import Tensor
from torch.distributions.multivariate_normal import MultivariateNormal

import gosafeopt
from gosafeopt.aquisitions.base_aquisition import BaseAquisition


class SafeOpt(BaseAquisition):
    best_lcb = -1e10

    def __init__(
        self,
        dim_obs: int,
        scale_beta: float,
        beta: float,
        context: Optional[Tensor] = None,
    ):
        super().__init__(dim_obs, scale_beta, beta, context=context, n_steps=3)

    def evaluate(self, x: Tensor, step: int = 0) -> Tensor:
        posterior = self.model_posterior(x)
        # match step:
        #     case 0:
        #         return self.lower_bound(posterior)  # type: ignore
        #     case 1:
        #         return self.maximizers(posterior)  # type: ignore
        #     case 2:
        #         return self.expanders(posterior)  # type: ignore
        #     case _:
        #         raise NotImplementedError
        if step == 0:
            return self.lower_bound(posterior)  # type: ignore
        elif step == 1:
            return self.maximizers(posterior)  # type: ignore
        elif step == 2:
            return self.expanders(posterior)
        else:
            raise NotImplementedError

    # def override_set_initialization(self) -> bool | str:
    def override_set_initialization(self):
        # TODO: somehow override initialization for global lower_bound
        return super().override_set_initialization()

    def is_internal_step(self, step: int = 0):
        return step == 0

    def lower_bound(self, x: GPyTorchPosterior) -> Tensor:
        l, _ = self.get_confidence_interval(x)  # noqa: E741

        max_lcb = torch.max(l[:, 0])
        if max_lcb > SafeOpt.best_lcb:
            SafeOpt.best_lcb = max_lcb

        slack = l - self.fmin

        return l[:, 0] + self.soft_penalty(slack)

    def maximizers(self, x: GPyTorchPosterior) -> Tensor:
        l, u = self.get_confidence_interval(x)  # noqa: E741
        scale = 1  # if not self.c["normalize_output"] else self.model.models[0].outcome_transform._stdvs_sq[0]
        values = (u - l)[:, 0] / scale
        improvement = u[:, 0] - SafeOpt.best_lcb

        interest_function = torch.sigmoid(100 * improvement / scale)
        interest_function -= interest_function.min()
        c = interest_function.max() - interest_function.min()
        c[c < 1e-5] = 1e-5
        interest_function /= c

        slack = l - self.fmin
        penalties = self.soft_penalty(slack)

        value = (values + penalties) * interest_function

        return value

    def expanders(self, x: GPyTorchPosterior) -> Tensor:
        l, u = self.get_confidence_interval(x)  # noqa: E741

        scale = 1  # if not self.c["normalize_output"] else self.model.models[0].outcome_transform._stdvs_sq[0]
        values = (u - l)[:, 0] / scale

        slack = l - self.fmin
        penalties = self.soft_penalty(slack)

        # TODO: how to set scale?
        normal = MultivariateNormal(
            loc=torch.zeros_like(slack[:, 1:], device=gosafeopt.device),
            covariance_matrix=torch.eye(slack.shape[1] - 1, device=gosafeopt.device),
        )
        interest_function = normal.log_prob(slack[:, 1:])
        interest_function -= interest_function.min()
        c = interest_function.max() - interest_function.min()
        c[c < 1e-5] = 1e-5
        interest_function /= c

        value = (values + penalties) * interest_function

        return value

    def reset(self):
        self.best_lcb = -1e10

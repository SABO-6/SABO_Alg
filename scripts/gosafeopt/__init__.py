import torch

torch.set_default_dtype(torch.float64)

# device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")
device = torch.device("cpu")

import torch
import torch.nn as nn
from torch.nn import functional as F



class Head(nn.Module):
    """ One head of self-attention """

    def __init__(self, head_size, n_embed, block_size, dropout):
        super().__init__()
        self.key = nn.Linear(n_embed, head_size, bias=False)
        self.query = nn.Linear(n_embed, head_size, bias=False)
        self.value = nn.Linear(n_embed, head_size, bias=False)
        self.register_buffer('tril', torch.tril(torch.ones(block_size, block_size)))
        self.dropout = nn.Dropout(dropout)

    
    def forward(self, x):
        B, T, C = x.shape
        k = self.key(x)  # (B, T, C)
        q = self.key(x)  # (B, T, C)
        # compute attention scores (affinities)
        wei = q @ k.transpose(-2, -1) * C**-0.5  # (B, T, 16) @ (B, 16, T) --> (B, T, T) 
        # make sure future doesn't communicate with past
        wei = wei.masked_fill(self.tril[:T, :T] == 0, float('-inf'))  
        wei = F.softmax(wei, dim=-1)  # (B,T,C)
        wei = self.dropout(wei)
        # perform the weighted aggregation of the values
        v = self.value(x)  # (B, T, C)
        out = wei @ v  # (B, T, T) @ (B, T, C) -> (B, T, C)
        return out
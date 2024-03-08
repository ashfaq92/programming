import torch.nn as nn

from MultiHeadAttention import MultiHeadAttention
from FeedForward import FeedForward
          
class Block(nn.Module):
    """ Transformer block: communication followed by computation """
    def __init__(self, n_embed, n_head, dropout, block_size):
        # n_embed: embedding dimension
        # n_head: the number of heads we'd like
        super().__init__()
        head_size = n_embed // n_head
        self.sa = MultiHeadAttention(n_head, head_size, n_embed, dropout, block_size)
        self.ffwd = FeedForward(n_embed, dropout)
        self.ln1 = nn.LayerNorm(n_embed)
        self.ln2 = nn.LayerNorm(n_embed)

    def forward(self, x):
        x = x + self.sa(self.ln1(x))
        x = x + self.ffwd(self.ln2(x))
        return x
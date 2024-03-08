import torch
import torch.nn as nn
from Block import Block 
from torch.nn import functional as F



# super simple bigram model
class BigramLanguageModel(nn.Module):
    def __init__(self, vocab_size, n_embed, block_size, n_layer, n_head, device, dropout):
        self.device = device
        self.block_size = block_size

        super().__init__()
        self.token_embedding_table = nn.Embedding(vocab_size, n_embed)
        self.position_embedding_table = nn.Embedding(block_size, n_embed)
        # self.blocks = nn.Sequential(
        #     Block(n_embed, n_head=4),
        #     Block(n_embed, n_head=4),
        #     Block(n_embed, n_head=4),
        #     nn.LayerNorm(n_embed),
        # )
        self.blocks = nn.Sequential(*[Block(n_embed=n_embed, n_head=n_head, dropout=dropout, block_size=block_size) for _ in range(n_layer)])
        self.ln_f = nn.LayerNorm(n_embed)  # final layer norm
        # self.sa_heads = MultiHeadAttention(4, n_embed//4)  # i.e., 4 heads of 8-d self-attention
        # self.ffwd = FeedForward(n_embed)
        self.lm_head = nn.Linear(n_embed, vocab_size)
    
    def forward(self, idx, targets=None):
        B, T = idx.shape
        # idx and targets are both (B, T) tensor of targets
        token_embedding = self.token_embedding_table(idx)  # (B, T, C) B=batch_size, T = block_size, C = Embedding size / channel / characters
        position_embedding = self.position_embedding_table(torch.arange(T, device=self.device))  # (T, C)
        x = token_embedding + position_embedding  # (B, T, C)
        # x = self.sa_heads(x)
        # x = self.ffwd(x)  # (B, T, C)
        x = self.blocks(x)
        logits = self.lm_head(x)  # (B, T, vocab_size)

        # implement loss fuction
        if targets is None:
            loss = None
        else:
            B, T, C = logits.shape
            logits = logits.view(B*T, C)
            targets = targets.view(B*T)
            loss = F.cross_entropy(logits, targets)
        
        return logits, loss 
    
    def generate(self, idx, max_new_tokens):
        # idx is (B, T) array of indices in the current context
        for _ in range(max_new_tokens):
            # crop idx to the last block_size tokens
            idx_cond = idx[:, -self.block_size:]
            # get the predictions
            logits, loss = self(idx_cond)  # currently loss is not being used
            # focus only the last time step
            logits = logits[:, -1, :]  # becomes (B, C)
            # apply softmax to get probabilities
            probs = F.softmax(logits, dim=-1)  # (B, C)
            # sample from the distribution
            idx_next = torch.multinomial(probs, num_samples=1)  # (B, 1)
            # append sampled index to the running sequence
            idx = torch.cat((idx, idx_next), dim=1)  # (B, T+1)
        return idx
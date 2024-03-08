import torch
from BigramLanugageModel import BigramLanguageModel 


# hyperparameters
torch.manual_seed(1337)
batch_size = 64  # number of embeding dimensions
block_size = 256  # the max context length for predictions
max_iters = 5000
eval_interval = 500
learning_rate = 3e-3  # minimum the better
device = 'cuda' if torch.cuda.is_available() else 'cpu'
eval_iters = 200
n_embed = 384  
n_head = 6
n_layer = 6
dropout = 0.2


# open text file 
with open('input.txt', 'r', encoding='utf-8') as f:
    text = f.read()


# characters that model can see or emit
chars = sorted(list(set(text)))  
vocab_size = len(chars)


# create a mapping from characters to integers
# convert raw text/string into some sequence of integers according to some vocabulary
stoi = { ch:i for i,ch in enumerate(chars) }  # assign integer value to each character
itos = { i:ch for i,ch in enumerate(chars) }
encode = lambda s: [stoi[c] for c in s]  # encoder: take a string, output a list of integers
decode = lambda l: ''.join([itos[i] for i in l])  # decoder: take a list of integers, output a string
# point: the codebook size and the sequence size are inversely proportional 


# train and test splits
data = torch.tensor(encode(text), dtype=torch.long)  # a long vector of all the encoded text
n = int(0.9 * len(data))
train_data = data[:n]
val_data = data[n:]

# data loading
def get_batch(split):
    # generate a small batch of data of inputs x and targets y
    data = train_data if split=='train' else val_data
    # generate 'batch_size' number of random numbers representing indexes in the long torch array. 
    # of course, the random numbers should be 'block_size' less than the total array to accommodate the last block
    ix = torch.randint(len(data)-block_size, (batch_size,))
    # now generate 'batch_size' number of sub-arrays in the data array having 'block_size' length
    x = torch.stack([data[i:i+block_size] for i in ix])
    # next token for each sequence in the 'x'
    y = torch.stack([data[i+1:i+block_size+1] for i in ix])
    # make sure to move the loaded data to the device
    x, y = x.to(device), y.to(device)
    return x, y

# tell pytorch that don't call back propagation
@torch.no_grad
def estimate_loss():
    out = {}
    model.eval()
    for split in ['train', 'val']:
        losses = torch.zeros(eval_iters)
        for k in range(eval_iters):
            X, Y = get_batch(split)
            logits, loss = model(X, Y)
            losses[k] = loss.item()
        out[split] = losses.mean()
    model.train()
    return out


model = BigramLanguageModel(vocab_size, n_embed, block_size, n_layer, n_head, device, dropout)
m = model.to(device)  # also move model weights to the device
optimizer = torch.optim.AdamW(m.parameters(), lr=learning_rate)  # lr = learning rate

for iter in range(max_iters):
    # evaluate the loss on train and val sets
    if iter % eval_interval == 0:
        losses = estimate_loss()
        print(f"step {iter}: train loss {losses['train']:.4f}, val loss {losses['val']:.4f}")

    
    # sample a batch of data
    xb, yb = get_batch('train')

    # evaluate the loss
    logits, loss = m(xb, yb)
    # zero out the gradientes from the previous step
    optimizer.zero_grad(set_to_none=True)
    #  get the gradients for all the parameters
    loss.backward()
    #  use those gradients to update those parameters
    optimizer.step()


# generate from model
context = torch.zeros((1, 1), dtype=torch.long, device=device)
next_tokens = m.generate(context, max_new_tokens=500)[0]  
decoded_tokens = decode(next_tokens.tolist())  # because our decode function takes python list
print(decoded_tokens)
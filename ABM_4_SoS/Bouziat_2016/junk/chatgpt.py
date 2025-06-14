# Apply fix: guard removal
import random
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

class Box:
    def __init__(self, color, pos):
        self.color = color
        self.pos = pos

class Nest:
    def __init__(self, color, pos):
        self.color = color
        self.pos = pos

class Robot:
    def __init__(self, color, pos, energy, max_energy, coop_mode):
        self.color = color
        self.pos = pos
        self.energy = energy
        self.max_energy = max_energy
        self.coop_mode = coop_mode
        self.carrying = None

    def step(self, boxes, nests, robots):
        self.energy -= 1
        if self.energy <= 0:
            return 'dead'
        visible = [b for b in boxes if abs(b.pos[0]-self.pos[0])<=3 and abs(b.pos[1]-self.pos[1])<=3]
        if self.coop_mode == 'sos':
            for color in ['red','blue','green']:
                group = [r for r in robots if r.color==color and r.energy>0]
                if group:
                    low = sum(1 for r in group if r.energy<100)
                    if low > len(group)/2:
                        self.reward_func = lambda bc: 75
                        break
        if self.carrying:
            nest = next(n for n in nests if n.color==self.carrying.color)
            self.move_towards(nest.pos)
            if self.pos == nest.pos:
                reward = self.reward_func(self.carrying.color)
                self.energy = min(self.max_energy, self.energy+reward)
                if self.carrying in boxes:
                    boxes.remove(self.carrying)
                self.carrying = None
        else:
            best = None; best_score=-1e9
            for b in visible:
                dist = abs(b.pos[0]-self.pos[0])+abs(b.pos[1]-self.pos[1])
                nest = next(n for n in nests if n.color==b.color)
                dist += abs(nest.pos[0]-b.pos[0])+abs(nest.pos[1]-b.pos[1])
                reward = self.reward_func(b.color)
                score = reward - dist
                if score>best_score:
                    best_score=score; best=b
            if best:
                if self.coop_mode=='agent' and any(r.carrying==best for r in robots):
                    holder = next(r for r in robots if r.carrying==best)
                    if self.energy < holder.energy:
                        holder.carrying=None
                        self.carrying=best
                else:
                    self.carrying=best
            else:
                self.random_move()
        return 'alive'

    def random_move(self):
        moves = [(1,0),(-1,0),(0,1),(0,-1)]
        dx,dy = random.choice(moves)
        self.pos = (max(0,min(49,self.pos[0]+dx)), max(0,min(49,self.pos[1]+dy)))

    def move_towards(self, target):
        x,y = self.pos; tx,ty = target
        self.pos = (x + np.sign(tx-x), y + np.sign(ty-y))

    def reward_func(self, box_color):
        return 100 if box_color==self.color else 50

def run_sim(coop_mode):
    nests = [Nest(c,pos) for c,pos in zip(['red','blue','green'],[(0,0),(49,0),(25,49)])]
    robots = [Robot(random.choice(['red','blue','green']), (random.randrange(50),random.randrange(50)), 300,300,coop_mode) for _ in range(90)]
    boxes=[]
    stats={'mean_energy':[],'num_boxes':[],'alive_robots':[]}
    for t in range(200):
        if t%3==0:
            boxes.append(Box(random.choice(['red','blue','green']),(random.randrange(50),random.randrange(50))))
        new_robots=[]
        for r in robots:
            status = r.step(boxes, nests, robots)
            if status=='alive':
                new_robots.append(r)
        robots = new_robots
        energies=[r.energy for r in robots]
        stats['mean_energy'].append(np.mean(energies) if energies else 0)
        stats['num_boxes'].append(len(boxes))
        stats['alive_robots'].append(len(robots))
    return pd.DataFrame(stats)

results = {mode: run_sim(mode) for mode in ['none','agent','sos']}

for metric in results['none'].columns:
    plt.figure()
    for mode, df in results.items():
        plt.plot(df[metric], label=mode)
    plt.title(metric)
    plt.xlabel('Time')
    plt.ylabel(metric)
    plt.legend()
    plt.show()

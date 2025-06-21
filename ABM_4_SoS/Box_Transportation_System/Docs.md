# CoCaRo Multi-Agent System Simulation

**CoCaRo** is an AMAS (Adaptive Multi-Agent System) used to model and simulate a carrying system of colored boxes by robots, providing a foundation for advanced System of Systems research.

This project presents the results of a CoCaRo (Colored Boxes and Robots) multi-agent system simulation, demonstrating emergent behaviors in resource collection and energy management. The simulation successfully reproduces theoretical predictions of adaptive multi-agent systems with finite resources.

## System Specifications
1. A robot has to find and catch a box and deposit it in the nest having the *same* color as the box.  
2. Each robot has an initial amount of energy that it consumes at each movement.  
3. When a robot deposits a box, it receives a reward in the form of energy allowing it to remain alive longer.  
4. The value of the reward depends on the color of the deposited box compared to the color of the robot:  
   - A robot will get a better reward if it deposits a box of its *own color*.


### Environment

1. **Displacement Grid**  
    - A 50x50 grid of square cells.  
    - A cell may be empty or may contain: A robot `OR` A box `OR` A nest  
    - The grid contains three *nests*: blue, red, and green. These nests are equidistant from each other to prevent proximity bias in simulations.

2. **Nest**: An object in which robots deposit boxes.

3. **Boxes**  
    - May be red, blue, or green.  
    - Appear randomly on the grid at regular time intervals.


### Robot
The robot agent chooses the most appropriate action based on its current *state* and *perceptions*.

#### Actions
- `Move`: The robot moves on the grid according to a Monte Carlo distribution until it finds a box.
- `Deposit`: The robot deposits the carried box into the nest of matching color.
- `Pickup`: Take/pick up a box from current position and find nest that matches the color for this box
- `Go`: Go towards a specific target (box or nest)

#### States

- `carried`: The agent is carrying a box.  
- `target`: The agent has targeted a box.  
- `onPosBox`: The agent is on the same cell as its target box.  
- `onPosNest`: The agent is on the same cell as the nest matching its carried box.

#### Perceptions & Behaviors
- `if perceives target               → go(targeted_box)`
- `if perceives target ∧ onPosBox   → take(targeted_box)`
- `if perceives carried ∧ onPosNest → deposit(carried_box)`
- `if perceives ¬target ∧ ¬carried  → move`

### Criticality
- Difficulty of a robot agent and is defined in terms of its energy level
- Effectiveness of robot: energy level (may rapidly deteriorate)
- If
   - $r_i$: robot agent
   - $C_{ri}$: Criticality of a robot agent (temporal function)
   - $t$: A specific time (simulation tick?)
   - $Ne_{ri}(t)$: battery level of agent $r_i$ at $t$ 
   - $Max_{Ne}$: Max battery level
   - then: $C_{r_i}(t) = Max_{Ne} - Ne_{r_i}(t)$

### Anticipated Criticality
- Function that allows the robot agent to know the criticality it is going to get once the box $b_k$ is deposited in the nest. 
- It enables the robot agent to choose the box that will offer it most energy. 
- If
   - $CA_{r_i}$: Anticipated criticality
   - $b_k$: box
   - $t$: time
   - $Max_{Ne}$: Max battery level
   - $Ne_a(b_k, t)$: Energy got by depositing the box $b_k$ at $t$
   - then: $CA_{r_i}(b_k, t) = Max_{Ne} - Ne_a(b_k, t)$


## Simulation

**Simulation Framework**: Custom Python implementation  
**Analysis Tools**: Matplotlib for visualization, statistical analysis across multiple runs  
**Code Repository**: `ABM_4_SoS/Box_Transportation_System/` 

### Configuration

#### Environment
- Grid Size: 50 × 50 cells (2,500 total cells)
- Simulation Duration: 10,000 steps
- Box Generation Rate: 1 box every 3 time units
- Total Simulation Runs: 50 independent runs with different random seeds

#### Agent
- Total Robots: 90 (30 per color: RED, GREEN, BLUE)
- Initial Energy: 300 units per robot
- Maximum Energy: 300 units
- Perception Radius: 3 cells
- Movement Cost: 1 energy unit per step

#### Reward Structure
- Base Reward: +2 energy for any box deposit
- Color Bonus: +1 additional energy for same-color deposits
- Net Same-Color Gain: +3 energy per successful deposit

### Behavior Described in the Paper

1. The mean battery level of the robots rapidly decreases around the 500th to 1000th simulation tick but never reaches zero. Up to the 10,000th tick, the mean battery level remains slightly above 50.

2. The number of boxes in the environment gradually increases over time.

3. The number of active (alive) robots declines following a similar trend to the mean battery level, suggesting some correlation between the two. However, the number of active robots never reaches zero; approximately 5 to 10 robots remain in the environment up to the 10,000th simulation cycle.

4. Even at the 0th simulation cycle, there are about 10 to 20 boxes already present in the environment.



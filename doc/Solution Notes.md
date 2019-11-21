# Solution Notes

## Information Clarification

**State ID**

| State    | ID   |
| -------- | ---- |
| Free     | 0    |
| Tree     | 1    |
| Shooter  | 2    |
| Pick Up  | 3    |
| Drop Off | 4    |
| Base     | 5    |

**Action ID**

| Action | ID   |
| ------ | ---- |
| North  | 1    |
| South  | 2    |
| East   | 3    |
| West   | 4    |
| Hover  | 5    |

## State Transition Matrix

The key point of this programming exercise is finding the correct state transition matrix and then all the other parts follow easily. We focus on deriving the state transition matrix in this section.

First, we know that the number of valid states should be $K = 2 \times (M \times N - N_{tree})$. Here, $M \times N$ denotes all cells contained in the map and we subtract $N_{tree}$ to get the number of accessible cells. At each accessible cell, there are two possible states, i.e., with or without the package.

Second, consider only the action we take. The probability of ending in corresponding accessible cell should be $1$ and $0$ elsewhere.

Next, take the wind into account. Denote the number of adjacent trees by $T (0 \leq T \leq 4)$ ($T$ can be $4$ only when you are unluckily spawned in the middle of trees, in while case you will never complete the mission). The probability of staying at the current position becomes $1 - P_{wind}$ and you may end up in the base with a probability of $\frac{T}{4} \times P_{wind}$ because you can crash into the trees. If you are lucky that the drone does not crash, you have a probability of $\frac{4-T}{4} \times P_{wind}$ to arrive in an adjacent cell.

Finally, examine how many angry residents exist within $R$ cells. The probability of not being shot and thus staying there is $\Pi(1-P_{r_i})$.

## Robustness Discussion

- 


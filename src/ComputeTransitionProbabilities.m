function P = ComputeTransitionProbabilities(stateSpace, map)
% ComputeTransitionProbabilities Compute the transition probabilities
% between all states in the state space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, map) 
%   computes the transition probabilities between all states in the state 
%   space for all control inputs.
%
%   Input arguments:
%       stateSpace:
%           A (K x 3) - matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N) - matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       P:
%           A (K x K x L) - matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

%% declare global variables
global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX

%% step 1: undisturbed movement
% initialize transition matrix
P = zeros(K, K, 5);

% find naive transition matrix
for i = 1:K
   for j = 1:K
       currPos = stateSpace(i,:); % current state
       nextPos = stateSpace(j,:); % next state
       horzMove = nextPos(1) - currPos(1); % movement along m direction
       vertMove = nextPos(2) - currPos(2); % movement along n direction
       packMove = nextPos(3) - currPos(3); % change in package state
       if horzMove==1 && vertMove==0 % move east
           P(i,j,EAST) = 1;
       elseif horzMove==-1 && vertMove==0 % move west
           P(i,j,WEST) = 1;
       elseif horzMove==0 && vertMove==1 % move north
           P(i,j,NORTH) = 1;
       elseif horzMove==0 && vertMove==-1 % move south
           P(i,j,SOUTH) = 1;
       elseif horzMove==0 && vertMove==0 && packMove==0 % stay
           P(i,j,HOVER) = 1;
       end
   end
end

%% step 2: wind disturbance

%% step 3: angry resident gun shot

%% step 4: pick up package

end

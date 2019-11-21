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
P = zeros(K, K, 5); % final transition matrix
P1 = zeros(K, K, 5); % transition matrix in step 1

% find naive transition matrix
for i = 1:K % time complexity: O(n2)
   for j = 1:K
       currPos = stateSpace(i,:); % current state
       nextPos = stateSpace(j,:); % next state
       horzMove = nextPos(1) - currPos(1); % movement along m direction
       vertMove = nextPos(2) - currPos(2); % movement along n direction
       packMove = nextPos(3) - currPos(3); % change in package state
       if packMove~=0 % should not pick package yet
           continue;
       elseif horzMove== 1 && vertMove== 0 % east movement detected
           P1(i,j,EAST) = 1;
       elseif horzMove==-1 && vertMove== 0 % west movement detected
           P1(i,j,WEST) = 1;
       elseif horzMove== 0 && vertMove== 1 % north movement detected
           P1(i,j,NORTH) = 1;
       elseif horzMove== 0 && vertMove==-1 % south movement detected
           P1(i,j,SOUTH) = 1;
       elseif horzMove== 0 && vertMove== 0 % hover movement detected
           P1(i,j,HOVER) = 1;
       end
   end
end

% clear used variables
clear currPos;
clear nextPos;
clear horzMove;
clear vertMove;
clear packMove;

%% step 2: wind disturbance
% compute some constants
M = size(map, 1); % map size in m direction
N = size(map, 2); % map size in n direction
P_PEACE = 1 - P_WIND; % probability of a peaceful day

% find base state index
[baseM,baseN] = find(map==BASE);
if isempty(baseM)
    error('Error: Invalid Map (No Base)');
elseif (size(baseM,1)>1)
    error('Error: Invalid Map (Multiple Bases)');
end
[~, baseIndex] = ismember([baseM,baseN,0], stateSpace, 'row');
if ~baseIndex
    error('Error: State Space Constructed Wrongly')
end

%% step 3: angry resident gun shot

%% step 4: pick up package

end

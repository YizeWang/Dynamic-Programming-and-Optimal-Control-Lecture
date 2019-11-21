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
global GAMMA R P_WIND P_PEACE 
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX

%% step 1: undisturbed movement
P = zeros(K, K, 5); % final transition matrix
P1 = zeros(K, K, 5); % transition matrix in step 1
P2 = zeros(K, K, 5); % transition matrix in step 2

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

% compute transition matrix after wind
for i = 1:K % iterate starting state
    for j = 1:K % iteratre ending state
        for k = [NORTH, SOUTH, EAST, WEST, HOVER] % iterate actions
            if P1(i,j,k)==1 % when naive movement valid
                P2(i,j,k) = P2(i,j,k) + P1(i,j,k) * (1 - P_WIND); % staying probability
                for windAction = [NORTH, SOUTH, EAST, WEST] % iterate wind actions
                    nextStateIndex = FindIndex(stateSpace, map, baseIndex, j, windAction); % find next state resulting from wind
                    P2(i,nextStateIndex,k) = P2(i,nextStateIndex,k) + P_WIND/4; % update transition matrix
                end
            else
                continue;
            end
        end
    end
end

%% step 3: angry resident gun shot

%% step 4: pick up package

end

function nextStateIndex = FindIndex(stateSpace, map, baseIndex, currState, action)
% FindIndex Compute the index in stateSpace of next state if action applied
%
%   nextStateIndex = FindIndex(stateSpace, map, action)
%   Compute the index in stateSpace of next state if action applied
%   Return the base state index if next state would be tree or outside map
%
%   Input arguments:
%       stateSpace:
%           A (K x 3) - matrix, where the i-th row represents the i-th
%           element of the state space.
%       map:
%           A (M x N) - matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%       baseIndex:
%           A scalar representing the index in stateSpace of base state
%       currState:
%           A scalar representing the index in stateSpace of current state
%       action:
%           A scalar representing the action
%
%   Output arguments:
%       nextStateIndex:
%           A scalar representing the index in stateSpace of next state 
%           if action applied. Return the base state index if next state
%           would be tree or outside map

%% declare global variables
global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX

%% parse information
currM = stateSpace(currState, 1); % current state in m
currN = stateSpace(currState, 2); % current state in n
currPsi = stateSpace(currState, 3); % current state in psi

%% compute next state
if action==NORTH && ismember([currM,currN+1,currPsi],stateSpace,'row')
    [~, nextStateIndex] = ismember([currM,currN+1,currPsi],stateSpace,'row');
elseif action==SOUTH && ismember([currM,currN-1,currPsi],stateSpace,'row')
    [~, nextStateIndex] = ismember([currM,currN-1,currPsi],stateSpace,'row');
elseif action==EAST && ismember([currM+1,currN,currPsi],stateSpace,'row')
    [~, nextStateIndex] = ismember([currM+1,currN,currPsi],stateSpace,'row');
elseif action==WEST && ismember([currM-1,currN,currPsi],stateSpace,'row')
    [~, nextStateIndex] = ismember([currM-1,currN,currPsi],stateSpace,'row');
else
    nextStateIndex = baseIndex; % if crash into next state, return to base
end

end
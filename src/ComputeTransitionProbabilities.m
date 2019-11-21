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
for i = 1:K
   for j = 1:K
       currPos = stateSpace(i,:); % current state
       nextPos = stateSpace(j,:); % next state
       horzMove = nextPos(1) - currPos(1); % movement along m direction
       vertMove = nextPos(2) - currPos(2); % movement along n direction
       packMove = nextPos(3) - currPos(3); % change in package state
       if packMove~=0 % should not pick package yet
           continue;
       elseif horzMove== 1 && vertMove== 0 % move east
           P1(i,j,EAST) = 1;
       elseif horzMove==-1 && vertMove== 0 % move west
           P1(i,j,WEST) = 1;
       elseif horzMove== 0 && vertMove== 1 % move north
           P1(i,j,NORTH) = 1;
       elseif horzMove== 0 && vertMove==-1 % move south
           P1(i,j,SOUTH) = 1;
       elseif horzMove== 0 && vertMove== 0 % stay
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
% compute number of barriers around states
numBarr = zeros(K, 1); % vector of numbers of barriers around states
M = size(map, 1); % map size in m direction
N = size(map, 2); % map size in n direction
for i = 1:K
    currM = stateSpace(i,1); % current state in m
    currN = stateSpace(i,2); % current state in n
    % detect whether on the edge of m direction
    if currM==1 || currM==M
        numBarr(i) = numBarr(i) + 1;
    end
    % detect whether on the edge of n direction
    if currN==1 || currN==N
        numBarr(i) = numBarr(i) + 1;
    end
    % detect tree in north
    if currN~=N && map(currM,currN+1)==TREE
        numBarr(i) = numBarr(i) + 1;
    end
    % detect tree in south
    if currN~=1 && map(currM,currN-1)==TREE
        numBarr(i) = numBarr(i) + 1;
    end
    % detect tree in east
    if currM~=M && map(currM+1,currN)==TREE
        numBarr(i) = numBarr(i) + 1;
    end
    % detect tree in west
    if currM~=1 && map(currM-1,currN)==TREE
        numBarr(i) = numBarr(i) + 1;
    end
end

%% step 3: angry resident gun shot

%% step 4: pick up package

end

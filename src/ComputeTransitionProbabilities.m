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
global GAMMA R P_WIND P_PEACE  %#ok<*NUSED>
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
       if ( packMove ~= 0 ) % should not pick package yet
           continue;
       elseif ( horzMove == 1 && vertMove == 0 ) % east movement detected
           P1(i, j, EAST) = 1;
       elseif ( horzMove == -1 && vertMove == 0 ) % west movement detected
           P1(i, j, WEST) = 1;
       elseif ( horzMove == 0 && vertMove== 1 ) % north movement detected
           P1(i, j, NORTH) = 1;
       elseif ( horzMove== 0 && vertMove== -1 ) % south movement detected
           P1(i, j, SOUTH) = 1;
       elseif ( horzMove == 0 && vertMove == 0 ) % hover movement detected
           P1(i, j, HOVER) = 1;
       end
   end
end

% clear used variables
clear currPos;
clear nextPos;
clear horzMove;
clear vertMove;
clear packMove;

%% step 2: wind disturbance and angry resident shot
% compute some constants
M = size(map, 1); % map size in m direction
N = size(map, 2); % map size in n direction
P_PEACE = 1 - P_WIND; % probability of a peaceful day

% find base state index
[baseM, baseN] = find( map == BASE );
if isempty(baseM)
    error('Error: Invalid Map (No Base)');
elseif ( size(baseM, 1) > 1 )
    error('Error: Invalid Map (Multiple Bases)');
end
[~, baseIndex] = ismember([baseM, baseN, 0], stateSpace, 'row');
if ~baseIndex
    error('Error: State Space Constructed Wrongly')
end

% find shooters
[shooterM, shooterN] = find( map == SHOOTER ); % find m and n of shooters
shooterPos = [shooterM, shooterN]; % shooter positions

% compute transition matrix after wind
for i = 1:K % iterate starting state
    for j = 1:K % iteratre ending state
        for k = [NORTH, SOUTH, EAST, WEST, HOVER] % iterate actions
            if i == TERMINAL_STATE_INDEX % do not move if mission completed
                P2(i,j,k) = (j == TERMINAL_STATE_INDEX);
            elseif ( P1(i,j,k) == 1 ) % when naive movement valid
                P_SHOT = ComputeShotProbability(stateSpace, j, shooterPos); % if no wind, stay or be shot
                P2(i,j,k) = P2(i,j,k) + (1 - P_WIND) * (1 - P_SHOT); % no wind and no shot
                P2(i,baseIndex,k) = P2(i,baseIndex,k) + (1 - P_WIND) * P_SHOT; % no wind but shot
                for windAction = [NORTH, SOUTH, EAST, WEST] % iterate wind actions
                    [isCrashed, nextStateIndex] = FindNextStateIndex(stateSpace, baseIndex, j, windAction); % find next state resulting from wind
                    if isCrashed
                        P2(i,baseIndex,k) = P2(i,baseIndex,k) + P_WIND/4; % return to base
                    else
                        P_SHOT = ComputeShotProbability(stateSpace, nextStateIndex, shooterPos);
                        P2(i,nextStateIndex,k) = P2(i,nextStateIndex,k) + P_WIND/4*(1-P_SHOT);
                        P2(i,baseIndex,k) = P2(i,baseIndex,k) + P_WIND/4*P_SHOT;
                    end
                end
            else
                continue;
            end
        end
    end
end

%% step 3: pick up package
% find pick up position state index
[pickM, pickN] = find( map == PICK_UP );
if isempty(pickM)
    error('Error: Invalid Map (No Pickup Position)');
elseif ( size(pickM, 1) > 1 )
    error('Error: Invalid Map (Multiple Pickup Positions)');
end
[~, pickIndex0] = ismember([pickM ,pickN ,0], stateSpace, 'row'); % index of state: pickup position without package
if ~pickIndex0
    error('Error: State Space Constructed Wrongly')
end
pickIndex1 = pickIndex0 + 1; % index of state: pickup position with package

% compute final transition probability matrix
P = P2;
for i = 1:K % iterate starting state
    for j = 1:K % iteratre ending state
        for k = [NORTH, SOUTH, EAST, WEST, HOVER] % iterate actions
            if ( j == pickIndex0 )
                P(i,pickIndex1,k) = P(i,pickIndex1,k) + P2(i,pickIndex0,k);
                P(i,pickIndex0,k) = 0;
            end
        end
    end
end

% note that it is impossible that the drone is at pick-up position but not
% carrying a package, comment this loop if you want the same result as
% exampleP.mat
for j = 1:K % iteratre ending state
    for k = [NORTH, SOUTH, EAST, WEST, HOVER] % iterate actions
        P(pickIndex0,j,k) = 0;
    end
end

end
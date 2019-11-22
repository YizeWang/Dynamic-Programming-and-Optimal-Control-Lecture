function P_SHOT = ComputeShotProbability(stateSpace, currStateIndex, shooterPos)
% ComputeShotProbability Compute the probability of being shot
%
%   P_SHOT = ComputeShotProbability(stateSpace, map, currState, shooterPos)
%   Compute the probability of being shot by a angry resident
%
%   Input arguments:
%       stateSpace:
%           A (K x 3) - matrix, where the i-th row represents the i-th
%           element of the state space.
%       currState:
%           A scalar representing the index in stateSpace of current state
%       shooterPos:
%           A matrix where the i-th row represent the i-th shoot position
%
%   Output arguments:
%       P_SHOT:
%       The probability of beging shot by a angry resident

%% declare global variables
global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX

%% parse information
currM = stateSpace(currStateIndex, 1); % current state in m
currN = stateSpace(currStateIndex, 2); % current state in n
numShooter = size(shooterPos, 1); % shooter number

%% compute distance
Distance = zeros(numShooter, 1); % initialize distance vector
for i = 1:numShooter
    shooterM = shooterPos(i, 1); % m of i-th shooter
    shooterN = shooterPos(i, 2); % n of i-th shooter
    Distance(i) = abs(shooterM-currM) + abs(shooterN-currN);
end

%% compute probability of being shot by each shooter
shotP = zeros(numShooter, 1); % initialize shot probability vector
for i = 1:numShooter
    if ( Distance(i) >= 0 && Distance(i) <= R )
        shotP(i) = GAMMA/(Distance(i)+1);
    else
        shotP(i) = 0;
    end
end

%% compute probability of being shot
notShotP = 1 - shotP; % probability of not being shot by each shooter
P_NOT_SHOT = prod(notShotP); % probability of not being shot by all shooters
P_SHOT = 1 - P_NOT_SHOT; % probability of being shot by at least one shooter

end
function G = ComputeStageCosts(stateSpace, map)
% COMPUTESTAGECOSTS Compute stage costs.
% Compute the stage costs for all states in the state space for all
% control inputs.
%
%   G = ComputeStageCosts(stateSpace, map) 
%   computes the stage costs for all states in the state space for all
%   control inputs.
%
%   Input arguments:
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.

%% declare global variables
global GAMMA R P_WIND Nc %#ok<*NUSED>
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K
global TERMINAL_STATE_INDEX
G = zeros( size(stateSpace, 1), 5 );

%% find base state index
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

%% find shooters
[shooterM, shooterN] = find( map == SHOOTER ); % find m and n of shooters
shooterPos = [shooterM, shooterN]; % shooter positions

%% compute stage cost
for curr = 1:K
    for act = [NORTH, SOUTH, EAST, WEST, HOVER] % iterate actions
        currM = stateSpace(curr, 1); % current state in m
        currN = stateSpace(curr, 2); % current state in n
        currPsi = stateSpace(curr, 3); % current state in psi
        % terminal state yields no cost
        if curr == TERMINAL_STATE_INDEX
            G(curr, act) = 0;
            continue;
        % find next position
        elseif act == NORTH
            nextState = [currM currN+1 currPsi];
        elseif act == SOUTH
            nextState = [currM currN-1 currPsi];
        elseif act == EAST
            nextState = [currM+1 currN currPsi];
        elseif act == WEST
            nextState = [currM-1 currN currPsi];
        else % act: hover
            nextState = [currM currN currPsi];
        end
        % if next position causes crash, make cost infinite
        if ~ismember(nextState, stateSpace, 'row')
            G(curr, act) = inf;
        % if next position is allowable
        else
            [~, nextStateIndex] = ismember(nextState, stateSpace, 'row');
            % compute shot probability after movement
            P_SHOT = ComputeShotProbability(stateSpace, nextStateIndex, shooterPos);
            % no wind and no shot
            G(curr, act) = G(curr, act) + 1 * (1 - P_WIND) * (1 - P_SHOT);
            % no wind but shot
            G(curr, act) = G(curr, act) + Nc * (1 - P_WIND) * P_SHOT;
            % wind affects position
            for windAct = [NORTH, SOUTH, EAST, WEST]
                [isCrashed, postWindStateIndex] = FindNextStateIndex(stateSpace, baseIndex, nextStateIndex, windAct);
                % if crash happens after wind action, return to base
                if isCrashed
                    G(curr, act) = G(curr, act) + Nc * P_WIND/4;
                % if no crash happens, check shoot
                else
                    P_SHOT = ComputeShotProbability(stateSpace, postWindStateIndex, shooterPos);
                    G(curr, act) = G(curr, act) + Nc * P_WIND/4 * P_SHOT;
                    G(curr, act) = G(curr, act) + 1 * P_WIND/4 * (1 - P_SHOT);
                end
            end
        end
    end
end

end
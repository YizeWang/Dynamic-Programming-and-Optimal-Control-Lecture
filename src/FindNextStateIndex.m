function [isCrashed, nextStateIndex] = FindNextStateIndex(stateSpace, baseIndex, currState, action)
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
%       baseIndex:
%           A scalar representing the index in stateSpace of base state
%       currState:
%           A scalar representing the index in stateSpace of current state
%       action:
%           A scalar representing the action
%
%   Output arguments:
%       isCrashed:
%           A boolean that indicates whether the drone is crashed
%       nextStateIndex:
%           A scalar representing the index in stateSpace of next state 
%           if action applied. Return the base state index if next state
%           would be tree or outside map

%% declare global variables
global NORTH SOUTH EAST WEST HOVER

%% parse information
isCrashed = false; % not crashed by default
currM = stateSpace(currState, 1); % current state in m
currN = stateSpace(currState, 2); % current state in n
currPsi = stateSpace(currState, 3); % current state in psi

%% compute next state
if action == NORTH && ismember([currM,currN+1,currPsi],stateSpace,'row')
    [~, nextStateIndex] = ismember([currM,currN+1,currPsi],stateSpace,'row');
elseif action == SOUTH && ismember([currM,currN-1,currPsi],stateSpace,'row')
    [~, nextStateIndex] = ismember([currM,currN-1,currPsi],stateSpace,'row');
elseif action == EAST && ismember([currM+1,currN,currPsi],stateSpace,'row')
    [~, nextStateIndex] = ismember([currM+1,currN,currPsi],stateSpace,'row');
elseif action == WEST && ismember([currM-1,currN,currPsi],stateSpace,'row')
    [~, nextStateIndex] = ismember([currM-1,currN,currPsi],stateSpace,'row');
else
    nextStateIndex = baseIndex; % if crash into next state, return to base
    isCrashed = true;
end

end
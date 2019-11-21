function stateIndex = ComputeTerminalStateIndex(stateSpace, map)
% ComputeTerminalStateIndex Compute the index of the terminal state in the
% stateSpace matrix
%
%   stateIndex = ComputeTerminalStateIndex(stateSpace, map) 
%   Computes the index of the terminal state in the stateSpace matrix
%
%   Input arguments:
%       stateSpace:
%           A (K x 3) - matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N) - matrix describing the terrain of the estate map.
%           With values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%       stateIndex:
%           An integer that is the index of the terminal state in the
%           stateSpace matrix

% find drop off position
global DROP_OFF
[dropM,dropN] = find(map==DROP_OFF);

% check existence and uniqueness of drop off position
if isempty(dropM)
    error('Error: Invalid Map (No Drop Off Position)');
elseif (size(dropM,1)>1)
    error('Error: Invalid Map (Multiple Drop Off Positions)');
end

% find terminal state index
[~, stateIndex] = ismember([dropM,dropN,1], stateSpace, 'row');

% error if no index found
if ~stateIndex
    error('Error: State Space Constructed Wrongly')
end

end
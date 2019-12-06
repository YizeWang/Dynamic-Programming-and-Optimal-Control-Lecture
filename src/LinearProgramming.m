function [J_opt, u_opt_ind] = LinearProgramming(P, G)
% LINEARPROGRAMMING Linear Programming
% Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

%% declare global variables
global K HOVER
global TERMINAL_STATE_INDEX

%% initialize variables
J_opt = zeros(K, 1); % initialize optimal cost
J_opt_temp = zeros(K, 5); % store temporary cost-to-go matrix
u_opt_ind = HOVER * ones(K, 1); % initialize optimal control with Hover
nonTerminalState = [1:TERMINAL_STATE_INDEX-1 TERMINAL_STATE_INDEX+1:K]; % all index except terminal state


%% compute optimization input variables
A = []; % A in linprog function
b = []; % b in linprog function
f = -1 * ones(1, K-1); % f in linprog function, negative because max instead of min
% stack A matrix and b vector
for k = 1:5
    A = [A; eye(K-1)-P(nonTerminalState,nonTerminalState,k)];
    b = [b; G(nonTerminalState,k)];
end
% modify b vector: inf -> 1e5
b(isinf(b)) = 1e5;

%% compute J_opt and u_opt_ind
options = optimset('linprog');
options.Display = 'off';
J_opt(nonTerminalState) = linprog(f, A, b, [], [], [], [], options);
for k  = 1:5
    J_opt_temp(nonTerminalState,k) = G(nonTerminalState,k) + P(nonTerminalState,nonTerminalState,k) * J_opt(nonTerminalState);
end
[~, u_opt_ind] = min(J_opt_temp, [], 2);

end


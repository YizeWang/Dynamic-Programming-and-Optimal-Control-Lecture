function [J_opt, u_opt_ind] = PolicyIteration(P, G)
% POLICYITERATION Policy iteration
% Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%       P:
%           A (k x k x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%       G:
%           A (k x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%       J_opt:
%       	A (k x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%       u_opt_ind:
%       	A (k x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

%% declare global variables
global K HOVER
global TERMINAL_STATE_INDEX

%% initialize variables
P_u = zeros(K, K); % inialize coefficent matrix postmultiplied by J given policy
J_opt = zeros(K, 1); % initialize optimal cost
J_prev = rand(K, 1); % initialize with an infinite vector
J_opt_temp = zeros(K, 5); % store temporary cost-to-go matrix
u_opt_ind = HOVER * ones(K, 1); % initialize optimal control with Hover
nonTerminalState = [1:TERMINAL_STATE_INDEX-1 TERMINAL_STATE_INDEX+1:K]; % all index except terminal state

%% start iteration
while( ~isequal(J_opt,J_prev) )
    % update J_prev
    J_prev = J_opt;
    % compute index of elements in G that will end up in q
    G_ind = sub2ind([K,5], 1:K, u_opt_ind'); % compute index from G
    q = G(G_ind)'; % stage cost vector given current policy
    % construct P_u matrix postmultiplied by J given current policy
    for i = 1:K
        P_u(i,:) = P(i,:,u_opt_ind(i));
    end
    % solve J_opt using linear equation solver
    A = eye(K-1) - P_u(nonTerminalState,nonTerminalState); % A matrix
    b = q(nonTerminalState); % b vector
    J_opt(nonTerminalState) = linsolve(A,b);
    % policy improvement
    % construct J_opt_temp: the k-th column indicates the cost-to-go vector
    % after applying control k
    for k  = 1:5
        J_opt_temp(:,k) = G(:,k) + P(:,:,k) * J_opt;
    end
    [~, u_opt_ind] = min(J_opt_temp, [], 2);
end

end

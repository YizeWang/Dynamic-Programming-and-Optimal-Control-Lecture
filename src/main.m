% Dynamic Programming and Optimal Control
% Fall 2019
% Programming Exercise
%
% Author: Yize Wang
% Email: yizwang@student.ethz.ch

%% Clear workspace and command window
clc;
clear all;
close all;

%% Initialize stopwatches
mainStart = tic;

%% Options
mapSize = [15, 20]; % [M, N]
% Set to true to generate a random map of size mapSize, else set to false
% to load the pre-exsisting example map
generateRandomWorld = false;

% Plotting options
global PLOT_POLICY PLOT_COST
PLOT_POLICY = true;
PLOT_COST = false;

%% Global problem parameters
global GAMMA R Nc P_WIND
GAMMA  = 0.2; % Shooter gamma factor
R = 2; % Shooter range
Nc = 10; % Time steps required to bring drone to base when it crashes
P_WIND = 0.1; % Gust of wind probability

% IDs of elements in the map matrix
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE 
FREE = 0;
TREE = 1;
SHOOTER = 2;
PICK_UP = 3;
DROP_OFF = 4;
BASE = 5;

% Index of each action in the P and G matrices. Use this ordering
global NORTH SOUTH EAST WEST HOVER
NORTH  = 1;
SOUTH = 2;
EAST = 3;
WEST = 4;
HOVER = 5;

%% Generate map
% map(m,n) represents the cell at indices (m,n) according to the axes
disp('Generate map');
if generateRandomWorld
	map = GenerateWorld(mapSize(1), mapSize(2)); %#ok<*UNRCH>
else
    % load a pre-generated map
    load('exampleWorld.mat');
end
MakePlots(map);

%% Generate state space
startTime = tic;
% Generate a (K x 3) - matrix 'stateSpace', where each accessible cell is
% represented by two rows (with and without carrying a package).
stateSpace = [];
for m = 1 : size(map, 1)
    for n = 1 : size(map, 2)
        if map(m, n) ~= TREE
            stateSpace = [stateSpace;
                          m, n, 0;
                          m, n, 1];
        end
    end
end
% State space size
global K
K = size(stateSpace,1);
disp("Generate state space: " + toc(startTime) + " sec");

%% Set the following to true as you progress with the files
transitionProbabilitiesImplemented = true;
stageCostsImplemented = true;
valueIterationImplemented = true; 
policyIterationImplemented = false;
linearProgrammingImplemented = false;

%% Compute the terminal state index
global TERMINAL_STATE_INDEX %#ok<*NUSED>
if transitionProbabilitiesImplemented
    TERMINAL_STATE_INDEX = ComputeTerminalStateIndex(stateSpace, map);
end

%% Compute transition probabilities
if transitionProbabilitiesImplemented
    startTime = tic;
    P = ComputeTransitionProbabilities(stateSpace, map);
    disp("Compute transition probabilities: " + toc(startTime) + " sec");
end

%% Compute stage costs
if stageCostsImplemented 
    startTime = tic;
    G = ComputeStageCosts(stateSpace, map);
    disp("Compute stage costs: " + toc(startTime) + " sec");
end

%% Solve stochastic shortest path problem
% value iteration
if valueIterationImplemented
    startTime = tic;
    [ J_opt_vi, u_opt_ind_vi ] = ValueIteration(P, G);
    
    if size(J_opt_vi,1)~=K || size(u_opt_ind_vi,1)~=K
        disp('[ERROR] the size of J and u must be K')
    end
    disp("Solve stochastic shortest path problem with Value Iteration " + toc(startTime) + " sec");
end

% policy iteration
if policyIterationImplemented
    startTime = tic;
    [ J_opt_pi, u_opt_ind_pi ] = PolicyIteration(P, G);
    
    if size(J_opt_pi,1)~=K || size(u_opt_ind_pi,1)~=K
        disp('[ERROR] the size of J and u must be K')
    end
    disp("Solve stochastic shortest path problem with Policy Iteration" + toc(startTime) + " sec");
end

% linear programming
if linearProgrammingImplemented
    startTime = tic;
    [ J_opt_lp, u_opt_ind_lp ] = LinearProgramming(P, G);
    
    if size(J_opt_lp,1)~=K || size(u_opt_ind_lp,1)~=K
        disp('[ERROR] the size of J and u must be K')
    end
    disp("Solve stochastic shortest path problem with Linear Programming" + toc(startTime) + " sec");
end

%% Plot results
startTime = tic;
if valueIterationImplemented
    MakePlots(map, stateSpace, J_opt_vi, u_opt_ind_vi, 'Value iteration');
end
if policyIterationImplemented
    MakePlots(map, stateSpace, J_opt_pi, u_opt_ind_pi, 'Policy iteration');
end
if linearProgrammingImplemented
    MakePlots(map, stateSpace, J_opt_lp, u_opt_ind_lp, 'Linear programming');
end
disp("Plot results " + toc(startTime) + " sec");

%% Terminated
disp("Terminated: " + toc(mainStart) + " sec");
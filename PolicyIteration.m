function [ J_opt, u_opt_ind ] = PolicyIteration( P, G )
%POLICYITERATION Policy iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (k x k x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (k x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (k x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (k x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).

global K HOVER FREE

L = 5;

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

MAX_IT = 99999999;

%% Define array for Q

%shouldnt be changed with different mus cuz we can just define it to cover
%all choices for each input 

q = zeros(K, L);

for i = 1:K
    for u = 1:L
        if G(i, u) == Inf
            q(i, u) = Inf;
        else
            for j = 1:K
              q(i, u) = q(i, u) + P(i, j, u)*G(i, u);
            end
        end
    end
end

%% Define guess of mu 

u_0 = randi([1 5],K,1);
u_0(TERMINAL_STATE_INDEX) = HOVER;

%think we need to do something better here, should be fine with the wind

u_h = u_0;
it = 0;
it2 = 0;
J_h = zeros(K, 1);

while(1)
    
    % Define Matrices for (I-P)J = G
    P_h = zeros(K, K);
    for i = 1:K
        for j = 1:K
            P_h(i, j) = P(i, j, u_h(i)); %here
        end
    end
            
    G_h = zeros(K, 1);
    for i = 1:K
        G_h(i) = q(i, u_h(i));
    end
        
    P_h(TERMINAL_STATE_INDEX, :) = [];
    P_h(:, TERMINAL_STATE_INDEX) = [];
    G_h(TERMINAL_STATE_INDEX, :) = []; %need to do this to ensure non-singularity
    %correct, otherwise matrix is singular

    assignin('base','P_h',P_h)
    
    J_h = (eye(K-1) - P_h)\G_h; %results in J_h with all NaN entries 
    
    J_h = [J_h(1:TERMINAL_STATE_INDEX-1); 0; J_h(TERMINAL_STATE_INDEX:end)]; %add back in cost for terminal state
    
    %% Now lets get a new minimum policy
    
    u_hp1 = ones(K,1); %for some reason it is not getting updated
    u_hp1 = u_hp1*6;

    for i = 1:K
        u_min = [0,99999999999]; %first value is the action (in [1,5]), second is the value (made it arbitrarily big)
        for l = 1:5
            a = q(i, l);
            for j = 1:K
                a = a + P(i, j, l)*J_h(j);
            end

            if a < u_min(2) && a~=0
                u_min = [l, a];
            end
        end
        u_hp1(i) = u_min(1);
    end
    
    %compare new policy to old
    
    a = 0;
    for i = 1:K
       if u_hp1(i) == u_h
            a = a+1;
       end
    end
    
    u_h = u_hp1;

    if a == K || it == MAX_IT
        break
    end
    
    it = it + 1
    
end

J_opt = J_h;
u_opt_ind = u_h;

% J_opt = zeros(K, 1);
% u_opt_ind = zeros(K, 1);

end

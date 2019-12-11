function [ J_opt, u_opt_ind ] = ValueIteration(P, G)
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space. Mapping of the
%       	terminal state is arbitrary (for example: HOVER).
global K HOVER

L = 5;
MAX_DIFF = .001; %use this value * K to determine when we can stop 
MAX_IT = 9999;

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

%% Compute q

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

%define initial guess as random values between 0 and 1

a = .1;
b = 3;
V_0 = (b-a).*rand(K,1) + a; %bounds each element of first guess between 3 and .1?)
V_h = V_0;

%% Begin Iterating

V_hp1 = zeros(K, 1);
u_hp1 = zeros(K, 1);

it = 1;

while(1)
    for i = 1:K
        minv = [0,99999999];
        for u = 1:L
           a = q(i, u);
           for j = 1:K 
               a = a + P(i, j, u)*V_h(j);
           end
           if a < minv(2)
               minv(2) = a;
               minv(1) = u;
           end
        end
        V_hp1(i) = minv(2);
        u_hp1(i) = minv(1);
    end
    
    tot_diff = 0;
    for i = 1:K
        tot_diff = tot_diff + abs(V_hp1(i) - V_h(i));
    end
    
    if tot_diff < MAX_DIFF*K || it == MAX_IT
        disp('Number of iterations required:')
        disp(it)
        break
    end
    
    V_h = V_hp1;
    it = it + 1;
    
end

J_opt = V_hp1;
u_opt_ind = u_hp1;

end

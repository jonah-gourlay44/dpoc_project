function [ J_opt, u_opt_ind ] = LinearProgramming(P, G)
%LINEARPROGRAMMING Linear Programming
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
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

%% Handle terminal state
% Do yo need to do something with the teminal state before starting policy
% iteration ?
global TERMINAL_STATE_INDEX
% IMPORTANT: You can use the global variable TERMINAL_STATE_INDEX computed
% in the ComputeTerminalStateIndex.m file (see main.m)

J_opt = zeros(K,1);
u_opt_ind = zeros(K,1);
f = ones(K-1,1);
V_u = zeros(K-1,4);
I = eye(K-1);

%G(G == Inf) = 50;
G(TERMINAL_STATE_INDEX,:) = [];
P(TERMINAL_STATE_INDEX,:,:) = [];
P(:,TERMINAL_STATE_INDEX,:) = [];
for u=1:4
    ind = find(G(:,u) == Inf);
    P_prime = P;
    P_prime(ind,ind,:) = [];
    x = linprog(-1*f(:,1),I(:,:)-P(:,:,u),G(:,u),[],[],zeros(K-1,1),100*ones(K-1,1));
    V_u(:,u) = x;
end

V_u(V_u <= 0) = 1000;

for i=1:K
    if i == TERMINAL_STATE_INDEX
        J_opt(i) = 0;
        u_opt_ind(i) = HOVER;
    else
        if i > TERMINAL_STATE_INDEX
            [minimum, j] = min(V_u(i-1,:));
        else
            [minimum, j] = min(V_u(i,:));
        end
        J_opt(i) = minimum;
        u_opt_ind(i) = j;
    end
end
    

end


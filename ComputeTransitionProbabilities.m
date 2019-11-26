function P = ComputeTransitionProbabilities( stateSpace, map)
%COMPUTETRANSITIONPROBABILITIES Compute transition probabilities.
% 	Compute the transition probabilities between all states in the state
%   space for all control inputs.
%
%   P = ComputeTransitionProbabilities(stateSpace, map) 
%   computes the transition probabilities between all states in the state 
%   space for all control inputs.
%
%   Input arguments:
%
%       stateSpace:
%           A (K x 3)-matrix, where the i-th row represents the i-th
%           element of the state space.
%
%       map:
%           A (M x N)-matrix describing the world. With
%           values: FREE TREE SHOOTER PICK_UP DROP_OFF BASE
%
%   Output arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.

global GAMMA R P_WIND
global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global NORTH SOUTH EAST WEST HOVER
global K TERMINAL_STATE_INDEX

[pick_up_y, pick_up_x] = find(map==PICK_UP);
[drop_off_y, drop_off_x] = find(map==DROP_OFF);
[base_y, base_x] = find(map==BASE);

%array of tree coordinates
[trees_y, trees_x] = find(map==TREE);
trees = [trees_x, trees_y];

%array of angry neighbor coordinates
[shooters_y, shooters_x] = find(map==SHOOTER);
shooters = [shooters_x, shooters_y];

M = length(map(:,1));
N = length(map(1,:));

P = zeros(K,K,5);

for l=1:5
    x_add = 0;
    y_add = 0;
    if l==NORTH
        y_add = 1;
    elseif l==SOUTH
        y_add = -1;
    elseif l==WEST
        x_add = -1;
    elseif l==EAST
        x_add = 1;
    end 
    for i=1:length(stateSpace)
        i_y=stateSpace(i,1); i_x=stateSpace(i,2); i_package=stateSpace(i,3);
        
        x_new = i_x + x_add;
        y_new = i_y + y_add;
        
        wind = P_WIND * 0.25;
            
        max_x = x_new + 1;
        min_x = x_new - 1;
        max_y = y_new + 1;
        min_y = y_new - 1;
        
        %control input moves drone out of map boundaries
        out_of_bounds = x_new > N || x_new < 1 || y_new > M || y_new < 1;
        
        %control input moves drone into a tree
        tree_collision = ismember(trees, [x_new y_new], 'rows');
        
        %illegal input action
        illegal_input = out_of_bounds | tree_collision;
        
        for j=1:length(stateSpace)
            j_y=stateSpace(j,1); j_x=stateSpace(j,2); j_package=stateSpace(j,3);
            
            is_pickup = [pick_up_x, pick_up_y] == [j_x, j_y];
            is_dropoff = [drop_off_x, drop_off_y] == [j_x, j_y];
            
            %package can be picked up/dropped off
            can_courier = is_pickup | is_dropoff;
            
            %package was picked up/dropped off
            couriered = i_package ~= j_package;
            
            %illegal pick up/drop off scenarios
            illegal_scenario = ~can_courier | (~is_pickup & (j_package == 1)) | (~is_dropoff & (j_package == 0));
            
            %illegal pick up/drop off
            illegal_courier = couriered & illegal_scenario;
     
            
            %don't allow illegal scenarios
            if illegal_courier
                P(i,j,l) = 0; 
            %don't allow illegal input 
            elseif illegal_input
                
                
            end
           
            
            
            
            
        end
    end
end
        
end

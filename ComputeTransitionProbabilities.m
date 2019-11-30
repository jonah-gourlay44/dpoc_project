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

global OUT_OF_BOUNDS
OUT_OF_BOUNDS = -1;

wind = P_WIND * 0.25;

%pick up location
[pick_up_x, pick_up_y] = find(map==PICK_UP);
pick_up = [pick_up_x, pick_up_y];

%drop off location
[drop_off_x, drop_off_y] = find(map==DROP_OFF);
drop_off = [drop_off_x, drop_off_y];

%base location
[base_x, base_y] = find(map==BASE);
base = [base_x, base_y];

start_index = find(stateSpace(:,1) == base_x & stateSpace(:,2) == base_y & stateSpace(:,3) == 0);

%array of free coordinates
[free_x, free_y] = find(map==FREE);
free = [free_x, free_y];

%array of tree coordinates
[trees_x, trees_y] = find(map==TREE);
trees = [trees_x, trees_y];

%array of angry neighbor coordinates
[shooters_x, shooters_y] = find(map==SHOOTER);
shooters = [shooters_x, shooters_y];

P = zeros(K,K,5);

for l=1:5
    
    %determine control input
    switch l
        case HOVER
            x_add = 0;
            y_add = 0;
        case NORTH
            y_add = 1;
            x_add = 0;
        case SOUTH
            y_add = -1;
            x_add = 0;
        case WEST
            x_add = -1;
            y_add = 0;
        case EAST
            x_add = 1;
            y_add = 0;
    end 
    
    for i=1:length(stateSpace)
        i_x=stateSpace(i,1); i_y=stateSpace(i,2); i_package=stateSpace(i,3);
        
        x_new = i_x + x_add;
        y_new = i_y + y_add;
        
        new_coord = [x_new, y_new];
        
        new_square = OUT_OF_BOUNDS;
        
        if sum(ismember(free, new_coord, 'rows')) >= 1
            new_square = FREE;
        elseif sum(ismember(trees, new_coord, 'rows')) >= 1
            new_square = TREE;
        elseif sum(ismember(shooters, new_coord, 'rows')) >= 1
            new_square = SHOOTER;
        elseif sum(new_coord - base) == 0
            new_square = BASE;
        elseif sum(new_coord - pick_up) == 0
            new_square = PICK_UP;
        elseif sum(new_coord - drop_off) == 0
            new_square = DROP_OFF;
        end
        
        %control input moves drone out of map boundaries
        out_of_bounds = new_square == OUT_OF_BOUNDS;
        
        %control input moves drone into a tree
        tree_collision = new_square == TREE;
        
        %illegal input action
        illegal_input = out_of_bounds | tree_collision;
        
        max_x = x_new + 1;
        min_x = x_new - 1;
        max_y = y_new + 1;
        min_y = y_new - 1;
        
        neighbors = [[max_x,y_new,i_package];[min_x,y_new,i_package];[x_new,max_y,i_package];[x_new,min_y,i_package];
                     [max_x,y_new,~i_package];[min_x,y_new,~i_package];[x_new,max_y,~i_package];[x_new,min_y,~i_package]];        
       
        disp(stateSpace(i,:));
        disp(illegal_input);
        %if illegal input, drone is automatically put back at the start
        %position
        if illegal_input
            P(i,:,l) = 0;
            P(i,start_index,l) = 1;
        elseif i == TERMINAL_STATE_INDEX
            P(i,:,l) = 0;
            P(i,TERMINAL_STATE_INDEX,l) = 1;
        else
            for j=1:length(neighbors)
                
                j_x=neighbors(j,1); j_y=neighbors(j,2); j_package=neighbors(j,3);
                
                next_coord = [j_x, j_y];
        
                next_square = OUT_OF_BOUNDS;
        
                if ismember(free, next_coord, 'rows')
                    next_square = FREE;
                elseif ismember(trees, next_coord, 'rows')
                    next_square = TREE;
                elseif ismember(shooters, next_coord, 'rows')
                    next_square = SHOOTER;
                elseif sum(next_coord - base) == 0
                    next_square = BASE;
                elseif sum(next_coord - pick_up) == 0
                    next_square = PICK_UP;
                elseif sum(next_coord - drop_off) == 0
                    next_square = DROP_OFF;
                end
            
                is_pickup = next_square == PICK_UP;
                is_dropoff = next_square == DROP_OFF;
            
                %package can be picked up/dropped off
                can_courier = is_pickup | is_dropoff;
            
                %package was picked up/dropped off
                couriered = i_package ~= j_package;
            
                %illegal pick up/drop off scenarios
                illegal_scenario = ~can_courier | (~is_pickup & (j_package == 1)) | (~is_dropoff & (j_package == 0));
            
                %illegal pick up/drop off
                illegal_courier = couriered & illegal_scenario;
                
                %out of bounds
                out_of_bounds = next_square == OUT_OF_BOUNDS;
                
                %tree
                hit_tree = next_square == TREE;
                
                %in range of angry neighbor
                in_range = false;
                distances = ones(length(shooters)) * -1;
                for k=1:length(shooters)
                    shooter_coord = shooters(k);
                    distance = sum(abs(next_coord - shooter_coord));
                    if distance <= R 
                        in_range = true;
                        distances(k) = distance;
                    end
                end
                
                %blown into tree or out of bounds
                collision = out_of_bounds | hit_tree;
                
                %probability of being shot down
                p_shot_down = 0;
                if in_range
                    for k=1:length(distances)
                        p_shooter = 0;
                        if distances(k) ~= -1
                            p_shooter = GAMMA/(1 + distances(k));
                        end
                        if ~collision
                            P(i,start_index,l) = P(i,start_index,l) + p_shooter;
                        end
                        p_shot_down = p_shot_down + p_shooter;
                    end
                end
            
                %don't allow illegal drop off/pick up scenarios
                if illegal_courier
                    P(i,j,l) = 0; 
                %probability of a collision
                elseif collision
                    P(i,start_index,l) = P(i,start_index,l) + wind;
                    P(i,j,l) = 0;
                %probability to transition to this state if it is a legal
                %transition
                else
                    P(i,j,l) = P(i,j,l) + (1 - p_shot_down);         
                end
            end
        end
    end
end
        
end

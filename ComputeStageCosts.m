function G = ComputeStageCosts( stateSpace, map )
%COMPUTESTAGECOSTS Compute stage costs.
% 	Compute the stage costs for all states in the state space for all
%   control inputs.
%
%   G = ComputeStageCosts(stateSpace, map) 
%   computes the stage costs for all states in the state space for all
%   control inputs.
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
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the expected stage cost if we are in state i and 
%           apply control input l.

    global GAMMA R P_WIND Nc
    global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
    global NORTH SOUTH EAST WEST HOVER
    global K
    global TERMINAL_STATE_INDEX
    
    global OUT_OF_BOUNDS

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
    
    G = zeros(K,5);
    
    for i=1:length(stateSpace)
        
        x_orig = stateSpace(i,1); y_orig = stateSpace(i,2); i_package = stateSpace(i,3);
        
        for l=1:5
            
            switch l
                case NORTH
                    x_add = 0;
                    y_add = 1;
                case SOUTH
                    x_add = 0;
                    y_add = -1;
                case EAST
                    x_add = 1;
                    y_add = 0;
                case WEST
                    x_add = -1;
                    y_add = 0;
                case HOVER
                    x_add = 0;
                    y_add = 0;
            end
            
            x_new = x_orig + x_add;
            y_new = y_orig + y_add;
            
            new_coord = [x_new, y_new];
            
            new_square = OUT_OF_BOUNDS;
            
            if sum(ismember(free, new_coord, 'rows')) >= 1
                new_square = FREE;
            elseif sum(ismember(trees, new_coord, 'rows')) >= 1
                new_square = TREE;
            elseif sum(ismember(shooters, new_coord, 'rows')) >= 1
                new_square = SHOOTER;
            elseif sum(abs(new_coord - base)) == 0
                new_square = BASE;
            elseif sum(abs(new_coord - pick_up)) == 0
                new_square = PICK_UP;
            elseif sum(abs(new_coord - drop_off)) == 0
                new_square = DROP_OFF;
            end
            
            x_max = x_new + 1;
            x_min = x_new - 1;
            y_max = y_new + 1;
            y_min = y_new - 1;
            
            neighbors = [[x_min y_new i_package];[x_min y_new ~i_package];
                         [x_max y_new i_package];[x_max y_new ~i_package];
                         [x_new y_min i_package];[x_new y_min ~i_package];
                         [x_new y_max i_package];[x_new y_max ~i_package];
                         [x_new y_new i_package];[x_new y_new ~i_package]];

            p_crash = 0;
            
            if i == TERMINAL_STATE_INDEX
                G(i,l) = 0;
            elseif new_square == TREE || new_square == OUT_OF_BOUNDS
                G(i,l) = Inf;
            else
                g = zeros(length(stateSpace),1);
                p = zeros(1,length(stateSpace));
                g(start_index,1) = 1;
                for j=1:length(neighbors)
                    x_next = neighbors(j,1); y_next = neighbors(j,2); j_package = neighbors(j,3);

                    next_coord = [x_next, y_next];
            
                    next_square = OUT_OF_BOUNDS;
            
                    if sum(ismember(free, next_coord, 'rows')) >= 1
                        next_square = FREE;
                    elseif sum(ismember(trees, next_coord, 'rows')) >= 1
                        next_square = TREE;
                    elseif sum(ismember(shooters, next_coord, 'rows')) >= 1
                        next_square = SHOOTER;
                    elseif sum(abs(next_coord - base)) == 0
                        next_square = BASE;
                    elseif sum(abs(next_coord - pick_up)) == 0
                        next_square = PICK_UP;
                    elseif sum(abs(next_coord - drop_off)) == 0
                        next_square = DROP_OFF;
                    end

                    %package can be picked up
                    is_pickup = next_square == PICK_UP;
            
                    %package was couriered
                    couriered = i_package ~= j_package;
            
                    %illegal pick up/drop off
                    illegal_courier = (couriered & ~is_pickup) | (~couriered & is_pickup);
                
                    %out of bounds
                    out_of_bounds = next_square == OUT_OF_BOUNDS;
                
                    %tree
                    hit_tree = next_square == TREE;
                
                    %wind
                    wind_gust = sum(next_coord - new_coord) ~= 0;
                
                    %in range of angry neighbor
                    in_range = false;
                    distances = ones(length(shooters),1) * -1;
                    for k=1:length(shooters)
                        shooter_coord = shooters(k, :);
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
                            p_shot_down = p_shot_down + p_shooter;
                        end
                    end
                    
                    m = find(stateSpace(:,1) == x_next & stateSpace(:,2) == y_next & stateSpace(:,3) == j_package);
                
                    %probability of a collision
                    if collision && ~illegal_courier
                        p_crash = p_crash + wind;

                    %probability to transition to this state if it is a legal
                    %transition
                    elseif ~illegal_courier
                        if wind_gust
                            p(1,m) = wind*(1 - p_shot_down);
                            p_crash = p_crash + wind*p_shot_down;
                        else 
                            p(1,m) = (1 - P_WIND)*(1 - p_shot_down);
                            p_crash = p_crash + (1 - P_WIND)*p_shot_down;
                        end
                    end    
                    
                    g(m,1) = 1;  
                end
                crash_cost = Nc * p_crash;
                G(i,l) = dot(g, p);
                G(i,l) = G(i,l) + crash_cost;
            end
        end
    end
   
    
end



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
    global P
    
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
            
            if i == TERMINAL_STATE_INDEX
                G(i,l) = 0;
            elseif new_square == TREE || new_square == OUT_OF_BOUNDS
                G(i,l) = Inf;
            else
                g = zeros(length(stateSpace),1);
                g(start_index,1) = Nc;
                for j=1:length(neighbors)
                    x_next = neighbors(j,1); y_next = neighbors(j,2); j_package = neighbors(j,3);
                    
                    k = find(stateSpace(:,1) == x_next & stateSpace(:,2) == y_next & stateSpace(:,3) == j_package);
                    g(k,1) = 1;   
                end
                
                G(i,l) = dot(g, P(i,:,l));
            end
        end
    end
   
    
end



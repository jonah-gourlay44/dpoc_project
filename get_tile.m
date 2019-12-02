function [tile_name] = get_tile(tile_coord, map)

global FREE TREE SHOOTER PICK_UP DROP_OFF BASE
global OUT_OF_BOUNDS
OUT_OF_BOUNDS = -1;

%pick up location
[pick_up_x, pick_up_y] = find(map==PICK_UP);
pick_up = [pick_up_x, pick_up_y];

%drop off location
[drop_off_x, drop_off_y] = find(map==DROP_OFF);
drop_off = [drop_off_x, drop_off_y];

%base location
[base_x, base_y] = find(map==BASE);
base = [base_x, base_y];

%array of free coordinates
[free_x, free_y] = find(map==FREE);
free = [free_x, free_y];

%array of tree coordinates
[trees_x, trees_y] = find(map==TREE);
trees = [trees_x, trees_y];

%array of angry neighbor coordinates
[shooters_x, shooters_y] = find(map==SHOOTER);
shooters = [shooters_x, shooters_y];

tile_name='OUT_OF_BOUNDS';

if sum(ismember(free, tile_coord, 'rows')) >= 1
    tile_name = 'FREE';
elseif sum(ismember(trees, tile_coord, 'rows')) >= 1
    tile_name = 'TREE';
elseif sum(ismember(shooters, tile_coord, 'rows')) >= 1
    tile_name = 'SHOOTER';
elseif sum(abs(tile_coord - base)) == 0
    tile_name = 'BASE';
elseif sum(abs(tile_coord - pick_up)) == 0
    tile_name = 'PICK_UP';
elseif sum(abs(tile_coord - drop_off)) == 0
    tile_name = 'DROP_OFF';
end
end


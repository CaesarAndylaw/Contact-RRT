function [up_contact, down_contact] = check_horizontal_contact(x_near, map_grid, height)

    up_grid = [height - ceil(x_near.mu(2) + 0.2) + 1, ceil(x_near.mu(1))]; % in the map matrix coordinate
    down_grid = [height - ceil(x_near.mu(2) - 0.2) + 1, ceil(x_near.mu(1))]; % in the map matrix coordinate
    if map_grid(up_grid(1),up_grid(2)) == 1
       up_contact = 1;
    else
       up_contact = 0;
    end
    if map_grid(down_grid(1),down_grid(2)) == 1
       down_contact = 1;
    else
       down_contact = 0;
    end

end
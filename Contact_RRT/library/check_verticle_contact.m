function [left_contact, right_contact] = check_verticle_contact(x_near, map_grid, height)

    left_grid = [height - ceil(x_near.mu(2)) + 1, ceil(x_near.mu(1) - 0.2)]; % in the map matrix coordinate
    right_grid = [height - ceil(x_near.mu(2)) + 1, ceil(x_near.mu(1) + 0.2)]; % in the map matrix coordinate
    if map_grid(left_grid(1),left_grid(2)) == 1
       left_contact = 1;
    else
       left_contact = 0;
    end
    if map_grid(right_grid(1),right_grid(2)) == 1
       right_contact = 1;
    else
       right_contact = 0;
    end

end
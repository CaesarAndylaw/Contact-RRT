function is_contact = check_contact(x_near)
    if (x_near.wall_property == 'V') || (x_near.wall_property == 'H') || (x_near.wall_property == 'B')
        is_contact = true;
    else
        is_contact = false;
    end
end
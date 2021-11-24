function [x_new, invalid_sample] = local_planner_slide(x_near, height, map_grid, eps)
    % make sure the x near is in contact or the sample is invalid
    if x_near.wall_property == 'V' || x_near.wall_property == 'H' || x_near.wall_property == 'B'
        % valid sample, x near is in contact, then we can slide
        invalid_sample = false;
        
        % start to slide 
        if x_near.wall_property == 'V'
            % verticle slide 
            % random pick a direction 
            god_decide = rand(1);
            if god_decide > 0.5
                vec_dir = [0,1];
            else
                vec_dir = [0,-1];
            end
        end
    else
        x_new = [];
        invalid_sample = true;
    end
end
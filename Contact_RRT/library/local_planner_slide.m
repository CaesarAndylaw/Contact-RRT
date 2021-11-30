function [x_new, invalid_sample] = local_planner_slide(p_smp, x_near, height, map_grid, eps)

    % make sure the sliding only happens for those x_near is in contact 
    assert(x_near.wall_property == 'V' || x_near.wall_property == 'H' || x_near.wall_property == 'B')

    % hyper parameters
    resolution = 0.1;
    mu = [0,0];
    Sigma = [1, 0; 0, 1];
    new_particles = [];
    if x_near.wall_property == 'V' || x_near.wall_property == 'H' || x_near.wall_property == 'B'
        % valid sample, x near is in contact, then we can slide
%         invalid_sample = false;
        
        % start to slide 
        if x_near.wall_property == 'V'
            % verticle slide 
            verticle_slide_particles
        end

        if x_near.wall_property == 'H'
            % horizontal slide 
            horizontal_slide_particles
        end

        if x_near.wall_property == 'B'
            % random select a sliding surface 
            seed = rand(1);
            if seed >= 0.5
                % verticle slide 
                verticle_slide_particles
            else
                % horizontal slide 
                horizontal_slide_particles
            end
        end
    end
end
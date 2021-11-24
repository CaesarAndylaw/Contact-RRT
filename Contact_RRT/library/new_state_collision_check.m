function [x_new, invalid_sample] = new_state_collision_check(p_smp, x_near, height, map_grid, eps)
% generate the new state 
    diff = p_smp - x_near.mu;
    resolution = 0.1;
%     steps = ceil(norm(diff) / resolution);
    mu = [0,0];
    Sigma = [1, 0; 0, 1];

    new_particles = [];
    
    % importnat collision count, to count how many points are colliding 
    collision_count = 0;
    for i = 1:size(x_near.particles,1)
        collision = false;

        % sample the motion error for this particle 
        delta =  eps*mvnrnd(mu,Sigma,1);

        % compute the diff for each particle 
        tmp_diff = delta*norm(diff) + diff;
        steps = ceil(norm(tmp_diff) / resolution);
        
        % start moving
        % new moving steps
        for j = 1:steps
            if j <= steps - 1
                tmp_point = x_near.particles(i,:) + j*resolution/norm(tmp_diff)*tmp_diff; % mid point resolution
            else
                tmp_point = x_near.particles(i,:) + tmp_diff; % original target 
            end
            tmp_grid = [height - ceil(tmp_point(2)) + 1, ceil(tmp_point(1))]; % in the map matrix coordinate
%             tmp_grid = [ceil(tmp_point(2)), height - ceil(tmp_point(1)) + 1]; % in the map matrix coordinate

            if map_grid(tmp_grid(1),tmp_grid(2)) == 1
                % there's actually collision
                collision = true;
                % backward a little bit
                tmp_particle = x_near.particles(i,:) + (j-1)*resolution/norm(tmp_diff)*tmp_diff;
                collision_count = collision_count + 1; % count that this particle is in contact
                break % no need further 
            end
        end
        
        if collision == false
            % no collision, then original temp_particle
            tmp_particle = x_near.particles(i,:) + tmp_diff;
        end        

        % collect the new particle
        new_particles = [new_particles; tmp_particle];
    end
    
    invalid_sample = false;
    x_new = State(new_particles); % generate the new state
    if collision_count ~=0 && collision_count ~=  size(new_particles,1)
        % first situation, some contact, some noncontact
        invalid_sample = true;
    end

    if collision_count ~= 0 && min(diag(x_new.Sigma)) > 0.01
        % second situation, all contact, but not same surface
        invalid_sample = true;
    end

    if invalid_sample == false && collision_count ~=0
        % find the valid sample, and if all particles are in contact,
        % then determine vertical or horizontal wall 
        diag_sigma = diag(x_new.Sigma);
        if diag_sigma(1) < 0.01 && diag_sigma(2) > 0.01
            x_new.wall_property = 'V';
        end 
        if diag_sigma(1) > 0.01 && diag_sigma(2) < 0.01
            x_new.wall_property = 'H';
        end 
        if diag_sigma(1) < 0.01 && diag_sigma(2) < 0.01
            x_new.wall_property = 'B';
        end 
    end
end
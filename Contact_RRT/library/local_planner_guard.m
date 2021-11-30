function [x_new, invalid_sample] = local_planner_guard(p_smp, x_near, height, map_grid, eps)
% generate the new state 
    diff = p_smp - x_near.mu;
    resolution = 0.1;
%     steps = ceil(norm(diff) / resolution);
    mu = [0,0];
    Sigma = [1, 0; 0, 1];

    new_particles = [];
    
    for i = 1:size(x_near.particles,1)
        collision = false;

        % sample the motion error for this particle 
        delta =  eps*mvnrnd(mu,Sigma,1);

        % compute the diff for each particle 
        tmp_diff = delta*norm(diff) + diff;
        steps = ceil(norm(tmp_diff) / resolution);
        
        % start moving
        % new moving steps
%         for j = 1:steps
        j = 0;
        while collision == false
            % add one time step 
            j = j + 1;
            tmp_point = x_near.particles(i,:) + j*resolution/norm(tmp_diff)*tmp_diff;

            tmp_grid = [height - ceil(tmp_point(2)) + 1, ceil(tmp_point(1))]; % in the map matrix coordinate
%             tmp_grid = [ceil(tmp_point(2)), height - ceil(tmp_point(1)) + 1]; % in the map matrix coordinate

            if map_grid(tmp_grid(1),tmp_grid(2)) == 1
                % there's actually collision
                collision = true;
                % backward a little bit
                tmp_particle = x_near.particles(i,:) + (j-1)*resolution/norm(tmp_diff)*tmp_diff;
                break % no need further 
            end
        end
        assert(collision == true);

        % collect the new particle
        new_particles = [new_particles; tmp_particle];
    end
    
    % since all of the particles are in contact, we will check if in the
    % same contact surface
    invalid_sample = false;
    x_new = State(new_particles); % generate the new state
%     if collision_count ~=0 && collision_count ~=  size(new_particles,1)
%         % first situation, some contact, some noncontact
%         invalid_sample = true;
%     end

    if min(diag(x_new.Sigma)) > 0.01
        % second situation, all contact, but not same surface
        invalid_sample = true;
    end

    if invalid_sample == false
        % find the valid sample, and if all particles are in contact,
        % then determine vertical or horizontal wall 
        diag_sigma = diag(x_new.Sigma);
        if diag_sigma(1) < 0.01 && diag_sigma(2) > 0.01
            % make sure it is actually in contact 
            [left_contact, right_contact] = check_verticle_contact(x_new, map_grid, height);
            if left_contact + right_contact == 1
                x_new.wall_property = 'V';
            end
        end 
        if diag_sigma(1) > 0.01 && diag_sigma(2) < 0.01
            [up_contact, down_contact] = check_horizontal_contact(x_new, map_grid, height);
            if up_contact + down_contact == 1
                x_new.wall_property = 'H';
            end
        end 
        if diag_sigma(1) < 0.01 && diag_sigma(2) < 0.01
            [left_contact, right_contact] = check_verticle_contact(x_new, map_grid, height);
            [up_contact, down_contact] = check_horizontal_contact(x_new, map_grid, height);
            if up_contact + down_contact == 1 && left_contact + right_contact == 1
                x_new.wall_property = 'B';
            end
        end 
    end
end
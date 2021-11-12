function x_new = no_motion_error_state_collision_check(p_smp, x_near, height, map_grid, eps)
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
        
        % old moving steps 
        % start moving
        for j = 1:steps
            if j <= steps - 1
                tmp_point = x_near.particles(i,:) + j*resolution/norm(diff)*diff; % mid point resolution
            else
                tmp_point = x_near.particles(i,:) + diff; % original target
            end
            tmp_grid = [height - ceil(tmp_point(2)) + 1, ceil(tmp_point(1))]; % in the map matrix coordinate

            if map_grid(tmp_grid(1),tmp_grid(2)) == 1
                % there's actually collision
                collision = true;
                % backward a little bit
                tmp_particle = x_near.particles(i,:) + (j-1)*resolution/norm(diff)*diff;
                break % no need further 
            end
        end
        
        if collision == false
            % no collision, then original temp_particle
            tmp_particle = x_near.particles(i,:) + diff;
        end  

        % collect the new particle
        new_particles = [new_particles; tmp_particle];
    end
    x_new = State(new_particles);
end
function [reach_goal_x, reach] = check_cross_target(edge, goal, radius)
% check if the particles of this vector come across the target 
    x_new = edge.edge(1);
    x_near = edge.edge(2);
    mu_old = x_near.mu;
    mu_new = x_new.mu;

    % reach the target
    reach = false;
    
    % check cross at certain resolution 
    % directly check the mean of the distribution 
    resolution = 0.1;
    step = ceil(norm(mu_new - mu_old)/resolution);
    for i = 1:step-1
        mu_tmp = mu_old + (mu_new - mu_old)*i/step;
        if norm(mu_tmp - goal) < radius
            disp('reach the target');
            tmp_step = i;
            reach = true;
            break;
        end
    end
    
    % there is some mean reached the goal, now recover the particles
    reach_goal_x = [];
    if reach == true
        particle_diff = x_new.particles - x_near.particles;
        new_particles = x_near.particles + tmp_step * resolution / norm(mu_new - mu_old) * particle_diff;
        reach_goal_x = State(new_particles);
    end
end
            % determine left contact or right contact 
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

            if left_contact + right_contact ~= 1
                disp(left_contact);
            end
            assert(left_contact + right_contact == 1); % there should only be one contact

            % get the projected goal 
            proj_smp = [x_near.mu(1), p_smp(2)];

            % get the diff 
            diff = proj_smp - x_near.mu;
            assert(diff(1) == 0);
            
            
            % move every particle until, reach the projected goal, leave
            % contact, or collide with another wall 
            collision_count = 0;
            leave_count = 0;
            for i = 1:size(x_near.particles,1)
                collision = false;
                leave = false;
        
                % sample the motion error for this particle 
                delta =  eps*mvnrnd(mu,Sigma,1);
                delta_contact = [0, delta(2)]; % verticle slide
                
                % compute the diff for each particle 
                tmp_diff = delta_contact*norm(diff) + diff;
                assert(tmp_diff(1) == 0);
                steps = ceil(norm(tmp_diff) / resolution);
                
                % start moving
                % new moving steps
        %         for j = 1:steps
               for j = 1:steps
                    if j <= steps - 1
                        tmp_point = x_near.particles(i,:) + j*resolution/norm(tmp_diff)*tmp_diff; % mid point resolution
                    else
                        tmp_point = x_near.particles(i,:) + tmp_diff; % original target 
                    end
                    tmp_grid = [height - ceil(tmp_point(2)) + 1, ceil(tmp_point(1))]; % in the map matrix coordinate
        %             tmp_grid = [ceil(tmp_point(2)), height - ceil(tmp_point(1)) + 1]; % in the map matrix coordinate
                    

                    %%%%%% break if there's another contact %%%%%%%
                    if map_grid(tmp_grid(1),tmp_grid(2)) == 1
                        % there's actually collision
                        collision = true;
                        % backward a little bit
                        tmp_particle = x_near.particles(i,:) + (j-1)*resolution/norm(tmp_diff)*tmp_diff;
                        collision_count = collision_count + 1; % count that this particle is in contact
                        break % no need further 
                    end

                    %%%%%% break if the particle leaves contact %%%%%%%
                    if (left_contact == 1 && right_contact == 0)
                        should_contact_grid = [height - ceil(tmp_point(2)) + 1, ceil(tmp_point(1) - 0.2)]; % in the map matrix coordinate
                    end
                    if (left_contact == 0 && right_contact == 1)
                        should_contact_grid = [height - ceil(tmp_point(2)) + 1, ceil(tmp_point(1) + 0.2)]; % in the map matrix coordinate
                    end

                    if map_grid(should_contact_grid(1),should_contact_grid(2)) == 0
                        % left the contact already 
                        leave = true;
                        % no need to backward a little bit
%                         tmp_particle = x_near.particles(i,:) + (j-1)*resolution/norm(tmp_diff)*tmp_diff;
                        tmp_particle = tmp_point;
                        leave_count = leave_count + 1; % count that this particle is in contact
                        break % no need further 
                    end
                end
                
                if collision == false && leave == false
                    % no collision, and not leave contact, then original temp_particle
                    tmp_particle = x_near.particles(i,:) + tmp_diff;
                end    

                % collect the new particle
                new_particles = [new_particles; tmp_particle];
            end


            % determine if the sample is invalid 
            invalid_sample = false;
            x_new = State(new_particles); % generate the new state
            if collision_count ~=0 && collision_count ~=  size(new_particles,1)
                % first situation, some contact, some noncontact
                invalid_sample = true;
            end

            if leave_count ~=0 && leave_count ~=  size(new_particles,1)
                % second situation, some leave contact, some still contact
                invalid_sample = true;
            end
        
            if invalid_sample == false && collision_count ~=0
                % find the valid sample, and if all particles are in contact,
                % then determine vertical or horizontal wall 
                diag_sigma = diag(x_new.Sigma);
%                 if diag_sigma(1) < 0.01 && diag_sigma(2) > 0.01
%                     x_new.wall_property = 'V';
%                 end 
%                 if diag_sigma(1) > 0.01 && diag_sigma(2) < 0.01
%                     x_new.wall_property = 'H';
%                 end 
%                 if diag_sigma(1) < 0.01 && diag_sigma(2) < 0.01
%                     x_new.wall_property = 'B';
%                 end 
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

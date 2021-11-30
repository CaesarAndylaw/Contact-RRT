classdef State
   properties
      mu
      Sigma
      particles
      predecessor
      wall_property % can only be 'H': horizontal or 'V': vertical or 'B' both contact; 'N' denotes not in contact
   end
   methods        
        function obj = State(particles)
            if nargin == 1
                obj.particles = particles;
                % update the mean and covariance
                assert(size(obj.particles,2) == 2, 'the state dimension is not 2');
                obj.mu = mean(obj.particles,1);
                obj.Sigma = cov(obj.particles);
                obj.wall_property = 'N';
                assert(isequal(size(obj.Sigma), [2,2]));
                assert(isequal(size(obj.mu), [1,2]));
            end
        end

        function update_mean_var(obj)
            assert(size(obj.particles,2) == 2, 'the state dimension is not 2');
            obj.mu = mean(obj.particles,1);
            obj.Sigma = cov(obj.particles);
            assert(isequal(size(obj.Sigma), [2,2]));
            assert(isequal(size(obj.mu), [1,2]));
        end
    end
end

%%%%%%%%%%%% the main function for RRT simulator %%%%%%%%%%%%
clc
clear

%% construct the 2D map with obstacle 
% square map
map_grid = [1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1
            1	0	0	0	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	0	1
            1	0	0	0	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	0	1
            1	0	0	0	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	0	1
            1	0	0	0	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	0	1
            1	0	0	0	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	0	1
            1	0	0	0	0	0	0	0	0	0	0	0	0	0	0   0 	0	0	0	0	0	0	0	0	0	0	1
            1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
            1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
            1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
            1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1	1	1	1	1	1	1   1
            1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
            1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
            1	1	1	1	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
            1	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
            1	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
            1	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
            1	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
            1	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
            1	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
            1	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
            1	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
            1	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
            1	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
            1	0	0	0	0	0	0	0	0	0	1	0	0	0	0	0	0	0	0	0	0	0	0	0	0	0	1
            1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1	1];

map = occupancyMap(map_grid,1);
sv.Map = map;
x_lim = map.XLocalLimits;
y_lim = map.YLocalLimits;


%% goal and start with probability
% initial settings for mean and covariance for initial point 
mu = [2 3];
Sigma = 0.1*[1 1.5; 1.5 3];
% rng('default')  % For reproducibility
rng(2)
R = mvnrnd(mu,Sigma,10);
x_init = State(R);
% reset mu sigma to groundtruth value 
x_init.mu = mu;
x_init.Sigma = Sigma;

% set the goal
goal = [20, 8];
radius = 1;


%%  contact RRT initialization 
reach = false; 

V = [];
E = [];

V = [V x_init];
%% the implementation of RRT with uncertainties
while ~reach
    
    % randomly configuration sample 
    p_smp = [x_lim(1) + (x_lim(2)-x_lim(1)).*rand, y_lim(1) + (y_lim(2)-y_lim(1)).*rand];
    
    % select the nearest vertice 
    x_near = nearest_point(V, p_smp);

    % generate the new vertice
    x_new = new_state(p_smp, x_near);

    % add new vertex and edges 
    x_new.predecessor = x_near;
    V = [V x_new];
    edge_new = Edge(x_new, x_near);
    E = [E edge_new];

    % judge if need to quit 
    if norm(x_new.mu - goal) < radius
        disp('reach the target');
        reach = true;
        final_x = x_new;
    end
end

%% visualization 
show(map);
hold on 
plot(R(:,1),R(:,2),"Color",'r','Marker','o','LineStyle','none');
% goal
h = ccl(goal(1),goal(2),radius);


% draw the RRT planned trajectory (mu trajectory)
% x_tmp = final_x;
% trajectory = [x_tmp.mu];
% while true
%     trajectory = [trajectory; x_tmp.predecessor.mu];
%     x_tmp = x_tmp.predecessor;
%     if size(x_tmp.predecessor, 1) == 0
%         break
%     end
% end
% hold on 
% plot(trajectory(:,1),trajectory(:,2),"Color",'b','Marker','*','LineStyle','-');

% draw the particle trajectory 
x_tmp = final_x;
for i = 1:size(x_tmp.particles, 1)
    x_tmp = final_x;
    trajectory = [x_tmp.particles(i,:)];
    while true
        trajectory = [trajectory; x_tmp.predecessor.particles(i,:)];
        x_tmp = x_tmp.predecessor;
        if size(x_tmp.predecessor, 1) == 0
            break
        end
    end
    hold on 
    plot(trajectory(:,1),trajectory(:,2),"Color",[0 0.4470 0.7410],'LineStyle','-','LineWidth',1);
end





%% functions 
function x_near = nearest_point(V, x_smp)
% find the nearest point based on x sample
    min_dist = inf;
    for i = 1:size(V, 2)
        dist = norm(V(i).mu - x_smp);
        if dist < min_dist
            min_idx = i;
            min_dist = dist;
        end
    end
    x_near = V(min_idx);
end

function x_new = new_state(p_smp, x_near)
% generate the new state 
    diff = p_smp - x_near.mu;
    new_particles = [];
    for i = 1:size(x_near.particles,1)
        tmp_particle = x_near.particles(i,:) + diff;
        new_particles = [new_particles; tmp_particle];
    end
    x_new = State(new_particles);
end
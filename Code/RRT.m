addpath GJK2D
clear all; close all;
startup_rvc
numLinks = 2;
L = repmat(Link('d', 0, 'a', 10, 'alpha', 0, 'm', .001, 'r', [-0.5, 0, 0]), numLinks, 1);
R = SerialLink(L);
R.base = trotx(pi/2)*R.base;

w = 1; 

obs{1} = [-0.5 -15; 0.5 -15; 0.5 15; -0.5 15];
obs{1} = bsxfun(@plus, obs{1}, [-11.5 20]);%; 1 0.9; 1.2 0.9; 1.2 1.1; 1 1.1];
obs{2} = [-0.5 -17.5; 0.5 -17.5; 0.5 17.5; -0.5 17.5];
obs{2} = bsxfun(@plus, obs{2}, [11.5 17.5]);
obs{3} = [-0.5 -17.5; 0.5 -17.5; 0.5 17.5; -0.5 17.5];
obs{3} = bsxfun(@plus, obs{3}, [-11.5 -17.5]);
obs{4} = [-0.5 -15; 0.5 -15; 0.5 15; -0.5 15];
obs{4} = bsxfun(@plus, obs{4}, [11.5 -20]);

for i = 1:length(obs)
    obs{i} = cat(1, obs{i}, obs{i}(1,:));
end

base_angle = linspace(0,2*pi,360);
upper_angle = linspace(-pi,pi,360);
b=2*pi/359;
u=2*pi/359;
c_space=[];

for i = 1:length(base_angle)
    for j=1:length(upper_angle)
        P = generateArmPolygons(R, [base_angle(i) upper_angle(j)], w);
        if ~gjk2Darray(P, obs)
            c_space(i,j)=1;
        else
            c_space(i,j)=0;
        end
    end
end

figure
subplot(121)
colormap([1,1,1;0,0,0]);
imagesc(255*c_space');
title('Configuration space')
set(gca,'YDir','normal');
% plot axis from -180 to 180
xlabel('q1')
ylabel('q2')
hold on

%%
% joint in rad, q_start/goal in deg, collision in rad 
% graph title from 0 to 360,  actual from -180 to 180
stepsize = 3;
numNodes = 3000;        

waypoint1=[-5,-5];
waypoint2=[-9,0];

q_start.coord=rad2deg(inverseKin(R,waypoint1(1),waypoint1(2),1));
q_start.coord(1)=wrapTo360(q_start.coord(1));
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord=wrapTo180(rad2deg(inverseKin(R,waypoint2(1),waypoint2(2),1)));
q_goal.coord(:,1)=wrapTo360(q_goal.coord(:,1));
q_goal.cost = 0;               % used for backwards


plot(wrapTo360(q_goal.coord(:,1)),q_goal.coord(:,2)+180,'xr');
hold on
plot(q_start.coord(1),q_start.coord(2)+180,'xg');
hold on

nodes(1) = q_start;

for i = 1:1:numNodes
    % generate biased random number 
    q_rand = [biasedrandom(360,0,q_goal.coord(1),0.15),biasedrandom(180,-180,q_goal.coord(2),0.15)];   % 
%     q_rand2 = [biasedrandom(360,0,q_goal.coord(2,1),0.15),biasedrandom(180,-180,q_goal.coord(2,2),0.15)];
%     if rand(1)<=0.5
%         q_rand = q_rand;
%     else
%         q_rand = q_rand2;
%     end
    % Break if goal node is already reached
    for j = 1:1:length(nodes)
        if pdist([nodes(j).coord; q_goal.coord])<=1
            break
        end
    end
    
    % Pick the closest node from existing list to branch out from
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = pdist([n.coord; q_rand],'euclidean');
        ndist = [ndist tmp];     % node distance
    end
    [mindis, idx] = min(ndist);
    q_near = nodes(idx);        % closest node
    
    % find qnew 
    q_new.coord = new_conf(q_rand, q_near.coord, mindis, stepsize);
    % need to do a interpolation from q_rand to q_new.coord then use
    % P=generateArmPolygons to check each point whether in collision.
    % Probably do in a function
    if ~collision(deg2rad(q_near.coord),deg2rad(q_rand),obs,R);   % collision check from q_new.coord to q_rand no collision
        line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2)+180, q_new.coord(2)+180], 'Color', 'k');
        drawnow
        hold on
        q_new.cost = pdist([q_new.coord; q_near.coord],'euclidean') + q_near.cost;
        
        % Update parent to least cost node
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_near.coord
                q_new.parent = j;
            end
        end
        
        % Append the new nodes to nodes
        nodes = [nodes q_new];
    end
end

% q_goal.coord=nodes(j).coord;

D = [];
for j = 1:1:length(nodes)
    tmpdist = pdist([nodes(j).coord; q_goal.coord],'euclidean');
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
q_list=[];
[mindis, idx] = min(D);
q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];
while q_end.parent ~= 0
    start = q_end.parent;
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2)+180, nodes(start).coord(2)+180], 'Color', 'r', 'LineWidth', 1);
    q_list=[q_end.coord(1), q_end.coord(2); q_list];
    hold on
    q_end = nodes(start);
end
q_list=deg2rad([nodes(start).coord(1),nodes(start).coord(2);q_list]);
title('waypoint 6 to waypoint 7')

%% plot
subplot(122)
time_delay = 0.05;
axLim = [-40 40 -10 10 -35 35];
R.plotopt = {'view', 'x', 'noshading', 'noname', 'ortho', ...
    'workspace', axLim, 'tile1color', [1,1,1], 'delay', time_delay, 'trail', '',...
    'noshadow', 'nobase', 'nowrist', ...
    'linkcolor', 'b', 'toolcolor', 'b', ...
    'jointcolor', 0.2*[1 1 1], 'jointdiam', 2, 'scale', 0.5};
axis(axLim); hold on;

for i = 1:length(obs)
    fill3(obs{i}(:,1), ones(size(obs{i}(:,1))), obs{i}(:,2), 0.8*[1 1 1], 'facealpha', 1)
end
hold on

anim = Animate('P6-P7');
for i = 1:length(q_list)
    R.plot([q_list(i,:)])
    drawnow limitrate;
    anim.add();
end






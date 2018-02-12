addpath GJK2D
clear all; close all;
startup_rvc
time_delay = 0.05;
axLim = [-40 40 -10 10 -35 35];
numLinks = 2;
L = repmat(Link('d', 0, 'a', 10, 'alpha', 0, 'm', .001, 'r', [-0.5, 0, 0]), numLinks, 1);
R = SerialLink(L);
R.base = trotx(pi/2)*R.base;

% R.plotopt = {'view', 'x', 'noshading', 'noname', 'ortho', ...
%     'workspace', axLim, 'tile1color', [1,1,1], 'delay', time_delay, 'trail', '',...
%     'noshadow', 'nobase', 'nowrist', ...
%     'linkcolor', 'b', 'toolcolor', 'b', ...
%     'jointcolor', 0.2*[1 1 1], 'jointdiam', 2, 'scale', 0.5};
% axis(axLim); axis square; hold on;
% R.plot([0 0]);
w = 1; % get width of robot arm based on its size in the figure

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
    fill3(obs{i}(:,1), ones(size(obs{i}(:,1))), obs{i}(:,2), 0.8*[1 1 1], 'facealpha', 1)
end

base_angle = linspace(0,2*pi,360);
upper_angle = linspace(0,2*pi,360);
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
colormap([1,1,1;0,0,0]);
c_space(:,180)=1;
imagesc(255*c_space');
title('Q2 (a) Configuration space')
xlabel('q1')
ylabel('q2')

%%



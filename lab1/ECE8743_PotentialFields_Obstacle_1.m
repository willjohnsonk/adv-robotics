%%% Southfield,Michigan
%%% May 23, 2016
%%% Potential Fields for Robot Path Planning
%
%
% Initially proposed for real-time collision avoidance [Khatib 1986].  
% Hundreds of papers published on APF
% A potential field is a scalar function over the free space.
% To navigate, the robot applies a force proportional to the 
% negated gradient of the potential field.
% A navigation function is an ideal potential field 

clc
close all
clear 
%% Defining environment variables
startPos = [5,5];   % Origin of robot (x, y)                    [5,5]
goalPos = [90, 95]; % Position of the goal (x, y)               [90,95]
obs1Pos = [50, 50]; % Position of a object (x, y)               [50,50]
obsRad = 10;        % The radius of the object                  10
goalR = 0.2;        % The radius of the goal                    0.2
goalS = 15;         % The spread of attraction of the goal      20
obsS = 14;          % The spread of repulsion of the obstacle   30
alpha = 0.55;        % Strength of attraction                    0.8
beta = 0.8;         % Strength of repulsion                     0.6


%% Carry out the Potential Field Math as follows: 

u = zeros(100, 100);        % Initialize potential field values (u,v) for every
v = zeros(100, 100);        % point in the grid as 0
% testu = zeros(100, 100);
% testv = zeros(100, 100);

% Iteration over the grid to definite the PF vector at each point value
for x = 1:1:100
    for y = 1:1:100
        [uG, vG] = GoalDelta(x, y, goalPos(1), goalPos(2), goalR, goalS, alpha);    % Attractive delta calculation
        [uO, vO] = ObsDelta(x, y, obs1Pos(1), obs1Pos(2), obsRad, obsS, beta);      % Repulsive delta calculation
        xnet = uG + uO;                 % Net delta values for each (x, y) point in the field
        ynet = vG + vO;
        vspeed = sqrt(xnet^2 + ynet^2); % Velocity calculations for each point
        theta = atan2(ynet,xnet);
        u(x,y) = vspeed*cos(theta);     % Final potential field value for each point based on attraction/repulsion
        v(x,y) = vspeed*sin(theta);
%         hold on
        
    end
end
% Vizualize the potential field grid
[X,Y] = meshgrid(1:1:100,1:1:100);
figure
quiver(Y, X, u, v, 3)


%% Defining the grid

% Plotting the obstacle using the circle function
circles(obs1Pos(1),obs1Pos(2),obsRad, 'facecolor','red')
axis square

hold on % Plotting start position
circles(startPos(1),startPos(2),2, 'facecolor','green')

hold on % Plotting goal position
circles(goalPos(1),goalPos(2),2, 'facecolor','yellow')

%% Priting of the path
currentPos = startPos;
x = 0;

% Pathfinding calculation and visualization
while sqrt((goalPos(1)-currentPos(1))^2 + (goalPos(2)-currentPos(2))^2) > 1                 % While the distance to the goal > 1
    tempPos = currentPos + [u(currentPos(1),currentPos(2)), v(currentPos(1),currentPos(2))]; % Find position value of next point given PF
    currentPos = round(tempPos);                                                             % Round previous value to update position
    hold on
    plot(currentPos(1),currentPos(2),'-o', 'MarkerFaceColor', 'black')                      % Update the grid with new position
    pause(0.5)
end

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
startPos = [5,15];  % Origin of the robot (x, y)
goalPos = [90, 95]; % Position of the goal (x, y)
obs1Pos = [50, 50]; % Position of object 1 (x, y)
obs2Pos = [30, 80]; % Position of object 2 (x, y)
obsRad = 10; % Radius of the objects                    10
goalR = 0.2; % The radius of the goal                   0.2
goalS = 15;  % The spread of attraction of the goal     20
obsS = 16;   % The spread of repulsion of the obstacle  20
alpha = 0.65; % Strength of attraction                   0.8
beta = 0.65;  % Strength of repulsion                    0.8


%% To perform the Potential Field Math as follows:
u = zeros(100, 100);        % Initialize the potential field values for every point (u, v)
v = zeros(100, 100);        % as 0
% testu = zeros(100, 100);
% testv = zeros(100, 100);

% Iteration over the grid to define the potential field at each point value
for x = 1:1:100
    for y = 1:1:100
        [uG, vG] = GoalDelta(x, y, goalPos(1), goalPos(2), goalR, goalS, alpha);    % Delta calculation for the attractive force
        [uO, vO] = ObsDelta(x, y, obs1Pos(1), obs1Pos(2), obsRad, obsS, beta);      % Delta calculation for object 1's repulsive force
        [uO2, vO2] = ObsDelta(x, y, obs2Pos(1), obs2Pos(2), obsRad, obsS, beta);    % Delta calculation for object 2's repulsive force
        xnet = uG + uO + uO2;   % Net delta calculations for each point in the grid
        ynet = vG + vO + vO2;
        vspeed = sqrt(xnet^2 + ynet^2); % Calculation of the velocity vector for each point
        theta = atan2(ynet,xnet);
        u(x,y) = vspeed*cos(theta);     % Final potential field values for (x, y) points
        v(x,y) = vspeed*sin(theta);
%         hold on
        
    end
end
%%

% Vizualize the potential field grid
[X,Y] = meshgrid(1:1:100,1:1:100);
figure
quiver(Y, X, u, v, 3)


%% Defining the grid

% Plotting the obstacles
circles(obs1Pos(1),obs1Pos(2),obsRad, 'facecolor','red')
axis square

hold on
circles(obs2Pos(1),obs2Pos(2),obsRad, 'facecolor','red')

hold on % Plotting start position
circles(startPos(1),startPos(2),2, 'facecolor','green')

hold on % Plotting goal position
circles(goalPos(1),goalPos(2),2, 'facecolor','yellow')

%% Priting of the path
currentPos = startPos;
x = 0;

% Pathfinding calculation and visualization
while sqrt((goalPos(1)-currentPos(1))^2 + (goalPos(2)-currentPos(2))^2) > 1                 % While distance to goal > 1
    tempPos = currentPos + [u(currentPos(1),currentPos(2)), v(currentPos(1),currentPos(2))]; % Find next position given PF values
    currentPos = round(tempPos);                                                             % Set new position and round
    hold on
    plot(currentPos(1),currentPos(2),'o', 'MarkerFaceColor', 'black')                       % Draw to the screen
    pause(0.5)
end

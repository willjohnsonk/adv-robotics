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

% https://ieeexplore.ieee.org/document/5565069

clc
close all
clear 
%% Defining environment variables
startPos = [22, 41];  % Origin of the robot (x, y)
goalPos = [80, 41]; % Position of the goal (x, y)

% Defining individual obstacles
obs1Pos = [27, 31]; % Position of object 1 (x, y)
obs1Rad = 4; % Radius of the object

obs2Pos = [36, 42]; % Position of object 2 (x, y)
obs2Rad = 4; % Radius of the object

obs3Pos = [48, 55]; % Position of object 3 (x, y)
obs3Rad = 4; % Radius of the object

obs4Pos = [51, 27]; % Position of object 4 (x, y)
obs4Rad = 4; % Radius of the object

obs5Pos = [56, 46]; % Position of object 5 (x, y)
obs5Rad = 4; % Radius of the object

obs6Pos = [64, 34]; % Position of object 6 (x, y)
obs6Rad = 4; % Radius of the object

obs7Pos = [75, 35]; % Position of object 7 (x, y)
obs7Rad = 4; % Radius of the object


goalR = 0.2; % The radius of the goal
goalS = 10;  % The spread of attraction of the goal
obsS = 10;   % The spread of repulsion of the obstacle
alpha = 0.65; % Strength of attraction
beta = 0.6;  % Strength of repulsion


%% To perform the Potential Field Math as follows:
u = zeros(100, 100);        % Initialize the potential field values for every point (u, v)
v = zeros(100, 100);        % as 0
% testu = zeros(100, 100);
% testv = zeros(100, 100);

% Iteration over the grid to define the potential field at each point value
for y = 1:1:100
    for x = 1:1:100
        [uG, vG] = GoalDelta(x, y, goalPos(1), goalPos(2), goalR, goalS, alpha);     % Delta calculation for the attractive force
        [uO1, vO1] = ObsDelta(x, y, obs1Pos(1), obs1Pos(2), obs1Rad, obsS, beta);      % Delta calculation for object 1's repulsive force
        [uO2, vO2] = ObsDelta(x, y, obs2Pos(1), obs2Pos(2), obs2Rad, obsS, beta);    % Delta calculation for object 2's repulsive force
        [uO3, vO3] = ObsDelta(x, y, obs3Pos(1), obs3Pos(2), obs3Rad, obsS, beta);    % Delta calculation for object 3's repulsive force
        [uO4, vO4] = ObsDelta(x, y, obs4Pos(1), obs4Pos(2), obs4Rad, obsS, beta);    % Delta calculation for object 4's repulsive force
        [uO5, vO5] = ObsDelta(x, y, obs5Pos(1), obs5Pos(2), obs5Rad, obsS, beta);    % Delta calculation for object 5's repulsive force
        [uO6, vO6] = ObsDelta(x, y, obs6Pos(1), obs6Pos(2), obs6Rad, obsS, beta);    % Delta calculation for object 6's repulsive force
        [uO7, vO7] = ObsDelta(x, y, obs7Pos(1), obs7Pos(2), obs7Rad, obsS, beta);    % Delta calculation for object 7's repulsive force
        
        xnet = uG + uO1 + uO2 + uO3 + uO4 + uO5 + uO6 + uO7;   % Net delta calculations for each point in the grid
        ynet = vG + vO1 + vO2 + vO3 + vO4 + vO5 + vO6 + vO7;
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

% Plotting the obstacles using circle function
circles(obs1Pos(1),obs1Pos(2),obs1Rad, 'facecolor','red')
axis square

hold on
circles(obs2Pos(1),obs2Pos(2),obs2Rad, 'facecolor','red')

hold on
circles(obs3Pos(1),obs3Pos(2),obs3Rad, 'facecolor','red')

hold on
circles(obs4Pos(1),obs4Pos(2),obs4Rad, 'facecolor','red')

hold on
circles(obs5Pos(1),obs5Pos(2),obs5Rad, 'facecolor','red')

hold on
circles(obs6Pos(1),obs6Pos(2),obs6Rad, 'facecolor','red')

hold on
circles(obs7Pos(1),obs7Pos(2),obs7Rad, 'facecolor','red')


hold on % Plotting start position
circles(startPos(1),startPos(2),2, 'facecolor', 'green')

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
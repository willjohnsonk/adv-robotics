%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% ECE8743 Advanced Robotics
% Visibility Graph based robot global path planning for static obstacles
% Wm. Peyton Johnson
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clc
close all
clear

%% Define the key graph points
start = [4,4];
goal = [90,85];

% Variables for clarity
n = 9;      % Number of obstacles
v = 38;     % Number of vertices (including start and end)

% Define the polyshape object with an x and y array
rect1 = polyshape([20 20 40 40], [10 30 30 10]);
rect2 = polyshape([50 50 65 65], [5 25 25 5]);
rect3 = polyshape([70 70 90 90], [10 30 30 10]);
rect4 = polyshape([10 10 30 30], [40 60 60 40]);
rect5 = polyshape([40 40 70 70], [40 60 60 40]);
rect6 = polyshape([80 80 95 95], [35 75 75 35]);
rect7 = polyshape([10 10 15 15], [70 90 90 70]);
rect8 = polyshape([20 20 40 40], [70 90 90 70]);
rect9 = polyshape([50 50 70 70], [65 90 90 65]);

% Helper arrays for building edges
obs_array = [rect1, rect2, rect3, rect4, rect5, rect6, rect7, rect8, rect9];
vertex_array = [start; [20,10]; [20,30]; [40,30]; [40,10]; [50,5]; [50,25]; [65,25]; [65,5]; [70,10]; [70,30]; [90,30]; [90,10]; [10,40]; [10,60]; [30,60]; [30,40]; [40,40]; [40,60]; [75,60]; [75,40]; [80,35]; [80,75]; [95,75]; [95,35]; [10,70]; [10,90]; [15,90]; [15,70]; [20,70]; [20,90]; [40,90]; [40,70]; [50,65]; [50,90]; [70,90]; [70,65]; goal];
adj_matrix = zeros(v);  % Will be used to build the graph G

%% Determining edges
tic

% Store each edge as [[x1,y1], [x2,y2]] using vertcat
edge_array = [];

% Loop for adding each edge of a given obstacle as a valid edge for the VG
for p = 1:n
   for verts = 1:length(obs_array(p).Vertices)-1
       per_edge = [obs_array(p).Vertices(verts,:), obs_array(p).Vertices(verts+1,:)];
       edge_array = vertcat(edge_array, per_edge);
   end
       % Catches the last edge so the loop doesn't go out of range   
       per_edge = [obs_array(p).Vertices(1,:), obs_array(p).Vertices(4,:)];
       edge_array = vertcat(edge_array, per_edge);

end

% Loop for every vertex against every other vertex
for i = 1:v
    for j = 1:v
        % Polyshape function that returns values when a line segment
        % intersects a polyshape            
        [in1, out1] = intersect(rect1, [vertex_array(i, :); vertex_array(j, :)]);
        [in2, out2] = intersect(rect2, [vertex_array(i, :); vertex_array(j, :)]);
        [in3, out3] = intersect(rect3, [vertex_array(i, :); vertex_array(j, :)]);
        [in4, out4] = intersect(rect4, [vertex_array(i, :); vertex_array(j, :)]);
        [in5, out5] = intersect(rect5, [vertex_array(i, :); vertex_array(j, :)]);
        [in6, out6] = intersect(rect6, [vertex_array(i, :); vertex_array(j, :)]);
        [in7, out7] = intersect(rect7, [vertex_array(i, :); vertex_array(j, :)]);
        [in8, out8] = intersect(rect8, [vertex_array(i, :); vertex_array(j, :)]);
        [in9, out9] = intersect(rect9, [vertex_array(i, :); vertex_array(j, :)]);
        
        % If a particular edge does not intersect, add to VG        
        if isempty(in1) && isempty(in2) && isempty(in3) && isempty(in4) && isempty(in5) && isempty(in6) && isempty(in7) && isempty(in8) && isempty(in9)
            if (vertex_array(i, :) ~= vertex_array(j, :))
                
                edge = [vertex_array(i, :), vertex_array(j, :)];
                edge_array = vertcat(edge_array, edge);
                % Additionally, add the weight to the adj matrix for G                  
                adj_matrix(i,j) = sqrt( (vertex_array(i, 1) - vertex_array(j, 1))^2  +  (vertex_array(i, 2) - vertex_array(j, 2))^2);
                
            end
        end
    end
end


%% Calculating the shortest path
G = graph(adj_matrix);

path = shortestpath(G, 1, 38);
pathSize = size(path);

totalDis = 0;

% Loop for calculating the euclidean distance of the shortest path
for p=1:pathSize(2)-1
    
    n1 = path(p);
    n2 = path(p+1);
    
    totalDis = totalDis + (sqrt( (vertex_array(n1, 1) - vertex_array(n2, 1))^2  +  (vertex_array(n1, 2) - vertex_array(n2, 2))^2));
    
end

toc
totalDis


%% Graphing
axis([0 100 0 100])
axis square

plot(rect1)
hold on
plot(rect2)
hold on
plot(rect3)
hold on
plot(rect4)
hold on
plot(rect5)
hold on
plot(rect6)
hold on
plot(rect7)
hold on
plot(rect8)
hold on
plot(rect9)

% Plotting start and goal points
circles(start(1), start(2),2, 'facecolor','green')
circles(goal(1), goal(2),2, 'facecolor','yellow')

% Plots all edges in the final VG
for i= 1:length(edge_array)
    
    xs = [edge_array(i, 1), edge_array(i, 3)];
    ys = [edge_array(i, 2), edge_array(i, 4)];
    
    hold on
    plot(xs, ys, 'b')
    
end

% Plotting the shortest path as bolded edges
for p=1:pathSize(2)-1
    
    n1 = path(p);
    n2 = path(p+1);
    
    xpts = [vertex_array(n1, 1), vertex_array(n2, 1)];
    ypts = [vertex_array(n1, 2), vertex_array(n2, 2)];
    
    hold on
    plot(xpts, ypts, 'k', 'LineWidth', 3)
    title('Visibility Graph')
    
end
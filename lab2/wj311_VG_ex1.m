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
start = [0.5,15.5];
goal = [15.5,0.5];

% Variables for clarity
n = 7;      % Number of obstacles
v = 30;     % Number of vertices (including start and end)

% Define the polyshape object with an x and y array
rect1 = polyshape([5 5 9 9], [1 2 2 1]);
rect2 = polyshape([11 11 13 13], [0 8 8 0]);
rect3 = polyshape([1 1 4 4], [3 6 6 3]);
rect4 = polyshape([7 7 9 9], [6 10 10 6]);
rect5 = polyshape([2 2 4 4], [8 16 16 8]);
rect6 = polyshape([8 8 9 9], [13 16 16 13]);
rect7 = polyshape([12 12 15 15], [11 14 14 11]);

% Helper arrays for building edges
obs_array = [rect1, rect2, rect3, rect4, rect5, rect6, rect7];
vertex_array = [start; [5,1]; [5,2]; [9,2]; [9,1]; [11,0]; [11,8]; [13,8]; [13,0]; [1,3]; [1,6]; [4,6]; [4,3]; [7,6]; [7,10]; [9,10]; [9,6]; [2,8]; [2,16]; [4,16]; [4,8]; [8,13]; [8,16]; [9,16]; [9,13]; [12,11]; [12,14]; [15,14]; [15,11]; goal];
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

        % If a particular edge does not intersect, add to VG        
        if isempty(in1) && isempty(in2) && isempty(in3) && isempty(in4) && isempty(in5) && isempty(in6) && isempty(in7)
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

path = shortestpath(G, 1, 30);
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
axis([0 16 0 16])
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

% Plotting start and goal points
circles(start(1), start(2),0.5, 'facecolor','green')
circles(goal(1), goal(2),0.5, 'facecolor','yellow')

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
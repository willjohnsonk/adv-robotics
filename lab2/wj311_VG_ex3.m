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
start = [8,85];
goal = [95,5];

% Variables for clarity
n = 5;      % Number of obstacles
v = 64;     % Number of vertices (including start and end)

% Define the polyshape object with an x and y array
rect1 = polyshape([4 4 8 12 16 20 24 28 32 32 28 24 16 12 8], [64 68 76 80 84 88 88 84 80 68 64 60 56 56 60]);
rect2 = polyshape([36 36 40 48 52 60 64 64 60 52 44 40], [64 68 80 84 84 80 76 72 64 60 56 56]);
rect3 = polyshape([76 76 80 88 92 92 80], [80 84 92 92 88 76 76]);
rect4 = polyshape([64 64 68 72 76 84 88 92 92 88 84 72], [48 56 60 64 68 72 72 64 60 48 44 44]);
rect5 = polyshape([64 60 56 36 32 32 36 56 64 68 88 88 80 80 88 88], [8 8 12 16 20 32 36 36 32 28 24 20 20 16 16 12]);

% Helper arrays for building edges
obs_array = [rect1, rect2, rect3, rect4, rect5];
vertex_array = [start; [4,64]; [4,68]; [8,76]; [12,80]; [16,84]; [20,88]; [24,88]; [28,84]; [32,80]; [32,68]; [28,64]; [24,60]; [16,56]; [12,56]; [8,60]; [36,64]; [36,68]; [40,80]; [48,84]; [52,84]; [60,80]; [64,76]; [60,72]; [60,64]; [52,60]; [44,56]; [40,56]; [76,80]; [76,84]; [80,92]; [88,92]; [92,88]; [92,76]; [80,76]; [64,48]; [64,56]; [68,60]; [72,64]; [76,68]; [84,72]; [88,72]; [92,64]; [92,60]; [88,48]; [84,44]; [72,44]; [64,8]; [60,8]; [56,12]; [36,16]; [32,20]; [32,32]; [36,36]; [56,36]; [64,32]; [68,28]; [88,24]; [88,20]; [80,20]; [80,16]; [88,16]; [88,12]; goal];

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
      
       % Catches the last edge so the loop doesn't go out of range       
       if verts == length(obs_array(p).Vertices)-1
            per_edge = [obs_array(p).Vertices(1,:), obs_array(p).Vertices(length(obs_array(p).Vertices),:)];
            edge_array = vertcat(edge_array, per_edge);
       end
   end
   


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

        % If a particular edge does not intersect, add to VG        
        if isempty(in1) && isempty(in2) && isempty(in3) && isempty(in4) && isempty(in5)
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

path = shortestpath(G, 1, 64);
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
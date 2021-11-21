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

% Obstacles
n = 2;
v = 10;

rect1 = polyshape([20 20 60 60], [10 40 40 10]);
rect2 = polyshape([50 50 70 70], [60 80 80 60]);

obs_array = [rect1, rect2];
vertex_array = [[4,4]; [20,10]; [20,40]; [60,40]; [60,10]; [50,60]; [50,80]; [70,80]; [70,60]; [90,85]];
adj_matrix = zeros(v);

%% Determining edges
tic

% Store each edge as [[x1,y1], [x2,y2]] using vertcat
edge_array = [];

for p = 1:n
   for verts = 1:length(obs_array(p).Vertices)-1
       per_edge = [obs_array(p).Vertices(verts,:), obs_array(p).Vertices(verts+1,:)];
       edge_array = vertcat(edge_array, per_edge);
   end
       per_edge = [obs_array(p).Vertices(1,:), obs_array(p).Vertices(4,:)];
       edge_array = vertcat(edge_array, per_edge);

end


for i = 1:v
    for j = 1:v
        
        [in1, out1] = intersect(rect1, [vertex_array(i, :); vertex_array(j, :)]);
        [in2, out2] = intersect(rect2, [vertex_array(i, :); vertex_array(j, :)]);
        
        if isempty(in1) && isempty(in2)
            if (vertex_array(i, :) ~= vertex_array(j, :))
                
                edge = [vertex_array(i, :), vertex_array(j, :)];
                edge_array = vertcat(edge_array, edge);
                
                adj_matrix(i,j) = sqrt( (vertex_array(i, 1) - vertex_array(j, 1))^2  +  (vertex_array(i, 2) - vertex_array(j, 2))^2);
                
            end
        end
    end
end


%% Calculating the shortest path
G = graph(adj_matrix);

path = shortestpath(G, 1, 10);
pathSize = size(path);

totalDis = 0;

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

% Plotting start and goal points
circles(start(1), start(2),2, 'facecolor','green')
circles(goal(1), goal(2),2, 'facecolor','yellow')

for i= 1:length(edge_array)
    
    xs = [edge_array(i, 1), edge_array(i, 3)];
    ys = [edge_array(i, 2), edge_array(i, 4)];
    
    hold on
    plot(xs, ys, 'b')
    
end
for p=1:pathSize(2)-1
    
    n1 = path(p);
    n2 = path(p+1);
    
    xpts = [vertex_array(n1, 1), vertex_array(n2, 1)];
    ypts = [vertex_array(n1, 2), vertex_array(n2, 2)];
    
    hold on
    plot(xpts, ypts, 'k', 'LineWidth', 3)
    title('Visibility Graph')
    
end
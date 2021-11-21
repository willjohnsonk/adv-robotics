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
start = [180, 180];
goal = [1800, 1800];

% Variables for clarity
n = 15;      % Number of obstacles
v = 67;     % Number of vertices (including start and end)

% Define the polyshape object with an x and y array
rect1 = polyshape([0 220 380 240], [580 700 580 430]);
rect2 = polyshape([160 180 300 300], [810 1100 1140 780]);
rect3 = polyshape([160 120 240 400 300], [1240 1420 1800 1500 1200]);
rect4 = polyshape([420 380 500 900], [20 200 380 20]);
rect5 = polyshape([580 580 700 820 630], [500 600 600 500 380]);
rect6 = polyshape([440 500 600 580], [840 1100 1200 780]);
rect7 = polyshape([560 620 780 840], [1780 1940 1920 1620]);
rect8 = polyshape([920 900 1080 1200], [460 620 630 500]);
rect9 = polyshape([900 820 920 1060 1080], [920 1260 1280 1190 900]);
rect10 = polyshape([1340 1260 1280 1360 1420], [380 660 740 720 520]);
rect11 = polyshape([1210 1180 1320 1480], [940 1200 1260 1040]);
rect12 = polyshape([1180 1100 1160 1380 1300], [1580 1820 1840 1800 1570]);
rect13 = polyshape([1640 1480 1600 1720], [700 880 920 760]);
rect14 = polyshape([1610 1680 1980 1880], [1020 1280 1200 960]);
rect15 = polyshape([1460 1400 1460 1610], [1300 1660 1720 1420]);

% Helper arrays for building edges
obs_array = [rect1, rect2, rect3, rect4, rect5, rect6, rect7, rect8, rect9, rect10, rect11, rect12, rect13, rect14, rect15];
vertex_array = [start; [0,580]; [220,700]; [380,580]; [240,430]; [160,810]; [180,1100]; [300,1140]; [300, 780]; [160,1240]; [120,1420]; [240,1800]; [400,1500]; [300,1200]; [420,20]; [380,200]; [500,380]; [900,20]; [580,500]; [580,600]; [700,600]; [820,500]; [630,380]; [440,840]; [500,1100]; [600,1200]; [580,780]; [560,1780]; [620,1940]; [780,1920]; [840,1620]; [920,460]; [900,620]; [1080,630]; [1200,500]; [900,920]; [820,1260]; [920,1280]; [1060,1190]; [1080,900]; [1340,380]; [1260,660]; [1280,740]; [1360,720]; [1420,520]; [1210,940]; [1180,1200]; [1320,1260]; [1480,1040]; [1180,1580]; [1100,1820]; [1160,1840]; [1380,1800]; [1300,1570]; [1640,700]; [1480,880]; [1600,920]; [1720,760]; [1610,1020]; [1680,1280]; [1980,1200]; [1880,960]; [1460,1300]; [1400,1660]; [1460,1420]; [1610,1420]; goal];

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
   if length(obs_array(p).Vertices) == 4
       per_edge = [obs_array(p).Vertices(1,:), obs_array(p).Vertices(4,:)];
       edge_array = vertcat(edge_array, per_edge);
   end

   if length(obs_array(p).Vertices) == 5
       per_edge = [obs_array(p).Vertices(1,:), obs_array(p).Vertices(5,:)];
       edge_array = vertcat(edge_array, per_edge);
   end
   
end


% Loop for every vertex against every other vertex
for i = 1:v
    for j = 1:v
        % Polyshape function that returns values when a line segment
        % intersects a polyshape            
        [in1] = intersect(rect1, [vertex_array(i, :); vertex_array(j, :)]);
        [in2] = intersect(rect2, [vertex_array(i, :); vertex_array(j, :)]);
        [in3] = intersect(rect3, [vertex_array(i, :); vertex_array(j, :)]);
        [in4] = intersect(rect4, [vertex_array(i, :); vertex_array(j, :)]);
        [in5] = intersect(rect5, [vertex_array(i, :); vertex_array(j, :)]);
        [in6] = intersect(rect6, [vertex_array(i, :); vertex_array(j, :)]);
        [in7] = intersect(rect7, [vertex_array(i, :); vertex_array(j, :)]);
        [in8] = intersect(rect8, [vertex_array(i, :); vertex_array(j, :)]);
        [in9] = intersect(rect9, [vertex_array(i, :); vertex_array(j, :)]);
        [in10] = intersect(rect10, [vertex_array(i, :); vertex_array(j, :)]);
        [in11] = intersect(rect11, [vertex_array(i, :); vertex_array(j, :)]);
        [in12] = intersect(rect12, [vertex_array(i, :); vertex_array(j, :)]);
        [in13] = intersect(rect13, [vertex_array(i, :); vertex_array(j, :)]);
        [in14] = intersect(rect14, [vertex_array(i, :); vertex_array(j, :)]);
        [in15] = intersect(rect15, [vertex_array(i, :); vertex_array(j, :)]);
        
        % If a particular edge does not intersect, add to VG        
        if isempty(in1) && isempty(in2) && isempty(in3) && isempty(in4) && isempty(in5) && isempty(in6) && isempty(in7) && isempty(in8) && isempty(in9) && isempty(in10) && isempty(in11) && isempty(in12) && isempty(in13) && isempty(in14) && isempty(in15)
            if (vertex_array(i, :) ~= vertex_array(j, :))
                
                edge = [vertex_array(i, :), vertex_array(j, :)];
                weight = sqrt( (vertex_array(i, 1) - vertex_array(j, 1))^2  +  (vertex_array(i, 2) - vertex_array(j, 2))^2);
                
                edge_array = vertcat(edge_array, edge);
                adj_matrix(i,j) = weight;

            end
        end
    end
end


% compare iterated values against row values in the adj matrix (i, :) using
% percent difference?
% if a value in the row is within a certain threshold of weight, trash
% else, add to edge_array

% Calculating the shortest path
G = graph(adj_matrix);

path = shortestpath(G, 1, 67);
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
axis([0 2000 0 2000])
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
hold on
plot(rect10)
hold on
plot(rect11)
hold on
plot(rect12)
hold on
plot(rect13)
hold on
plot(rect14)
hold on
plot(rect15)

% Plotting start and goal points
circles(start(1), start(2),20, 'facecolor','green')
circles(goal(1), goal(2),20, 'facecolor','yellow')

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
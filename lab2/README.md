# Running the code

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% ECE8743 Advanced Robotics
% Visibility Graph based robot global path planning for static obstacles
% Wm. Peyton Johnson
%
% Configued in Environment 1/2/3
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%

Note (Original Scripts):
- other files are helper files for the above main scripts
- To switch between the three environment configurations, comment the others in visible_graph.m (well documented and commented code)
- ECE8743_Visible_graph_Environment_1.m for Environment 1
- ECE8743_Visible_graph_Environment_2.m for Environment 2
- ECE8743_Visible_graph_Environment_3.m for Environment 3
- These three environments of files can be revsied from ECE8743_Visible_graph.m

Note (UPDATED):
- No major edits were made to the given files, instead, I rewrote my own
- The modified Environment 1,2,3 can be run as expected, with the respective file names:
	- wj311_VG_env_1.m
	- wj311_VG_env_2.m
	- wj311_VG_env_3.m
	- Where environment 1 is the most general example to build from
- The additional maps 1,2,3 were also added as individual files as follows:
	- wj311_VG_ex1.m
	- wj311_VG_ex2.m
	- wj311_VG_ex3.m
- Each file can be run independently with no changes to the code.
- If you have issues with graphs appearing, restart Matlab
- visibility_graph.m is semi-legacy and is mostly identical to wj311_VG_env_1.m
- EuclDist.m is not needed for the modified code

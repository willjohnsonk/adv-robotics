# Running the code

1. Navigate to the codes folder
2. Run the "PotentialFields.m" file for 1 obstacle environment and "PotentialFields_Q2.m" for 2 obstacle environment.

Note: other files are helper files for the above main scripts

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

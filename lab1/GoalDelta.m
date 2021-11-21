function [delXG, delYG] = GoalDelta(vx, vy, gx, gy, goalR, goalS, alpha)
% This function gives delX, delY of attraction caused by the goal point
% vx:       Given coordinate X
% vy:       Given coordinate Y
% gx:       X value of goal
% gy:       Y value of goal
% goalR:    Radius of goal
% goalS:    Spread of goal attraction
% alpha:    Attraction strength

dGoal = sqrt((gx-vx)^2 + (gy-vy)^2); % distance between goal and current position

thetaG = atan2((gy-vy),(gx-vx));     % angle between goal and current position


if(dGoal < goalR)                                          % If the distance is less than the radius, end
    delXG = 0; delYG = 0;
    
elseif(((goalS + goalR) >= dGoal) && (dGoal >= goalR))   % If the potential field of the goal affects the particle
    delXG = alpha*(dGoal - goalR)*cos(thetaG);          % and the distance is greater than the goal radius calculate
    delYG = alpha*(dGoal - goalR)*sin(thetaG);          % the attraction to the goal

else
    delXG = alpha*goalS*cos(thetaG);                    % When the goal isn't affecting the particle, head towards it
    delYG = alpha*goalS*sin(thetaG);
end

end



% % Variation code for part 7
% K = alpha;
% A = 1;
% dGoal = sqrt((gx-vx)^2 + (gy-vy)^2); % distance between goal and current position
% thetaG = atan2((gy-vy),(gx-vx));     % angle between goal and current position
% 
% 
% if(dGoal < goalR)                                          % If the distance is less than the radius, end
%     delXG = 0; delYG = 0;
%     
% elseif(((goalS + goalR) >= dGoal) && (dGoal >= goalR))   % If the potential field of the goal affects the particle
%     delXG = K * ((dGoal)^A) * cos(thetaG);
%     delYG = K * ((dGoal)^A) * sin(thetaG);    
% else
%     delXG = K*goalS*cos(thetaG);                    % When the goal isn't affecting the particle, head towards it
%     delYG = K*goalS*sin(thetaG);
% end
% 
% end
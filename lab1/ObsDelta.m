function [delXO, delYO] = ObsDelta(vx, vy, ox, oy, obsRad, obsS, beta)
% This function gives delX, delY of repulsion caused by the obstacle
% vx:       Given coordinate X
% vy:       Given coordinate Y
% ox:       X value of the object
% oy:       Y value of the object
% obsRad:   Radius of the object
% obsS:     Spread of obstacle repulsion
% beta:     Repulsion strength

inf = 10;
dObs = sqrt((ox-vx)^2 + (oy-vy)^2);     % distance bw obstacle and current position
thetaO = atan2((oy-vy),(ox-vx));        % angle between goal and current position

if dObs<obsRad                                          % If the particle is projected to be in the object
    delXO = -(sign(cos(thetaO)))*inf;                   % go back by given value inf
    delYO = -(sign(sin(thetaO)))*inf;
    
elseif ((dObs < (obsS + obsRad)) && (dObs>=obsRad))     % If the particle is inside the potential field effect
    delXO = -beta*(obsS + obsRad - dObs)*cos(thetaO);   % and outside or on the boundry of the object, find the
    delYO = -beta*(obsS + obsRad - dObs)*sin(thetaO);   % repulsion on the particle
    
else
    delXO = 0;      % If outside an object's range, do nothing
    delYO = 0;
end

end



% % Variation code for part 7
% inf = 1;
% dObs = sqrt((ox-vx)^2 + (oy-vy)^2);     % distance bw obstacle and current position
% thetaO = atan2((oy-vy),(ox-vx));        % angle between goal and current position
% 
% L = beta;
% B = 2;
% 
% if(dObs < obsRad)                                        % If the particle is projected to be in the object
%     delXO = -(sign(cos(thetaO)))*inf;                   % go back by given value inf
%     delYO = -(sign(sin(thetaO)))*inf;
%     
% elseif ((dObs <= (obsS + obsRad)) && (dObs>=obsRad))     % If the particle is inside the potential field effect
%     delXO = -L * (((obsS + obsRad) / dObs)^B) * cos(thetaO); % Calculate the x and y repulsion
%     delYO = -L * (((obsS + obsRad) / dObs)^B) * sin(thetaO);
%     
% else
%     delXO = 0;      % If outside an object's range, do nothing
%     delYO = 0;
% end
% 
% end
function [value, isterminal, direction] = CrashEvent(T,X,Environnement)
%   Stop simulation at apogee
% %map_pos=abs(Environnement.map_x-2648540-X(1))< 2 & abs(Environnement.map_y-X(2))< 2;
% 
% [mapPosRow, mapPosCol]=find(abs(Environnement.map_x-X(1))< 2 & abs(Environnement.map_y-X(2))< 2);
% 
% %map_z=find(Environnement.map_z.*map_pos~=0);
% %if (X(1)<min(Environnement.map_x) | X(1)>max(Environnement.map_x) | X(2)<min(Environnement.map_y) | X(2)>max(Environnement.map_y))%| isempty(mapPosRow)
%     %disp('yes'); 
% if(isempty(mapPosRow))
%     map_z=Environnement.startAltitude;
% else 
%      %disp('reussi');
%     map_z=Environnement.map_z(mapPosRow(1),mapPosCol(1));
% end
z=X(3)-findAltitude(X(1),X(2),Environnement);

value = (z>0)-0.5;   % Rocket reaches the same altitude as where it was launched
isterminal = 1; % Stop the integration
direction = -1; % detect descending values
end

%[mapPosRow, mapPosCol]=find(abs(Environnement.map_x-X(1))< 2 & abs(Environnement.map_y-X(2))< 2)

% if (X(1)<min(Environnement.map_x) | X(1)>max(Environnement.map_x) | X(2)<min(Environnement.map_y) | X(2)>max(Environnement.map_y)| isempty(mapPosRow))
%     map_z=Environnement.startAltitude;
% else 
%     map_z=Environnement.map_z(mapPosRow(1),mapPosCol(1));
% end
% 
% z=X(3)-map_z-Environnement.startAltitude;
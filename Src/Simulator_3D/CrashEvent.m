function [value, isterminal, direction] = CrashEvent(T,X,Environnement)
%   Stop simulation at apogee
% %map_pos=abs(Environnement.map_x-2648540-X(1))< 2 & abs(Environnement.map_y-X(2))< 2;
% 
% [map_pos_row, map_pos_col]=find(abs(Environnement.map_x-X(1))< 2 & abs(Environnement.map_y-X(2))< 2);
% 
% %map_z=find(Environnement.map_z.*map_pos~=0);
% %if (X(1)<min(Environnement.map_x) | X(1)>max(Environnement.map_x) | X(2)<min(Environnement.map_y) | X(2)>max(Environnement.map_y))%| isempty(map_pos_row)
%     %disp('yes'); 
% if(isempty(map_pos_row))
%     map_z=Environnement.Start_Altitude;
% else 
%      %disp('reussi');
%     map_z=Environnement.map_z(map_pos_row(1),map_pos_col(1));
% end
z=X(3)-find_altitude(X(1),X(2),Environnement);

value = (z>0)-0.5;   % Rocket reaches the same altitude as where it was launched
isterminal = 1; % Stop the integration
direction = -1; % detect descending values
end

%[map_pos_row, map_pos_col]=find(abs(Environnement.map_x-X(1))< 2 & abs(Environnement.map_y-X(2))< 2)

% if (X(1)<min(Environnement.map_x) | X(1)>max(Environnement.map_x) | X(2)<min(Environnement.map_y) | X(2)>max(Environnement.map_y)| isempty(map_pos_row))
%     map_z=Environnement.Start_Altitude;
% else 
%     map_z=Environnement.map_z(map_pos_row(1),map_pos_col(1));
% end
% 
% z=X(3)-map_z-Environnement.Start_Altitude;
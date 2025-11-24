function value=findAltitude(X,Y,Environnement)
% Arguments:
% - X: an array of x coordinates;
% typically, will just be a 1x1
% - Y: an array of y coordinates
% typically will just be a 1x1
% - Environnement: output of environnementReader.m

% Output:
% - value: an array of z coordinates of the same shape as X,
%   corresponding to the altitude of the terrain at the specified
%   x,y coordinates (from X,Y), relative to Environnement.Start_Altitude

% X and Y should be of the same dimensions; if not, issue a warning
% our code will still work if Y has more elements than X,
% but if the opposite is true, we truncate X
if numel(X) ~= numel(Y)
    warning('X and Y do not have the same number of elements');
end
if numel(X) > numel(Y)
    X = X(1:numel(Y))
end

% initialize our to-be-returned value as a vector with the same
% dimensions as X, but filled with zeros
Z=0*X;

% iterate over X; we can now assume X and Y have the same number of elements
for i=1:numel(X)
    % find the [row,col] indices of the positions on the map close to [X(i),Y(i)]
    [mapPosRow, mapPosCol]=find(abs(Environnement.map_x-X(i))< 2 & abs(Environnement.map_y-Y(i))< 2);

    if(isempty(mapPosRow))
        % if no indices were found, assume the altitude is
        % the same as Environnement.Start_Altitude
        map_z=Environnement.Start_Altitude;
    else 
        % if indices were found, set the altitude to the
        % altitude specified in the map by the first [row,col] index pair
        map_z=Environnement.map_z(mapPosRow(1),mapPosCol(1));
    end

    % subtract Environnement.Start_Altitude to get the relative altitude
    Z(i)=map_z-Environnement.Start_Altitude;
end
value=Z;
end
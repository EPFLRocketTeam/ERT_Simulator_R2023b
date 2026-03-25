function dragCoefficient = dragOpenRocket(dragData, interpolationType, time, altitude, speed)
% Interpolates the drag coefficient from OpenRocket simulation data
%
% Inputs:
%   dragData  - Nx4 matrix [time, altitude, speed, drag_coefficient]
%   interpolationType - string: 'time', 'altitude', or 'speed'
%   time       - time value for interpolation (if using time interpolation)
%   altitude   - altitude value for interpolation (if using altitude interpolation)  
%   speed      - speed value for interpolation (if using speed interpolation)
%
% Output:
%   dragCoefficient - drag coefficient at specified condition

    % Input validation
    if nargin < 5
        error('dragOpenRocket:InsufficientInputs', 'All 5 input arguments are required');
    end
    
    if isempty(dragData)
        error('dragOpenRocket:EmptyData', 'Drag data cannot be empty');
    end
    
    if size(dragData, 2) < 4
        error('dragOpenRocket:InvalidDataSize', 'Drag data must have at least 4 columns');
    end
    
    % Remove any rows with NaN values
    dragData = dragData(all(~isnan(dragData), 2), :);
    
    if size(dragData, 1) < 2
        error('dragOpenRocket:InsufficientData', 'At least 2 valid data points are required for interpolation');
    end
    
    % Convert interpolation type to lowercase for case-insensitive matching
    interpolationType = lower(interpolationType);
    
    try
        if contains(interpolationType, 'time')
            % Time-based interpolation
            xData = dragData(:, 1);
            yData = dragData(:, 4);
            xQuery = time;
            
        elseif contains(interpolationType, 'altitude') 
            % Altitude-based interpolation
            xData = dragData(:, 2);
            yData = dragData(:, 4);
            xQuery = altitude;
            
        elseif contains(interpolationType, 'speed')
            % Speed-based interpolation
            xData = dragData(:, 3);
            yData = dragData(:, 4);
            xQuery = speed;
            
        else
            error('drag_OR:InvalidInterpType', ...
                'Interpolation type must be ''time'', ''altitude'', or ''speed''. Got: ''%s''', interpolationType);
        end
        
        % Remove duplicate x values to avoid polyfit issues
        [xUnique, idx] = unique(xData, 'stable');
        yUnique = yData(idx);
        
        % Use appropriate polynomial degree based on data size and distribution
        nbPoints = length(xUnique);
        if nbPoints < 2
            error('drag_OR:InsufficientUniqueData', ...
                'Insufficient unique data points for interpolation. Need at least 2 unique %s values, got %d.', ...
                interpolationType, nbPoints);
        end
        
        % Check if we're extrapolating far outside the data range
        xMin = min(xUnique);
        xMax = max(xUnique);
        is_extrapolation = xQuery < xMin || xQuery > xMax;
        
        if is_extrapolation
            % For extrapolation, use more conservative methods
            extrapolation_distance = max((xMin - xQuery) / (xMax - xMin), ...
                                        (xQuery - xMax) / (xMax - xMin));
            
            if extrapolation_distance > 2.0  % More than 2x the data range
                % Very far extrapolation - use nearest neighbor
                if xQuery < xMin
                    dragCoefficient = yUnique(1);
                else
                    dragCoefficient = yUnique(end);
                end
                warning('drag_OR:FarExtrapolation', ...
                    'Far extrapolation detected (%.1f%% beyond data range). Using nearest data point.', ...
                    (extrapolation_distance - 1) * 100);
            else
                % Moderate extrapolation - use linear extrapolation
                dragCoefficient = interp1(xUnique, yUnique, xQuery, 'linear', 'extrap');
                warning('drag_OR:ModerateExtrapolation', ...
                    'Moderate extrapolation detected. Using linear extrapolation.');
            end
        else
            % Interpolation within data range - use polynomial fit
            maxDegree = determineOptimalDegree(xUnique, nbPoints);
            
            if maxDegree < 2
                % Use linear interpolation for very small datasets
                dragCoefficient = interp1(xUnique, yUnique, xQuery, 'linear');
            else
                % Try polynomial fit with centering and scaling
                [ft_drag, S, mu] = polyfit(xUnique, yUnique, maxDegree);
                dragCoefficient = polyval(ft_drag, xQuery, S, mu);
            end
        end
        
        % Ensure drag coefficient is physically reasonable
        if dragCoefficient < 0
            warning('dragOpenRocket:NegativedragCoefficient', ...
                'Computed negative drag coefficient (dragCoefficient = %.4f). Clipping to 0.', dragCoefficient);
            dragCoefficient = 0;
        elseif dragCoefficient > 2  % Reduced from 5 to 2 for more realistic bounds
            warning('dragOpenRocket:LargedragCoefficient', ...
                'Computed unusually large drag coefficient (dragCoefficient = %.4f). Clipping to reasonable maximum.', dragCoefficient);
            dragCoefficient = min(dragCoefficient, 2); % Clip to maximum reasonable value
        end
        
    catch ME
        % If polynomial fitting fails, fall back to linear interpolation
        if any(strcmp(ME.identifier, {'MATLAB:polyfit:XYSizeMismatch', ...
                                     'MATLAB:polyfit:NotEnoughPoints', ...
                                     'MATLAB:polyfit:PolyfitNotUnique'}))
            try
                warning('dragOpenRocket:PolyfitFailed', ...
                    'Polynomial fit failed, using linear interpolation instead. Error: %s', ME.message);
                dragCoefficient = interp1(xUnique, yUnique, xQuery, 'linear', 'extrap');
            catch
                rethrow(ME);
            end
        else
            rethrow(ME);
        end
    end
end

function maxDegree = determineOptimalDegree(xData, nbPoints)
    % Determine the optimal polynomial degree to avoid overfitting and numerical issues
    
    % Maximum allowed degree based on data size
    absoluteMax = min(3, nbPoints - 1); % Further reduced from 4 to 3
    
    if nbPoints <= 2
        maxDegree = 1;
        return;
    end
    
    % Check data distribution - if data is very non-uniform, use lower degree
    xSorted = sort(xData);
    xDiff = diff(xSorted);
    uniformityRatio = std(xDiff) / mean(xDiff);
    
    % If data is very non-uniform, use lower degree
    if uniformityRatio > 2.0
        maxDegree = min(1, absoluteMax); % Use linear
    elseif uniformityRatio > 1.0
        maxDegree = min(2, absoluteMax);
    else
        maxDegree = absoluteMax;
    end
    
    % For very small datasets, further limit the degree
    if nbPoints < 8
        maxDegree = min(2, maxDegree);
    end
    
    % Ensure at least linear interpolation
    maxDegree = max(1, maxDegree);
end
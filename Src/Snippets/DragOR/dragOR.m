function CD = dragOR(drag_data, interp_type, time, altitude, speed)
% Interpolates the drag coefficient from OpenRocket simulation data
%
% Inputs:
%   drag_data  - Nx4 matrix [time, altitude, speed, drag_coefficient]
%   interp_type - string: 'time', 'altitude', or 'speed'
%   time       - time value for interpolation (if using time interpolation)
%   altitude   - altitude value for interpolation (if using altitude interpolation)  
%   speed      - speed value for interpolation (if using speed interpolation)
%
% Output:
%   CD - drag coefficient at specified condition

    % Input validation
    if nargin < 5
        error('drag_OR:InsufficientInputs', 'All 5 input arguments are required');
    end
    
    if isempty(drag_data)
        error('drag_OR:EmptyData', 'Drag data cannot be empty');
    end
    
    if size(drag_data, 2) < 4
        error('drag_OR:InvalidDataSize', 'Drag data must have at least 4 columns');
    end
    
    % Remove any rows with NaN values
    drag_data = drag_data(all(~isnan(drag_data), 2), :);
    
    if size(drag_data, 1) < 2
        error('drag_OR:InsufficientData', 'At least 2 valid data points are required for interpolation');
    end
    
    % Convert interpolation type to lowercase for case-insensitive matching
    interp_type = lower(interp_type);
    
    try
        if contains(interp_type, 'time')
            % Time-based interpolation
            x_data = drag_data(:, 1);
            y_data = drag_data(:, 4);
            x_query = time;
            
        elseif contains(interp_type, 'altitude') 
            % Altitude-based interpolation
            x_data = drag_data(:, 2);
            y_data = drag_data(:, 4);
            x_query = altitude;
            
        elseif contains(interp_type, 'speed')
            % Speed-based interpolation
            x_data = drag_data(:, 3);
            y_data = drag_data(:, 4);
            x_query = speed;
            
        else
            error('drag_OR:InvalidInterpType', ...
                'Interpolation type must be ''time'', ''altitude'', or ''speed''. Got: ''%s''', interp_type);
        end
        
        % Remove duplicate x values to avoid polyfit issues
        [x_unique, idx] = unique(x_data, 'stable');
        y_unique = y_data(idx);
        
        % Use appropriate polynomial degree based on data size and distribution
        n_points = length(x_unique);
        if n_points < 2
            error('drag_OR:InsufficientUniqueData', ...
                'Insufficient unique data points for interpolation. Need at least 2 unique %s values, got %d.', ...
                interp_type, n_points);
        end
        
        % Check if we're extrapolating far outside the data range
        x_min = min(x_unique);
        x_max = max(x_unique);
        is_extrapolation = x_query < x_min || x_query > x_max;
        
        if is_extrapolation
            % For extrapolation, use more conservative methods
            extrapolation_distance = max((x_min - x_query) / (x_max - x_min), ...
                                        (x_query - x_max) / (x_max - x_min));
            
            if extrapolation_distance > 2.0  % More than 2x the data range
                % Very far extrapolation - use nearest neighbor
                if x_query < x_min
                    CD = y_unique(1);
                else
                    CD = y_unique(end);
                end
                warning('drag_OR:FarExtrapolation', ...
                    'Far extrapolation detected (%.1f%% beyond data range). Using nearest data point.', ...
                    (extrapolation_distance - 1) * 100);
            else
                % Moderate extrapolation - use linear extrapolation
                CD = interp1(x_unique, y_unique, x_query, 'linear', 'extrap');
                warning('drag_OR:ModerateExtrapolation', ...
                    'Moderate extrapolation detected. Using linear extrapolation.');
            end
        else
            % Interpolation within data range - use polynomial fit
            max_degree = determineOptimalDegree(x_unique, n_points);
            
            if max_degree < 2
                % Use linear interpolation for very small datasets
                CD = interp1(x_unique, y_unique, x_query, 'linear');
            else
                % Try polynomial fit with centering and scaling
                [ft_drag, S, mu] = polyfit(x_unique, y_unique, max_degree);
                CD = polyval(ft_drag, x_query, S, mu);
            end
        end
        
        % Ensure drag coefficient is physically reasonable
        if CD < 0
            warning('drag_OR:NegativeCD', ...
                'Computed negative drag coefficient (CD = %.4f). Clipping to 0.', CD);
            CD = 0;
        elseif CD > 2  % Reduced from 5 to 2 for more realistic bounds
            warning('drag_OR:LargeCD', ...
                'Computed unusually large drag coefficient (CD = %.4f). Clipping to reasonable maximum.', CD);
            CD = min(CD, 2); % Clip to maximum reasonable value
        end
        
    catch ME
        % If polynomial fitting fails, fall back to linear interpolation
        if any(strcmp(ME.identifier, {'MATLAB:polyfit:XYSizeMismatch', ...
                                     'MATLAB:polyfit:NotEnoughPoints', ...
                                     'MATLAB:polyfit:PolyfitNotUnique'}))
            try
                warning('drag_OR:PolyfitFailed', ...
                    'Polynomial fit failed, using linear interpolation instead. Error: %s', ME.message);
                CD = interp1(x_unique, y_unique, x_query, 'linear', 'extrap');
            catch
                rethrow(ME);
            end
        else
            rethrow(ME);
        end
    end
end

function max_degree = determineOptimalDegree(x_data, n_points)
    % Determine the optimal polynomial degree to avoid overfitting and numerical issues
    
    % Maximum allowed degree based on data size
    absolute_max = min(3, n_points - 1); % Further reduced from 4 to 3
    
    if n_points <= 2
        max_degree = 1;
        return;
    end
    
    % Check data distribution - if data is very non-uniform, use lower degree
    x_sorted = sort(x_data);
    x_diff = diff(x_sorted);
    uniformity_ratio = std(x_diff) / mean(x_diff);
    
    % If data is very non-uniform, use lower degree
    if uniformity_ratio > 2.0
        max_degree = min(1, absolute_max); % Use linear
    elseif uniformity_ratio > 1.0
        max_degree = min(2, absolute_max);
    else
        max_degree = absolute_max;
    end
    
    % For very small datasets, further limit the degree
    if n_points < 8
        max_degree = min(2, max_degree);
    end
    
    % Ensure at least linear interpolation
    max_degree = max(1, max_degree);
end
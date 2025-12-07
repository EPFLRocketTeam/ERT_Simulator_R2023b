function val = cp_sin(delta_c, M, cp_sin_arr)
            % index of Mach number
            i = round((M-1.02) / 0.02);
            % index of local cone semi-vertex angle
            j = int32(round(abs(delta_c / 0.2 *180/pi)+1));

            % Get array dimensions
            [max_i, max_j] = size(cp_sin_arr);
            
            % Bound checking
            i = min(max(i, 1), max_i);
            j = min(max(j, 1), max_j);
            
            val= cp_sin_arr(i,j);
end
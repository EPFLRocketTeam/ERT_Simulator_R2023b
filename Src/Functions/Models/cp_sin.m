function val = cp_sin(delta_c, M, cp_sin_arr)
            % index of Mach number
            i = round((M-1.02) / 0.02);
            % index of local cone semi-vertex angle
            j = int32(round(abs(delta_c / 0.2 *180/pi)+1));
            
            val= cp_sin_arr(i,j);
end
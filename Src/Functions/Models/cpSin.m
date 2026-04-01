function value = cpSin(semiVertexAngle, machNumber, cpSinArray)
            % index of Mach number
            i = round((machNumber-1) / 0.02);
            % index of local cone semi-vertex angle
            j = int32(round(abs(semiVertexAngle / 0.2 *180/pi)+1));

            % Clamp indices to array bounds
            i = max(1, min(i, size(cpSinArray,1)));
            j = max(1, min(j, size(cpSinArray,2)));

            value = cpSinArray(i,j);
end
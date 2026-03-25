function value = cpSin(semiVertexAngle, machNumber, cpSinArray)
            % index of Mach number
            i = round((machNumber-1) / 0.02);
            % index of local cone semi-vertex angle
            j = int32(round(abs(semiVertexAngle / 0.2 *180/pi)+1));

            value = cpSinArray(i,j);
end
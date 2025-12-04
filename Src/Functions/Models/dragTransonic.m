function dragCoefficient = dragTransonic(rocket, angleOfAttack, freestreamVelocity, kinematicViscosity, speedOfSound)
    % INPUTS:
    % - rocket              : Rocket object
    % - angleOfAttack       : Angle of attack [rad]
    % - freestreamVelocity  : Free stream velocity [m/s]
    % - kinematicViscosity  : Dynamic viscosity [m2/s]
    % - speedOfSound        : Speed of sound [m/s]
    % OUTPUTS:
    % - dragCoefficient     : Drag coefficient
    % REFERENCES:
    % - Drag Coefficient Prediction, Chapter 1 Preetam Sharma
    % Semester Project Report, Professor Flavio Noca, December 2021.

    % -------------------------------------------------------------------------
    % 0.1 Velocity validation
    % -------------------------------------------------------------------------
    if freestreamVelocity < 0.1
        freestreamVelocity = 0.1;
    end 
    
    % -------------------------------------------------------------------------
    % 1. Parameters
    % -------------------------------------------------------------------------

    machNumber = freestreamVelocity / speedOfSound;
    
    % Rocket geometry
    totalLength = rocket.stagePositions(end);
    noseLength = rocket.stagePositions(2);
    maxDiameter = rocket.maxDiameter;
    maxRadius = maxDiameter / 2;
    baseDiameter = rocket.stageDiameters(end);
    
    % Body lengths
    aftBodyLength = totalLength - noseLength;  % Length aft of maximum diameter position
    basePosition = totalLength;                % Base position
    effectiveLength = totalLength;             % Effective length of rocket
    
    % Fin geometry
    rootChord = rocket.finRootChord;
    tipChord = rocket.finTipChord;
    finCount = rocket.numFins;
    maxFinThickness = rocket.finThickness;
    maxThicknessDistance = rocket.finLeadingEdgeLength;      % Distance from fin leading edge to maximum thickness
    
    % Derived parameters
    bodyWettedArea = 2 * pi * maxDiameter / 2 * (totalLength - noseLength) + ...
                     pi * maxDiameter / 2 * sqrt(noseLength^2 + maxDiameter^2 / 4);
    finWettedArea = rocket.virtualFinArea;
    totalWettedArea = bodyWettedArea + finWettedArea;
    normalizedThicknessDistance = maxThicknessDistance / rootChord;
    
    % Material coefficients
    bodyRoughnessCoefficient = 1.016e-5;
    finRoughnessCoefficient = 2.032e-5;
    
    % Additional fin parameters (may be useful)
    finLeadingEdgePosition = rocket.finRootPosition;
    
    % Model limitation check
    if noseLength / effectiveLength > 0.6
        warning("In drag calculation, the transonic model gives a bad approximation.");
    end
        
    % ---------------------------------------------------------------------
    % 2. Body friction drag
    % ---------------------------------------------------------------------
    
    % Compressible Reynolds Number
    compressibleReynoldsNumber = speedOfSound * machNumber * totalLength / (12 * kinematicViscosity) * ...
                               (1 + 0.0283 * machNumber - 0.043 * machNumber^2 + 0.2107 * machNumber^3 - ...
                                0.03829 * machNumber^4 + 0.002709 * machNumber^5);
    
    % Incompressible skin friction coefficient
    incompressibleSkinFriction = 0.037036 * compressibleReynoldsNumber ^ (-0.155079);
    
    % Compressible skin friction coefficient
    compressibleSkinFriction = incompressibleSkinFriction * ...
                             (1 + 0.00798 * machNumber - 0.1813 * machNumber^2 + 0.0632 * machNumber^3 - ...
                              0.00933 * machNumber^4 + 0.000549 * machNumber^5);
    
    % Incompressible skin friction coefficient with roughness
    incompressibleRoughFriction = 1 / ((1.89 + 1.62 * log10(totalLength / bodyRoughnessCoefficient)) ^ 2.5);
    
    % Compressible skin friction coefficient with roughness
    compressibleRoughFriction = incompressibleRoughFriction / ((1 + 0.2044 * machNumber^2));
    
    % Final skin friction coefficient
    if compressibleSkinFriction >= compressibleRoughFriction
        finalSkinFriction = compressibleSkinFriction;
    else
        finalSkinFriction = compressibleRoughFriction;
    end
        
    % Total friction drag for body
    bodyFrictionDragCoefficient = finalSkinFriction * (1 + 60 / ((totalLength / maxDiameter)^3) + ...
                                 0.0025 * (totalLength / maxDiameter)) * (4 * bodyWettedArea) / (pi * maxDiameter^2);
    
    
    % ---------------------------------------------------------------------
    % 3.1 Fin friction drag (compressible)
    % ---------------------------------------------------------------------
    
    % Compressible Reynolds Number
    compressibleReynoldsNumber = speedOfSound * machNumber * rootChord / (12 * kinematicViscosity) * ...
                               (1 + 0.0283 * machNumber - 0.043 * machNumber^2 + 0.2107 * machNumber^3 - ...
                                0.03829 * machNumber^4 + 0.002709 * machNumber^5);
    
    % Incompressible skin friction coefficient
    incompressibleSkinFriction = 0.037036 * compressibleReynoldsNumber ^ (-0.155079);
    
    % Compressible skin friction coefficient
    compressibleSkinFriction = incompressibleSkinFriction * ...
                             (1 + 0.00798 * machNumber - 0.1813 * machNumber^2 + 0.0632 * machNumber^3 - ...
                              0.00933 * machNumber^4 + 0.000549 * machNumber^5);
    
    % Incompressible skin friction coefficient with roughness
    incompressibleRoughFriction = 1 / ((1.89 + 1.62 * log10(rootChord / finRoughnessCoefficient)) ^ 2.5);
    
    % Compressible skin friction coefficient with roughness
    compressibleRoughFriction = incompressibleRoughFriction / ((1 + 0.2044 * machNumber^2));
    
    % Final skin friction coefficient
    if compressibleSkinFriction >= compressibleRoughFriction
        finalSkinFriction = compressibleSkinFriction;
    else
        finalSkinFriction = compressibleRoughFriction;
    end
        
    % ---------------------------------------------------------------------
    % 3.2 Fin friction drag (incompressible)
    % ---------------------------------------------------------------------
        
    % Incompressible Reynolds Number
    reynoldsNumber = speedOfSound * machNumber * rootChord / (12 * kinematicViscosity);
    
    % Ratio of fin tip chord to root chord
    taperRatio = tipChord / rootChord;
    
    % Average flat plate skin friction coefficient for each fin panel
    if taperRatio == 0
        finFrictionDrag = finalSkinFriction * (1 + 0.5646 / log10(reynoldsNumber));
    elseif taperRatio == 1
        finFrictionDrag = 0;
    else
        finFrictionDrag = finalSkinFriction * ((log10(reynoldsNumber))^2.6) / ((taperRatio^2 - 1)) * ...
                        ((taperRatio^2) / ((log10(reynoldsNumber * taperRatio))^2.6) - ...
                        1 / ((log10(reynoldsNumber))^2.6) + 0.5646 * ...
                        ((taperRatio^2) / ((log10(reynoldsNumber * taperRatio))^3.6) - ...
                        1 / ((log10(reynoldsNumber))^3.6)));
    end
    
    % Coefficient of friction drag for all fins
    finsFrictionDragCoefficient = finFrictionDrag * (1 + 60 * (maxFinThickness / rootChord)^4 + ...
                                 0.8 * (1 + 5 * normalizedThicknessDistance^2) * (maxFinThickness / rootChord)) * ...
                                 (4 * finCount * finWettedArea) / (pi * maxDiameter^2);
    
    % ---------------------------------------------------------------------
    % 4. Protuberance friction drag
    % ---------------------------------------------------------------------
    
    % First protuberance set
    protuberanceCount = 8;
    protuberanceLength = 0.194 * rootChord;
    protuberanceThickness = maxFinThickness;
    protuberanceWettedArea = 2 * pi * (maxRadius + maxFinThickness) / 4 * protuberanceLength + ...
                           2 * pi / 4 * protuberanceThickness * (2 * maxRadius + protuberanceThickness);
    protuberanceRoughness = 0.008;
    distanceFromNose = finLeadingEdgePosition + rootChord / 2;
    protuberanceCrossSection = pi / 4 * protuberanceThickness * (2 * maxRadius + protuberanceThickness);
    
    compressibleReynoldsNumber = speedOfSound * machNumber * protuberanceLength / (12 * kinematicViscosity) * ...
                               (1 + 0.0283 * machNumber - 0.043 * machNumber^2 + 0.2107 * machNumber^3 - ...
                                0.03829 * machNumber^4 + 0.002709 * machNumber^5);
    incompressibleSkinFriction = 0.037036 * compressibleReynoldsNumber^(-0.155079);
    compressibleSkinFriction = incompressibleSkinFriction * ...
                             (1 + 0.00798 * machNumber - 0.1813 * machNumber^2 + 0.0632 * machNumber^3 - ...
                              0.00933 * machNumber^4 + 0.000549 * machNumber^5);
    incompressibleRoughFriction = 1 / ((1.89 + 1.62 * log10(protuberanceLength / protuberanceRoughness))^2.5);
    compressibleRoughFriction = incompressibleRoughFriction / ((1 + 0.2044 * machNumber^2));
    
    if compressibleSkinFriction >= compressibleRoughFriction
        finalSkinFriction = compressibleSkinFriction;
    else
        finalSkinFriction = compressibleRoughFriction;
    end
    
    protuberanceSkinFriction = 0.8151 * finalSkinFriction * (distanceFromNose / protuberanceLength)^(-0.1243);
    protuberanceDrag1 = protuberanceSkinFriction * (1 + 1.798 * (sqrt(protuberanceCrossSection) / protuberanceLength)^(3/2)) * ...
                      4 * protuberanceCount * protuberanceWettedArea / (pi * maxDiameter^2);
    
    % Second protuberance set
    protuberanceCount = 1;
    protuberanceLength = 0.4094488;
    protuberanceThickness = 0.15748;
    protuberanceWettedArea = 2 * pi * (maxRadius + maxFinThickness) * protuberanceLength + ...
                           2 * pi * protuberanceThickness * (2 * maxRadius + protuberanceThickness);
    protuberanceRoughness = 0.008;
    distanceFromNose = totalLength - protuberanceLength;
    protuberanceCrossSection = pi * protuberanceThickness * (2 * maxRadius + protuberanceThickness);
    
    compressibleReynoldsNumber = speedOfSound * machNumber * protuberanceLength / (12 * kinematicViscosity) * ...
                               (1 + 0.0283 * machNumber - 0.043 * machNumber^2 + 0.2107 * machNumber^3 - ...
                                0.03829 * machNumber^4 + 0.002709 * machNumber^5);
    incompressibleSkinFriction = 0.037036 * compressibleReynoldsNumber^(-0.155079);
    compressibleSkinFriction = incompressibleSkinFriction * ...
                             (1 + 0.00798 * machNumber - 0.1813 * machNumber^2 + 0.0632 * machNumber^3 - ...
                              0.00933 * machNumber^4 + 0.000549 * machNumber^5);
    incompressibleRoughFriction = 1 / ((1.89 + 1.62 * log10(protuberanceLength / protuberanceRoughness))^2.5);
    compressibleRoughFriction = incompressibleRoughFriction / ((1 + 0.2044 * machNumber^2));
    
    if compressibleSkinFriction >= compressibleRoughFriction
        finalSkinFriction = compressibleSkinFriction;
    else
        finalSkinFriction = compressibleRoughFriction;
    end
    
    protuberanceSkinFriction = 0.8151 * finalSkinFriction * (distanceFromNose / protuberanceLength)^(-0.1243);
    protuberanceDrag2 = protuberanceSkinFriction * (1 + 1.798 * (sqrt(protuberanceCrossSection) / protuberanceLength)^(3/2)) * ...
                      4 * protuberanceCount * protuberanceWettedArea / (pi * maxDiameter^2);
    
    % ---------------------------------------------------------------------
    % 5. Drag due to excrescencies
    % ---------------------------------------------------------------------
    
    % Coefficient for excrescencies drag increment
    if machNumber < 0.78
        excrescenceCoefficient = 0.00038;
    elseif machNumber <= 1.04
        excrescenceCoefficient = -0.4501 * machNumber^4 + 1.5954 * machNumber^3 - 2.1062 * machNumber^2 + ...
                               1.2288 * machNumber - 0.26717;
    else
        excrescenceCoefficient = 0.0002 * machNumber^2 - 0.0012 * machNumber + 0.0018;
    end
    
    % Change in drag coefficient due to excrescencies 
    excrescenceDragIncrement = excrescenceCoefficient * (4 * totalWettedArea) / (pi * maxDiameter^2);
    
    % ---------------------------------------------------------------------
    % 6. Total friction and interference drag coefficient
    % ---------------------------------------------------------------------
    
    % Mutual interference factor of fins and launch lug with body
    interferenceFactor = 1.04;
    totalFrictionDrag = bodyFrictionDragCoefficient + interferenceFactor * finsFrictionDragCoefficient + ...
                      excrescenceDragIncrement + interferenceFactor * protuberanceDrag1 + interferenceFactor * protuberanceDrag2;
    
    % ---------------------------------------------------------------------
    % 7. Base drag
    % ---------------------------------------------------------------------
    
    % Constants for subsonic flows
    baseDragConstant = 0.0274 * atan((aftBodyLength / maxDiameter)) + 0.0116;
    baseDragExponent = 3.6542 * (aftBodyLength / maxDiameter)^(-0.2733);
    
    % Base drag coefficient for subsonic flows (M < 0.6)
    baseDragSubsonic = baseDragConstant * ((baseDiameter / maxDiameter)^baseDragExponent) / sqrt(totalFrictionDrag);
    
    % Constants for other flows
    if machNumber > 0.6 && machNumber < 1.0
        machCorrectionFactor = 1.0 + 215.8 * (machNumber - 0.6)^6.0;
    elseif machNumber < 2.0
        machCorrectionFactor = 2.0881 * (machNumber - 1)^3 - 3.7938 * (machNumber - 1)^2 + ...
                            1.4618 * (machNumber - 1) + 1.883917;
    else
        machCorrectionFactor = 0.297 * (machNumber - 2)^3 - 0.7937 * (machNumber - 2)^2 - ...
                            0.1115 * (machNumber - 2) + 1.64006;
    end
        
    % Base drag coefficient for other flows (M > 0.6)
    baseDragSupersonic = baseDragSubsonic * machCorrectionFactor;
    
    % ---------------------------------------------------------------------
    % 8. Transonic wave drag
    % ---------------------------------------------------------------------
    
    % Transonic drag divergence Mach Number
    dragDivergenceMach = -0.0156 * (noseLength / maxDiameter)^2 + 0.136 * (noseLength / maxDiameter) + 0.6817;
    
    % Constants for the wave drag
    if noseLength / effectiveLength < 0.2
        waveDragConstant1 = 2.4;
        waveDragExponent = -1.05;
    else
        waveDragConstant1 = -321.94 * (noseLength / effectiveLength)^2 + 264.07 * (noseLength / effectiveLength) - 36.348;
        waveDragExponent = 19.634 * (noseLength / effectiveLength)^2 - 18.369 * (noseLength / effectiveLength) + 1.7434;
    end
    
    % Final Mach Number of Transonic Region
    finalTransonicMach = waveDragConstant1 * (effectiveLength / maxDiameter)^waveDragExponent + 1.0275;
    
    % Other constants for the wave drag
    waveDragConstant2 = 50.676 * (noseLength / basePosition)^2 - 51.734 * (noseLength / basePosition) + 15.642;
    waveDragConstant3 = -2.2538 * (noseLength / basePosition)^2 + 1.3108 * (noseLength / basePosition) - 1.7344;
    
    % Maximum drag rise over transonic region
    if effectiveLength / maxDiameter >= 6
        maxWaveDragRise = waveDragConstant2 * (effectiveLength / maxDiameter)^waveDragConstant3;
    else
        maxWaveDragRise = waveDragConstant2 * 6^waveDragConstant3;
    end
    
    % Wave drag function
    normalizedMach = (machNumber - dragDivergenceMach) / (finalTransonicMach - dragDivergenceMach);
    waveDragFunction = -8.3474 * normalizedMach^5 + 24.543 * normalizedMach^4 - 24.946 * normalizedMach^3 + ...
                     8.6321 * normalizedMach^2 + 1.1195 * normalizedMach;
    
    % Transonic drag rise for given Mach Number
    if machNumber >= dragDivergenceMach && machNumber <= finalTransonicMach
        transonicWaveDragRise = maxWaveDragRise * waveDragFunction;
    else
        transonicWaveDragRise = 0;
    end
    
    % ---------------------------------------------------------------------
    % 9. Supersonic wave drag
    % ---------------------------------------------------------------------

    % Supersonic drag rise for given Mach Number
    if machNumber >= finalTransonicMach
        supersonicWaveDragRise = maxWaveDragRise;
    else
        supersonicWaveDragRise = 0;
    end
    
    % ---------------------------------------------------------------------
    % 10. Total drag coefficient
    % ---------------------------------------------------------------------
    
    dragCoefficient = bodyFrictionDragCoefficient + interferenceFactor * finsFrictionDragCoefficient + ...
                    excrescenceDragIncrement + baseDragSupersonic + transonicWaveDragRise + supersonicWaveDragRise;
end
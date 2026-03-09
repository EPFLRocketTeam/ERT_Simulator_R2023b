classdef windModelTest < matlab.unittest.TestCase
    % Unit tests for windModel function

    properties
        AddedPath;
    end
    
    properties (TestParameter)
        % Test parameters for different scenarios
        validModels = {'None', 'Gaussian', 'Logarithmic'};
        invalidModels = {'Invalid', 'Turbulent', 'Dryden', ''};
        altitudes = {0, 10, 50, 100, 500, 1000, 5000};
        times = {0, 0.1, 0.5, 1, 2, 5, 10};
        intensities = {0.05, 0.1, 0.2, 0.3};
    end
    
    methods (TestClassSetup)
        function addFunctionPath(testCase)
            % This function temporarily adds the directory of the drag 
            % function to the MATLAB path so the tests can find it.
            
            % 1. Get the path of the current test file directory 
            testDir = fileparts(mfilename('fullpath'));
            
            % 2. Move up directories to reach the root folder
            rootPath = fileparts(fileparts(fileparts(testDir))); 
            
            % 3. Construct the path to the function file's directory
            functionPath = fullfile(rootPath, 'Src', 'Functions', 'Models');
            
            % 4. Add the path to MATLAB's search path
            addpath(functionPath);
            
            % 5. Store the path so we can remove it later
            testCase.AddedPath = functionPath;
        end

        function setupTestEnvironment(testCase)
            % Set up the test environment
            % This runs once before all tests
            
            % Clear global variables before all tests
            clear global windTimes windSpeeds;
        end
    end
    
    methods (TestClassTeardown)
        function removeFunctionPath(testCase)
            % This removes the path added in TestClassSetup
            rmpath(testCase.AddedPath);
        end
    end
    
    methods (TestMethodSetup)
        function setupEachTest(testCase)
            % Setup before each test
            % Clear global variables to ensure test isolation
            clear global windTimes windSpeeds;
        end
    end
    
    methods (TestMethodTeardown)
        function teardownEachTest(testCase)
            % Clean up after each test
            clear global windTimes windSpeeds;
        end
    end
    
    methods (Test)
        % Test methods for 'None' model
        
        function testNoneModelConstantWind(testCase)
            % Test that 'None' model returns constant free stream velocity
            
            time = 5.0;
            turbulenceIntensity = 0.1;
            freeStream = 10.0;
            altitude = 100;
            
            windSpeed = windModel(time, turbulenceIntensity, freeStream, 'None', altitude);
            
            % Should return free stream velocity unchanged
            testCase.verifyEqual(windSpeed, freeStream, 'AbsTol', 1e-6, ...
                'None model should return free stream velocity');
        end
        
        function testNoneModelTimeIndependence(testCase)
            % Test that 'None' model is independent of time
            
            freeStream = 15.0;
            testTimes = [0, 1, 5, 10, 50, 100];
            
            for t = testTimes
                windSpeed = windModel(t, 0.1, freeStream, 'None', 100);
                testCase.verifyEqual(windSpeed, freeStream, 'AbsTol', 1e-6, ...
                    sprintf('None model should be time-independent at t=%.1f', t));
            end
        end
        
        function testNoneModelAltitudeIndependence(testCase)
            % Test that 'None' model is independent of altitude
            
            freeStream = 12.0;
            testAltitudes = [0, 10, 100, 500, 1000, 5000];
            
            for alt = testAltitudes
                windSpeed = windModel(5.0, 0.1, freeStream, 'None', alt);
                testCase.verifyEqual(windSpeed, freeStream, 'AbsTol', 1e-6, ...
                    sprintf('None model should be altitude-independent at h=%.1f', alt));
            end
        end
        
        % Test methods for 'Gaussian' model
        
        function testGaussianModelInitialization(testCase)
            % Test that Gaussian model initializes global variables
            
            time = 0;
            turbulenceIntensity = 0.1;
            freeStream = 10.0;
            
            windSpeed = windModel(time, turbulenceIntensity, freeStream, 'Gaussian', 100);
            
            % Check global variables were initialized
            global windTimes windSpeeds;
            
            testCase.verifyEqual(windTimes, 0, 'windTimes should be initialized to 0');
            testCase.verifyNumElements(windSpeeds, 1, 'windSpeeds should have one element');
        end
        
        function testGaussianModelFirstCall(testCase)
            % Test first call to Gaussian model
            
            time = 0;
            turbulenceIntensity = 0.15;
            freeStream = 20.0;
            
            windSpeed = windModel(time, turbulenceIntensity, freeStream, 'Gaussian', 100);
            
            % Should return a value close to free stream (within ~3 standard deviations)
            stdDev = turbulenceIntensity * freeStream;
            testCase.verifyTrue(abs(windSpeed - freeStream) <= 3*stdDev, ...
                'Wind speed should be within 3 standard deviations of mean');
            
            % Verify global variables
            global windTimes windSpeeds;
            testCase.verifyEqual(windTimes, time);
            testCase.verifyEqual(windSpeeds, windSpeed);
        end
        
        function testGaussianModelSubsequentCall(testCase)
            % Test subsequent call at later time
            
            % First call at t=0
            windSpeed1 = windModel(0, 0.1, 10.0, 'Gaussian', 100);
            
            % Second call at later time
            windSpeed2 = windModel(1.0, 0.1, 10.0, 'Gaussian', 100);
            
            % Verify global variables updated
            global windTimes windSpeeds;
            testCase.verifyEqual(length(windTimes), 2);
            testCase.verifyEqual(length(windSpeeds), 2);
            testCase.verifyEqual(windTimes(1), 0);
            testCase.verifyEqual(windTimes(2), 1.0);
            testCase.verifyEqual(windSpeeds(1), windSpeed1);
            testCase.verifyEqual(windSpeeds(2), windSpeed2);
        end
        
        function testGaussianModelInterpolation(testCase)
            % Test interpolation between generated points
            
            % Generate points at t=0 and t=1
            windSpeed0 = windModel(0, 0.1, 10.0, 'Gaussian', 100);
            windSpeed1 = windModel(1.0, 0.1, 10.0, 'Gaussian', 100);
            
            % Query at intermediate time
            windSpeedInterp = windModel(0.5, 0.1, 10.0, 'Gaussian', 100);
            
            % Should be linear interpolation
            expectedInterp = (windSpeed0 + windSpeed1) / 2;
            testCase.verifyEqual(windSpeedInterp, expectedInterp, 'RelTol', 0.01, ...
                'Should linearly interpolate between generated points');
        end
        
        function testGaussianModelMultipleCalls(testCase)
            % Test multiple calls with increasing time
            
            nCalls = 5;
            times = linspace(0, 4, nCalls);
            windSpeedsHistory = zeros(1, nCalls);
            
            for i = 1:nCalls
                windSpeedsHistory(i) = windModel(times(i), 0.1, 10.0, 'Gaussian', 100);
            end
            
            % Verify each call stored its value
            global windTimes windSpeeds;
            testCase.verifyEqual(length(windTimes), nCalls);
            testCase.verifyEqual(length(windSpeeds), nCalls);
            testCase.verifyEqual(windTimes, times);
            testCase.verifyEqual(windSpeeds, windSpeedsHistory);
        end
        
        function testGaussianModelOutOfOrder(testCase)
            % Test calling with non-increasing time
            
            % Generate points in order
            windModel(0, 0.1, 10.0, 'Gaussian', 100);
            windModel(2.0, 0.1, 10.0, 'Gaussian', 100);
            
            % Query at time between (should interpolate)
            windSpeedInterp = windModel(1.0, 0.1, 10.0, 'Gaussian', 100);
            
            global windTimes windSpeeds;
            % Should not add new point
            testCase.verifyEqual(length(windTimes), 2);
        end
        
        function testGaussianModelStatistical(testCase)
            % Test statistical properties over many calls
            
            nSamples = 1000;
            freeStream = 15.0;
            turbulenceIntensity = 0.2;
            stdDev = turbulenceIntensity * freeStream;
            
            windSamples = zeros(1, nSamples);
            
            % Generate many samples at different times
            for i = 1:nSamples
                windSamples(i) = windModel(i*0.01, turbulenceIntensity, freeStream, 'Gaussian', 100);
            end
            
            % Check statistical properties
            sampleMean = mean(windSamples);
            sampleStd = std(windSamples);
            
            testCase.verifyEqual(sampleMean, freeStream, 'RelTol', 0.1, ...
                'Sample mean should approximate free stream velocity');
            testCase.verifyEqual(sampleStd, stdDev, 'RelTol', 0.2, ...
                'Sample std deviation should approximate target');
        end
        
        % Test methods for 'Logarithmic' model
        
        function testLogarithmicModelAltitudeDependence(testCase)
            % Test that Logarithmic model varies with altitude
            
            freeStream = 10.0;
            altitude = 100;
            
            windSpeed = windModel(5.0, 0.1, freeStream, 'Logarithmic', altitude);
            
            % Should be different from free stream
            testCase.verifyNotEqual(windSpeed, freeStream, ...
                'Logarithmic model should modify wind speed');
            
            % Should be positive
            testCase.verifyTrue(windSpeed > 0, 'Wind speed should be positive');
        end
        
        function testLogarithmicModelMonotonic(testCase)
            % Test that wind speed increases monotonically with altitude
            
            freeStream = 10.0;
            altitudes = [10, 50, 100, 200, 500, 1000];
            windSpeedsAltitude = zeros(size(altitudes));
            
            for i = 1:length(altitudes)
                windSpeedsAltitude(i) = windModel(5.0, 0.1, freeStream, 'Logarithmic', altitudes(i));
            end
            
            % Should be strictly increasing
            testCase.verifyTrue(all(diff(windSpeedsAltitude) > 0), ...
                'Wind speed should increase with altitude in logarithmic model');
        end
        
        function testLogarithmicModelZeroAltitude(testCase)
            % Test behavior at zero altitude (should be problematic)
            
            freeStream = 10.0;
            windSpeed = windModel(5.0, 0.1, freeStream, 'Logarithmic', 0);
            
            % At zero altitude, log(0) returns -Inf
            testCase.verifyEqual(windSpeed, -Inf, ...
                'Windspeed at 0 altitude should be -Inf due to log(0)');
        end
        
        function testLogarithmicModelVeryLowAltitude(testCase)
            % Test at very low but positive altitude
            
            freeStream = 10.0;
            lowAltitude = 0.01; % 1 cm
            
            windSpeed = windModel(5.0, 0.1, freeStream, 'Logarithmic', lowAltitude);
            
            % Should be very low (but positive)
            testCase.verifyTrue(windSpeed > 0, 'Wind speed should be positive');
            testCase.verifyTrue(windSpeed < freeStream, ...
                'Wind speed should be less than free stream at low altitude');
        end
        
        function testLogarithmicModelConsistency(testCase)
            % Test that Logarithmic model always returns same value for same inputs
            
            freeStream = 15.0;
            altitude = 200;
            
            % Multiple calls with same inputs
            windSpeed1 = windModel(5.0, 0.1, freeStream, 'Logarithmic', altitude);
            windSpeed2 = windModel(5.0, 0.1, freeStream, 'Logarithmic', altitude);
            windSpeed3 = windModel(5.0, 0.1, freeStream, 'Logarithmic', altitude);
            
            % Should all be identical
            testCase.verifyEqual(windSpeed1, windSpeed2, 'AbsTol', 1e-6);
            testCase.verifyEqual(windSpeed2, windSpeed3, 'AbsTol', 1e-6);
        end
        
        % Cross-model tests
        
        function testModelSwitching(testCase)
            % Test switching between models
            
            % Start with Gaussian
            windGauss = windModel(0.1, 0.1, 10.0, 'Gaussian', 100);
            
            % Switch to Logarithmic
            windLog = windModel(1.0, 0.1, 10.0, 'Logarithmic', 100);
            
            % Switch back to Gaussian (should interpolate)
            windGauss2 = windModel(0.5, 0.1, 10.0, 'Gaussian', 100);
            
            global windTimes windSpeeds;
            
            % Gaussian calls should affect its own global variables
            testCase.verifyEqual(length(windTimes), 3); % t=0, t=0.5, t=1.0
        end
        
        function testInvalidModel(testCase, invalidModels)
            % Test that invalid model names throw error
            
            testCase.verifyError(@() windModel(5.0, 0.1, 10.0, invalidModels, 100), ...
                'MATLAB:UnknownModelError', ...
                sprintf('Invalid model "%s" should throw error', invalidModels));
        end
        
        function testGlobalPersistence(testCase)
            % Test that global variables persist between calls
            
            % First call
            windSpeed1 = windModel(0, 0.1, 10.0, 'Gaussian', 100);
            
            global windTimes windSpeeds;
            times1 = windTimes;
            speeds1 = windSpeeds;
            
            % Second call (should append)
            windSpeed2 = windModel(1.0, 0.1, 10.0, 'Gaussian', 100);
            
            testCase.verifyEqual(length(windTimes), 2);
            testCase.verifyEqual(windTimes(1), times1);
            testCase.verifyEqual(windTimes(2), 1.0);
            testCase.verifyEqual(windSpeeds(1), speeds1);
        end
        
        function testGaussianIntensityEffect(testCase, intensities)
            % Test that turbulence intensity affects the spread
            
            freeStream = 20.0;
            nSamples = 100;
            
            % Generate samples with different intensities
            samplesLow = zeros(1, nSamples);
            samplesHigh = zeros(1, nSamples);
            
            for i = 1:nSamples
                samplesLow(i) = windModel(i*0.01, intensities, freeStream, 'Gaussian', 100);
            end
            
            % Clear globals for next test
            clear global windTimes windSpeeds;
            
            for i = 1:nSamples
                samplesHigh(i) = windModel(i*0.01, intensities*2, freeStream, 'Gaussian', 100);
            end
            
            stdLow = std(samplesLow);
            stdHigh = std(samplesHigh);
            
            % Higher intensity should generally give higher standard deviation
            testCase.verifyTrue(stdHigh > stdLow * 0.8, ...
                'Higher turbulence intensity should increase spread');
        end
    end
    
    methods (Test, ParameterCombination = 'sequential')
        function testLogarithmicVariousAltitudes(testCase, altitudes)
            % Parameterized test for logarithmic model at different altitudes
            
            freeStream = 15.0;
            
            if altitudes == 0
                windSpeed = windModel(5.0, 0.1, freeStream, 'Logarithmic', altitudes);
                testCase.verifyEqual(windSpeed, -Inf, 'Wind speed should be -Inf due to log(0)');
            else
                windSpeed = windModel(5.0, 0.1, freeStream, 'Logarithmic', altitudes);
                
                % Should be positive and finite
                testCase.verifyTrue(windSpeed > 0, 'Wind speed should be positive');
                testCase.verifyTrue(isfinite(windSpeed), 'Wind speed should be finite');
                
                % Should be less than free stream at low altitudes, greater at high altitudes
                z0 = 0.0024;
                groundAltitude = 1.5;
                expectedSpeed = freeStream * (log(altitudes/z0) / log(groundAltitude/z0));
                
                testCase.verifyEqual(windSpeed, expectedSpeed, 'RelTol', 1e-6, ...
                    'Logarithmic model should match expected formula');
            end
        end
        
        function testGaussianVariousTimes(testCase, times)
            % Parameterized test for Gaussian model at different times
            
            global windTimes windSpeeds;
            
            freeStream = 12.0;
            
            windSpeed = windModel(times, 0.1, freeStream, 'Gaussian', 100);
            
            if times == 0
                % First call should initialize
                testCase.verifyEqual(windTimes(end), times);
                testCase.verifyEqual(windSpeeds(end), windSpeed);
            elseif times <= max(windTimes) && ~isempty(windTimes)
                % Should interpolate
                testCase.verifyTrue(isfinite(windSpeed), 'Interpolated speed should be finite');
            end
        end
    end
end
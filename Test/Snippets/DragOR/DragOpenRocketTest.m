classdef DragOpenRocketTest < matlab.unittest.TestCase
    % Test class for the drag_OR function
    
    properties (Access = private)
        AddedPath;
        SampleDragData;
    end
    
    methods (TestClassSetup)
        function addFunctionPath(testCase)
            % Add the directory containing drag_OR to the path
            testDir = fileparts(mfilename('fullpath'));
            rootPath = fileparts(fileparts(fileparts(testDir))); 
            functionPath = fullfile(rootPath, 'Src', 'Snippets', 'DragOR');
            addpath(functionPath);
            testCase.AddedPath = functionPath;
            
            % Create sample drag data
            testCase.createSampleDragData();
        end
    end
    
    methods (TestClassTeardown)
        function removeFunctionPath(testCase)
            rmpath(testCase.AddedPath);
        end
    end
    
    methods (Access = private)
        function createSampleDragData(testCase)
            % Create realistic sample drag data with better distribution
            time = linspace(0, 10, 30)';
            altitude = 50 * time + 10 * sin(time);
            speed = 80 + 20 * cos(time/2);
            dragCoeff = 0.5 + 0.3 * sin(time/3) + 0.1 * cos(time);
            testCase.SampleDragData = [time, altitude, speed, dragCoeff];
        end
    end

    methods (Test)
        
        function testTimeInterpolation(testCase)
            % Test interpolation based on time
            dragCoefficient = dragOR(testCase.SampleDragData, 'time', 5.5, 0, 0);
            testCase.verifyTrue(isscalar(dragCoefficient) && dragCoefficient >= 0, ...
                'Should return non-negative scalar');
        end
        
        function testAltitudeInterpolation(testCase)
            % Test interpolation based on altitude
            dragCoefficient = dragOR(testCase.SampleDragData, 'altitude', 0, 250, 0);
            testCase.verifyTrue(isscalar(dragCoefficient) && dragCoefficient >= 0, ...
                'Should return non-negative scalar');
        end
        
        function testSpeedInterpolation(testCase)
            % Test interpolation based on speed
            dragCoefficient = dragOR(testCase.SampleDragData, 'speed', 0, 0, 75);
            testCase.verifyTrue(isscalar(dragCoefficient) && dragCoefficient >= 0, ...
                'Should return non-negative scalar');
        end
        
        function testCaseInsensitivity(testCase)
            % Test case insensitivity
            dragCoefficient1 = dragOR(testCase.SampleDragData, 'TIME', 5, 0, 0);
            dragCoefficient2 = dragOR(testCase.SampleDragData, 'time', 5, 0, 0);
            testCase.verifyEqual(dragCoefficient1, dragCoefficient2, 'Should be case insensitive');
        end
        
        function testInvalidInterpolationType(testCase)
            % Test invalid interpolation type
            testCase.verifyError(@() dragOR(testCase.SampleDragData, 'invalid', 5, 0, 0), ...
                'drag_OR:InvalidInterpType');
        end
        
        function testEmptyData(testCase)
            % Test empty data
            testCase.verifyError(@() dragOR([], 'time', 5, 0, 0), ...
                'drag_OR:EmptyData');
        end
        
        function testInsufficientInputs(testCase)
            % Test insufficient inputs
            testCase.verifyError(@() dragOR(testCase.SampleDragData, 'time'), ...
                'drag_OR:InsufficientInputs');
        end
        
        function testSmallDataset(testCase)
            % Test with small dataset
            smallData = [1, 100, 50, 0.8; 2, 200, 60, 0.7; 3, 300, 70, 0.6];
            dragCoefficient = dragOR(smallData, 'time', 1.5, 0, 0);
            testCase.verifyTrue(isscalar(dragCoefficient) && dragCoefficient >= 0, ...
                'Should handle small datasets');
        end
        
        function testSinglePointDataset(testCase)
            % Test with single data point (should error)
            singlePoint = [1, 100, 50, 0.8];
            testCase.verifyError(@() dragOR(singlePoint, 'time', 1.5, 0, 0), ...
                'drag_OR:InsufficientData');
        end
        
        function testDataWithNaN(testCase)
            % Test data with NaN values
            dataWithNaN = testCase.SampleDragData;
            dataWithNaN(5, :) = NaN; % Add a NaN row
            dragCoefficient = dragOR(dataWithNaN, 'time', 5, 0, 0);
            testCase.verifyTrue(isscalar(dragCoefficient) && dragCoefficient >= 0, ...
                'Should handle NaN values gracefully');
        end
        
        function testDuplicateXValues(testCase)
            % Test data with duplicate x values
            duplicateData = [testCase.SampleDragData; testCase.SampleDragData(1:5, :)];
            dragCoefficient = dragOR(duplicateData, 'time', 5, 0, 0);
            testCase.verifyTrue(isscalar(dragCoefficient) && dragCoefficient >= 0, ...
                'Should handle duplicate x values');
        end
        
        function testModerateExtrapolation(testCase)
            % Test with moderate extrapolation (within reasonable bounds)
            warning('off', 'drag_OR:ModerateExtrapolation');
            dragCoefficient = dragOR(testCase.SampleDragData, 'time', 12, 0, 0);
            warning('on', 'drag_OR:ModerateExtrapolation');
            
            testCase.verifyTrue(isscalar(dragCoefficient) && dragCoefficient >= 0 && dragCoefficient <= 2, ...
                'Should handle moderate extrapolation gracefully');
        end

        function testFarExtrapolation(testCase)
            % Test with far extrapolation
            warning('off', 'drag_OR:FarExtrapolation');
            dragCoefficient = dragOR(testCase.SampleDragData, 'time', 100, 0, 0);
            warning('on', 'drag_OR:FarExtrapolation');
            
            testCase.verifyTrue(isscalar(dragCoefficient) && dragCoefficient >= 0 && dragCoefficient <= 2, ...
                'Should handle far extrapolation gracefully');
        end
    end
end
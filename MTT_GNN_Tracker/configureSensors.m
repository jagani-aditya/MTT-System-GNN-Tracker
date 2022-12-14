function [sensors] = configureSensors(egoCar,SensorsSampleRate)
% Configures the implemented radar and vision sensors on the vehicle
sensors = cell(12,1);

%% Long-range radars
% Front-facing long-range radar sensor at the center of the front bumper of the car.
sensors{1} = radarDetectionGenerator('SensorIndex', 1, ...
    'Height', 0.2, ...
    'SensorLocation',[egoCar.Wheelbase + egoCar.FrontOverhang, 0], ...
    'MaxRange', 200, ... 
    'RangeResolution', 2, ...
    'HasRangeRate', true, ...
    'RangeRateResolution', 0.77, ...
    'FieldOfView', [17, 5], ...
    'AzimuthResolution', 1, ...
    'UpdateInterval',SensorsSampleRate, ...
    'Yaw', 0);

% Rear-facing long-range radar sensor at the center of the rear bumper of the car.
sensors{2} = radarDetectionGenerator('SensorIndex', 2, ...
    'Height', 0.2, ...
    'SensorLocation',[-egoCar.RearOverhang, 0], ...
    'MaxRange', 200, ... 
    'RangeResolution', 2, ...
    'HasRangeRate', true, ...
    'RangeRateResolution', 0.77, ...
    'FieldOfView', [17, 5], ...
    'AzimuthResolution', 1, ...
    'UpdateInterval',SensorsSampleRate, ...
    'Yaw', 180);

% Left side-facing long-range radar sensor on top of the front-left wheel
sensors{3} = radarDetectionGenerator('SensorIndex', 3, ...
    'Height', 0.2, ...
    'SensorLocation',[egoCar.Wheelbase, egoCar.Width/2], ...
    'MaxRange', 200, ... 
    'RangeResolution', 2, ...
    'HasRangeRate', true, ...
    'RangeRateResolution', 0.77, ...
    'FieldOfView', [17, 5], ...
    'AzimuthResolution', 1, ...
    'UpdateInterval',SensorsSampleRate, ...
    'Yaw', 90);

% Right side-facing long-range radar sensor on top of the front-right wheel
sensors{4} = radarDetectionGenerator('SensorIndex', 4, ...
    'Height', 0.2, ...
    'SensorLocation',[egoCar.Wheelbase, -egoCar.Width/2], ...
    'MaxRange', 200, ... 
    'RangeResolution', 2, ...
    'HasRangeRate', true, ...
    'RangeRateResolution', 0.77, ...
    'FieldOfView', [17, 5], ...
    'AzimuthResolution', 1, ...
    'UpdateInterval',SensorsSampleRate, ...
    'Yaw', -90);

%% Mid-range radars
% Front-facing mid-range radar sensor at the center of the front bumper of the car.
sensors{5} = radarDetectionGenerator('SensorIndex', 5, ...
    'Height', 0.2, ...
    'SensorLocation',[egoCar.Wheelbase + egoCar.FrontOverhang, 0], ...
    'MaxRange', 60, ... 
    'RangeResolution', 2, ...
    'HasRangeRate', true, ...
    'RangeRateResolution', 1.54, ...
    'FieldOfView', [56,5], ...
    'AzimuthResolution', 4, ...
    'UpdateInterval',SensorsSampleRate, ...
    'Yaw', 0);

% Rear-facing mid-range radar sensor at the center of the rear bumper of the car.
sensors{6} = radarDetectionGenerator('SensorIndex', 6, ...
    'Height', 0.2, ...
    'SensorLocation',[-egoCar.RearOverhang, 0], ...
    'MaxRange', 60, ... 
    'RangeResolution', 2, ...
    'HasRangeRate', true, ...
    'RangeRateResolution', 1.54, ...
    'FieldOfView', [56,5], ...
    'AzimuthResolution', 4, ...
    'UpdateInterval',SensorsSampleRate, ...
    'Yaw', 180);

% Left side-facing mid-range radar sensor at the center of the side of the car
sensors{7} = radarDetectionGenerator('SensorIndex', 7, ...
    'Height', 0.2, ...
    'SensorLocation',[egoCar.Wheelbase, egoCar.Width/2], ...
    'MaxRange', 60, ... 
    'RangeResolution', 2, ...
    'HasRangeRate', true, ...
    'RangeRateResolution', 1.54, ...
    'FieldOfView', [56,5], ...
    'AzimuthResolution', 4, ...
    'UpdateInterval',SensorsSampleRate, ...
    'Yaw', 90);

% Right side-facing mid-range radar sensor at the center of the side of the car
sensors{8} = radarDetectionGenerator('SensorIndex', 8, ...
    'Height', 0.2, ...
    'SensorLocation',[egoCar.Wheelbase, -egoCar.Width/2], ...
    'MaxRange', 60, ... 
    'RangeResolution', 2, ...
    'HasRangeRate', true, ...
    'RangeRateResolution', 1.54, ...
    'FieldOfView', [56,5], ...
    'AzimuthResolution', 4, ...
    'UpdateInterval',SensorsSampleRate, ...
    'Yaw', -90);

%% Short-range radars

% Left diagonal-facing short-range radar sensor on top of the left headlight
sensors{9} = radarDetectionGenerator('SensorIndex', 9, ...
    'Height', 0.2, ...
    'SensorLocation',[egoCar.Wheelbase + egoCar.FrontOverhang, egoCar.Width/2], ...
    'MaxRange', 40, ... 
    'RangeResolution', 0.6, ...
    'HasRangeRate', true, ...
    'RangeRateResolution', 0.25, ...
    'FieldOfView', [150,5], ...
    'AzimuthResolution', 1, ... % Guess
    'UpdateInterval',SensorsSampleRate, ...
    'Yaw', 45);

% Right diagonal-facing short-range radar sensor on top of the right headlight
sensors{10} = radarDetectionGenerator('SensorIndex', 10, ...
    'Height', 0.2, ...
    'SensorLocation',[egoCar.Wheelbase + egoCar.FrontOverhang, -egoCar.Width/2], ...
    'MaxRange', 40, ... 
    'RangeResolution', 0.6, ...
    'HasRangeRate', true, ...
    'RangeRateResolution', 0.25, ...
    'FieldOfView', [150,5], ...
    'AzimuthResolution', 1, ... % Guess
    'UpdateInterval',SensorsSampleRate, ...
    'Yaw', -45);

% Left diagonal-facing short-range radar sensor on top of the left backlight
sensors{11} = radarDetectionGenerator('SensorIndex', 11, ...
    'Height', 0.2, ...
    'SensorLocation',[-egoCar.RearOverhang, egoCar.Width/2], ...
    'MaxRange', 80, ... 
    'RangeResolution', 1.2, ...
    'HasRangeRate', true, ...
    'RangeRateResolution', 0.25, ...
    'FieldOfView', [150,5], ...
    'AzimuthResolution', 1, ... % Guess
    'UpdateInterval',SensorsSampleRate, ...
    'Yaw', 90+45);

% Right diagonal-facing short-range radar sensor on top of the right backlight
sensors{12} = radarDetectionGenerator('SensorIndex', 12, ...
    'Height', 0.2, ...
    'SensorLocation',[-egoCar.RearOverhang, -egoCar.Width/2], ...
    'MaxRange', 80, ... 
    'RangeResolution', 1.2, ...
    'HasRangeRate', true, ...
    'RangeRateResolution', 0.25, ...
    'FieldOfView', [150,5], ...
    'AzimuthResolution', 1, ... % Guess
    'UpdateInterval',SensorsSampleRate, ...
    'Yaw', -90-45);


%% Vision cameras

% Front-facing camera located at front windshield.
sensors{13} = visionDetectionGenerator('SensorIndex', 13, ...
    'FalsePositivesPerImage', 0.1, ...
    'SensorLocation', [0.75*egoCar.Wheelbase, 0], ...
    'MaxRange', 130, ... 
    'Height', 1.1, ...
    'UpdateInterval',SensorsSampleRate, ...
    'Yaw', 0);

% Rear-facing camera located at front windshield.
sensors{14} = visionDetectionGenerator('SensorIndex', 14, ...
    'FalsePositivesPerImage', 0.1, ...
    'SensorLocation', [0.2*egoCar.Wheelbase 0], ...
    'MaxRange', 80, ... 
    'Height', 1.1, ...
    'UpdateInterval',SensorsSampleRate, ...
    'Yaw', 180);

end
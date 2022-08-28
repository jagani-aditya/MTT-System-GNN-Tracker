clear all;
close all;
clc;

%% Parameters
INIT_PARAMS;
DefineScenario;

%% Tracker
% Tracking is done in 2-D. Although the sensors return measurements in 3-D,
% the motion itself is confined to the horizontal plane, so there is no
% need to track the height.
tracker = multiObjectTracker('FilterInitializationFcn', @initSimDemoFilter, ...
    'AssignmentThreshold', 30, 'ConfirmationParameters', [4 5]);
positionSelector = [1 0 0 0; 0 0 1 0]; % Position selector
velocitySelector = [0 1 0 0; 0 0 0 1]; % Velocity selector

%% Define Sensors and Bird's Eye Plot
sensors = SensorsConfig(egoCar,SensorsSampleRate);
BEP = createDemoDisplay(egoCar, sensors);
BEP1 = createDemoDisplay(egoCar, sensors);

while advance(scenario)
    currentStep = currentStep + 1;
    % Get the scenario time
    time = scenario.SimulationTime;
    
    % Get the position of the other vehicle in ego vehicle coordinates
    ta = targetPoses(egoCar);

    % Simulate the sensors
    detections = {};
    isValidTime = false(1,length(sensors));
    for i = 1:length(sensors)
        [sensorDets,numValidDets,isValidTime(i)] = sensors{i}(ta, time);
        if numValidDets
            for j = 1:numValidDets
                % Vision detections do not report SNR. The tracker requires
                % that they have the same object attributes as the radar
                % detections. This adds the SNR object attribute to vision
                % detections and sets it to a NaN.
                if ~isfield(sensorDets{j}.ObjectAttributes{1}, 'SNR')
                    sensorDets{j}.ObjectAttributes{1}.SNR = NaN;
                end
            end
            detections = [detections; sensorDets]; %#ok<AGROW>
        end
    end
        
    
    % Update the tracker if there are new detections
    if any(isValidTime)
        TrackerStep = TrackerStep + 1;

        %% Cluster Detections
        VehicleDim = [sensors{1}.ActorProfiles.Length, sensors{1}.ActorProfiles.Width,...
                      sensors{1}.ActorProfiles.Height];
        [DetectionClusters] = ClusterDetections(detections, VehicleDim);
        %% Fetch Measurements
        Measurements = FetchMeasurements(DetectionClusters);

       %% Tracker Kalman Prediction
        % Predict the tracks states from previous step and propagate them to the current
        % time-step
        time1 = scenario.SimulationTime;
        Fusion_dt = time1 - time0;
        time0 = time1;
        for i = 1:size(Tracks,2)
            [Tracks(i).State, Tracks(i).StateCovariance] = KF_Predict(...
             Tracks(i).State, Tracks(i).StateCovariance,Q,Fusion_dt);
        end
        %% Tracker Data Association
        % Calculate the distance btw measured objects (detections) and tracks
        [Tracks] = DataAssoc(Tracks, Measurements,TrackerStep,AssignmentThreshold,N,R);
        
         %% Tracker Kalman Update     
        for i = 1:size(Tracks,2)
            if Tracks(i).Assigned
                [Tracks(i).State,Tracks(i).StateCovariance,Tracks(i).Sk] = KF_Correct(...
                 Tracks(i).State,Tracks(i).StateCovariance,Tracks(i).Measurement,R);
            end
        end
        
         %% Tracker Management
       [confirmedTracks,Tracks] = TrackManagement(Tracks,N,M,EliminationTH);
        
        %% Plot

%         % Update bird's-eye plot
          updateBEP(BEP, egoCar, detections, confirmedTracks, positionSelector, velocitySelector);
        %% Performance Metrics
        
        % Calculate the performance indices for each tracker
        [EAPerformanceIndices] = PerfomanceCalculation(ta,confirmedTracks,XScene,YScene,ActorRadius);
        
        % Performance metric 1) # of tracked and ground vehicles in the
        % scene at the current step
        Performance.Actors.Ground    = [Performance.Actors.Ground;  EAPerformanceIndices.NoOfActorsInScene];
        Performance.Actors.EATracks = [Performance.Actors.EATracks; EAPerformanceIndices.NoOfTracksInScene];
        
        % Performance metric 2) Mean distance of Actors in the scene wrt the
        % associated tracks using PerRadius
        Performance.MeanDistance.EA = [Performance.MeanDistance.EA; EAPerformanceIndices.MeanDistance];

        % Performance metric 3) # of ghost vehicles
        % Ghost vehicle: An actor that no track is asigned to within the
        % ghost region (PerRadius) around the vehicle at the currecnt step
        Performance.GhostActors.EA = [Performance.GhostActors.EA; EAPerformanceIndices.GhostActors];
    end

end

%% Performance Plot
plotMetrics;


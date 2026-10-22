function [s1,s2,M] = environment2()
    
% Areas coordinates (that robots use as final goal to reach)
    areaCoordinates = struct();
    areaCoordinates.A = [ 7.5,  0.6 ];
    areaCoordinates.B = [ 0.0,  7.6 ];
    areaCoordinates.C = [ 0.0,  3.3 ];
    areaCoordinates.D = [ 0.0, -1.1 ];
    areaCoordinates.E = [ 0.0, -5.5 ];
    areaCoordinates.F = [ 0.0, -8.6 ];
    areaCoordinates.G = [11.3, -8.0 ];
    areaCoordinates.H = [16.0, -8.0 ];
    areaCoordinates.I = [21.0, -8.0 ];
    areaCoordinates.L = [30.0, -7.5 ];
    areaCoordinates.M = [30.0, -2.24];
    areaCoordinates.N = [30.0,  3.0 ];
    areaCoordinates.O = [28.0,  7.4 ];
    areaCoordinates.P = [20.0,  7.4 ];
    areaCoordinates.Q = [12.5,  7.4 ];
    areaCoordinates.R = [17.0,  3.4 ];
    areaCoordinates.S = [26.0, -2.3 ];
    areaCoordinates.T = [17.0, -4.0 ];
    areaCoordinates.U = [11.3,  0.0 ];
    areaCoordinates.V = [15.8,  0.0 ];
    areaCoordinates.Z = [20.5,  0.0 ];
    
    % Sensors information (adjacencies for each sensor)
    sensorToAreas = struct();
    sensorToAreas.Camera_1  = {'B'};
    sensorToAreas.Camera_2  = {'E'};
    sensorToAreas.Camera_3  = {'U'};
    sensorToAreas.Camera_4  = {'Q'};
    sensorToAreas.Camera_5  = {'F'};
    sensorToAreas.Camera_6  = {'O'};
    sensorToAreas.Camera_7  = {'I'};
    sensorToAreas.Camera_8  = {'C'};
    sensorToAreas.Camera_9  = {'D'};
    sensorToAreas.Camera_10 = {'A'};
    sensorToAreas.Lidar_1  = {'A', 'F'};
    sensorToAreas.Lidar_2  = {'A', 'T'};
    sensorToAreas.Lidar_3  = {'G', 'T'};
    sensorToAreas.Lidar_4  = {'T', 'U'};
    sensorToAreas.Lidar_5  = {'H', 'T'};
    sensorToAreas.Lidar_6  = {'I', 'T'};
    sensorToAreas.Lidar_7  = {'L', 'S'};
    sensorToAreas.Lidar_8  = {'M', 'S'};
    sensorToAreas.Lidar_9  = {'S', 'Z'};
    sensorToAreas.Lidar_10 = {'N', 'S'};
    sensorToAreas.Lidar_11 = {'O', 'S'};
    sensorToAreas.Lidar_12 = {'P', 'R'};
    sensorToAreas.Lidar_13 = {'Q', 'R'};
    sensorToAreas.Lidar_14 = {'R', 'V'};
    
    %Robots initial position
    robotInitialPositions = [
                     0, -4.55;
                     0, 3.53
                        ];
    
    s1 = areaCoordinates;
    s2 = sensorToAreas;
    M = robotInitialPositions;
end



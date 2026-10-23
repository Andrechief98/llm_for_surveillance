function [s1,s2,M] = environment1()
    
    % Areas coordinates (that robots use as final goal to reach)
    areaCoordinates = struct();
    areaCoordinates.A = [4.0, -3.0];
    areaCoordinates.B = [3.0, -1.0];
    areaCoordinates.C = [5.0,  1.0];
    areaCoordinates.D = [4.0,  3.0];
    areaCoordinates.E = [0.0,  0.0];
    areaCoordinates.F = [-4.0, 3.5];
    areaCoordinates.G = [-4.0, 0.5];
    areaCoordinates.H = [-5.8, -3.0];
    areaCoordinates.I = [-3.0, -4.3];
    areaCoordinates.L = [-3.0, -2.5];
    % Sensors information (adjacencies for each sensor)
    sensorToAreas = struct();
    sensorToAreas.Lidar_1 = {'A', 'E'};
    sensorToAreas.Lidar_2 = {'A', 'B'};
    sensorToAreas.Lidar_3 = {'B', 'C'};
    sensorToAreas.Lidar_4 = {'C', 'D'};
    sensorToAreas.Lidar_5 = {'D', 'E'};
    sensorToAreas.Lidar_6 = {'E', 'G'};
    sensorToAreas.Lidar_7 = {'G', 'H'};
    sensorToAreas.Lidar_8 = {'L', 'I'};
    sensorToAreas.Camera_1 = {'B'};
    sensorToAreas.Camera_2 = {'F'};
    %Robots initial position
    robotInitialPositions = [
                     0, -4.55;
                     0, 3.53
                        ];
    s1 = areaCoordinates;
    s2 = sensorToAreas;
    M = robotInitialPositions;
end

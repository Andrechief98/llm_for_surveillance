function [tests] = testEnvironment2(seed)
    rng(seed)
    tests = {
        % LOGICAL SEQUENCE
        % [sensors]               "final intruder position"
        ["Camera_9"],                       "D"
        ["Camera_2"],                       "E"
        ["Lidar_7"],                        "L"
        ["Lidar_11"],                       "S"

        ["Camera_5", "Lidar_1"],            "A"
        ["Lidar_2", "Lidar_8"],             "M"
        ["Lidar_8","Lidar_14"],             "V"
        ["Lidar_13","Lidar_12"],            "R"

        ["Camera_10", "Lidar_2", "Lidar_8"],   "M"
        ["Lidar_1", "Camera_10", "Lidar_2"],   "S"
        ["Camera_6", "Lidar_11", "Lidar_12"],  "P"
        ["Lidar_5", "Lidar_2", "Lidar_1"],     "F"

        

        % FP & HI
        % [sensors]                         "final intruder position"          [Type of activation]
        ["Camera_5"],                       "-"                                 % [FP]
        ["Lidar_3"],                        randLetterFrom(["A", "T", "S"])      % [HI]
        ["Lidar_6"],                        "-"                                 % [FP]
        ["Lidar_14"],                       randLetterFrom(["Z", "R", "Q"])     % [HI]

        ["Lidar_4", "Lidar_2"],             randLetterFrom(["C", "B", "S"])     % [-, HI]
        ["Lidar_1","Camera_10"],            randLetterFrom(["E", "D", "C"])     % [-, HI]
        ["Camera_10", "Lidar_8"],                  "A"                          % [-, FP]
        ["Lidar_5","Camera_1"],                    "B"                          % [FP, -]

        ["Camera_6", "Lidar_11", "Lidar_4"],       "R"                          % [-, -, FP]
        ["Lidar_14", "Camera_2", "Lidar_10"],      "N"                          % [-, FP, -]
        ["Lidar_6", "Lidar_8", "Lidar_12"],  randLetterFrom(["R", "T", "Z"])    % [-, -, HI]
        ["Lidar_5", "Lidar_2", "Lidar_1"],   randLetterFrom(["D", "E", "C"])    % [-, -, HI]


      };
end


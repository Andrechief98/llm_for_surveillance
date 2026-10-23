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

        ["Camera_9", "Camera_10", "Lidar_1", "Camera_5"],  "F"
        ["Lidar_3", "Lidar_4", "Lidar_5", "Lidar_6"],      "S"
        ["Camera_6", "Lidar_11", "Lidar_2", "Camera_10"],  "A"
        ["Lidar_11", "Lidar_10", "Lidar_8", "Lidar_7"],    "L"

        

        % FP & HI
        % [sensors]                         "final intruder position"          [Type of activation]
        ["Camera_5"],                       "-"                                 % [FP]
        ["Lidar_3"],                        randLetterFrom(["A", "T", "S"])     % [HI]
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

        ["Camera_8", "Camera_10", "Lidar_2", "Lidar_13"],             "T"                     % [-, -, -, FP]
        ["Lidar_7", "Lidar_2", "Lidar_10", "Lidar_11"],               "R"                     % [-, FP, -, -]
        ["Camera_4", "Lidar_13", "Lidar_2", "Lidar_1"],   randLetterFrom(["C", "E", "D"])     % [-, -, -, HI]
        ["Lidar_12", "Lidar_10", "Lidar_9", "Lidar_8"],   randLetterFrom(["T", "L", "R"])     % [-, -, -, HI]

      };
end


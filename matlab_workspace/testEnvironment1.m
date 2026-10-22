function [tests] = testEnvironment1(seed)
    rng(seed)
    tests = {
        % LOGICAL SEQUENCE
        % [sensors]               "final intruder position"
        ["Camera_1"],                       "B" %
        ["Lidar_3"],                        "C"
        ["Lidar_6"],                        "G"
        ["Lidar_7"],                        "A"

        ["Lidar_5", "Camera_2"],            "F"
        ["Lidar_6", "Lidar_2"],             "A"
        ["Lidar_5","Lidar_4"],              "C"
        ["Lidar_8","Lidar_6"],              "E"

        ["Lidar_3", "Camera_1", "Lidar_7"], "A"
        ["Lidar_2", "Lidar_5", "Lidar_4"],  "C"
        ["Camera_2", "Lidar_6", "Lidar_8"], "G"
        ["Lidar_6", "Lidar_8", "Lidar_1"],  "H"

        % ["Camera_2", "Lidar_6", "Lidar_7", "Lidar_8"],  "L"
        % ["Lidar_7", "Lidar_6", "Lidar_5", "Lidar_4"],   "C"
        % ["Lidar_5", "Lidar_4", "Lidar_3", "Lidar_2"],   "A"
        % ["Lidar_1", "Lidar_8", "Lidar_7", "Lidar_6"],   "E"

        % FP & HI
        % [sensors]                         "final intruder position"          [Type of activation]
        ["Camera_2"],                       "-"                                 % [FP]
        ["Lidar_1"],                        randLetterFrom(["G", "E","A"])      % [HI]
        ["Lidar_2"],                        "-"                                 % [FP]
        ["Lidar_5"],                        randLetterFrom(["F", "C", "G"])     % [HI]

        ["Camera_1", "Lidar_3"],            randLetterFrom(["E", "A", "D"])     % [-, HI]
        ["Lidar_2", "Lidar_1"],             randLetterFrom(["E", "G", "B"])     % [-, HI]
        ["Lidar_3","Lidar_8"],              "C"                                 % [-, FP]
        ["Lidar_4","Lidar_2"],              "E"                                 % [FP, -]

        ["Camera_1", "Lidar_6", "Lidar_5"], "G"                                 % [-, -, FP]
        ["Lidar_1", "Lidar_2", "Camera_2"], "A"                                 % [-, -, FP]
        ["Lidar_5", "Lidar_4", "Lidar_6"],  randLetterFrom(["B", "H", "F"])     % [-, -, HI]
        ["Lidar_7", "Lidar_2", "Lidar_1"],  randLetterFrom(["G", "E", "B"])     % [-, -, HI]

        % ["Camera_1", "Lidar_2", "Lidar_1", "Lidar_7"],                 "A"                     % [-, -, -, FP]
        % ["Lidar_8", "Lidar_1", "Camera_1", "Lidar_3"],                 "C"                     % [-, FP, -, -]
        % ["Lidar_5", "Lidar_4", "Lidar_3", "Camera_1"],   randLetterFrom(["C", "B", "A"])       % [-, -, -, HI]
        % ["Lidar_3", "Camera_1", "Lidar_6", "Lidar_7"],   randLetterFrom(["G", "H", "I"])       % [-, -, -, HI]
      };
end


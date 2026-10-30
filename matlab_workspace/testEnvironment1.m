function [tests] = testEnvironment1(seed)
    rng(seed)
    tests = {
        % LOGICAL SEQUENCE
        % [sensors]               "final intruder position"
        ["Camera_1"],                       "B" %
        ["Lidar_3"],                        "C"
        ["Lidar_6"],                        "G"
        ["Lidar_2"],                        "A"
        ["Camera_2"],                       "F"

        ["Lidar_5", "Camera_2"],            "F"
        ["Lidar_6", "Lidar_1"],             "A"
        ["Lidar_5","Lidar_4"],              "C"
        ["Lidar_7","Lidar_6"],              "E"
        ["Camera_1","Lidar_6"],             "G"

        ["Lidar_3", "Camera_1", "Lidar_2"],   "A"
        ["Lidar_1", "Lidar_5", "Lidar_4"],    "C"
        ["Camera_2", "Lidar_6", "Lidar_7"],   "G"
        ["Lidar_6", "Lidar_7", "Lidar_8"],    "I"
        ["Lidar_8", "Camera_2", "Lidar_5"],   "D"

        ["Camera_2", "Lidar_6", "Lidar_7", "Lidar_8"],  "L"
        ["Lidar_7", "Lidar_6", "Lidar_5", "Lidar_4"],   "C"
        ["Lidar_5", "Lidar_6", "Lidar_1", "Lidar_2"],   "A"
        ["Lidar_1", "Lidar_8", "Lidar_7", "Lidar_6"],   "E"
        ["Camera_1", "Lidar_3", "Lidar_4", "Lidar_5"],  "D"

        % FP & HI
        % [sensors]                         "final intruder position"          [Type of activation]
        ["Camera_2"],                       "-"                                 % [FP]
        ["Lidar_8"],                        randLetterFrom(["H", "I", "E"])      % [HI]
        ["Lidar_1"],                        "-"                                 % [FP]
        ["Lidar_5"],                        randLetterFrom(["F", "C", "D"])     % [HI]
        ["Camera_1"],                       randLetterFrom(["B", "C", "E"])     % [HI]

        ["Camera_1", "Lidar_3"],            randLetterFrom(["E", "A", "C"])     % [-, HI]
        ["Lidar_1", "Lidar_8"],             randLetterFrom(["H", "G", "B"])     % [-, HI]
        ["Lidar_3","Lidar_7"],              "C"                                 % [-, FP]
        ["Lidar_4","Lidar_1"],              "E"                                 % [FP, -]
        ["Lidar_7","Lidar_8"],              randLetterFrom(["E", "A", "C"])     % [-, HI]

        ["Camera_1", "Lidar_6", "Lidar_5"], "G"                                 % [-, -, FP]
        ["Lidar_8", "Lidar_1", "Camera_2"], "A"                                 % [-, -, FP]
        ["Lidar_4", "Lidar_5", "Lidar_6"],  randLetterFrom(["B", "I", "F"])     % [-, -, HI]
        ["Lidar_2", "Lidar_1", "Lidar_8"],  randLetterFrom(["G", "E", "B"])     % [-, -, HI]
        ["Camera_2", "Lidar_2", "Lidar_5"], "D"                                 % [-, FP, -]

        ["Camera_1", "Lidar_2", "Lidar_1", "Lidar_7"],                 "A"                     % [-, -, -, FP]
        ["Lidar_8", "Lidar_1", "Camera_1", "Lidar_3"],                 "C"                     % [-, FP, -, -]
        ["Lidar_5", "Lidar_4", "Lidar_3", "Camera_1"],   randLetterFrom(["E", "I", "A"])       % [-, -, -, HI]
        ["Lidar_3", "Camera_1", "Lidar_6", "Lidar_7"],   randLetterFrom(["L", "E", "I"])       % [-, -, -, HI]
        ["Lidar_7", "Camera_8", "Camera_2", "Lidar_1"],                "E"                     % [-, -, FP, -]
      };
end


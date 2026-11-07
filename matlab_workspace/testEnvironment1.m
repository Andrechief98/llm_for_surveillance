function [tests] = testEnvironment1(index)

    choiceOfAreas = [
        "L", "A"     % row 1:  ["Lidar_8"] REDO
        "D", "C"     % row 2:  ["Lidar_5"]
        "C", "E"     % row 3:  ["Camera_1"]
        "E", "C"     % row 4:  ["Camera_1", "Lidar_3"] REDO
        "G", "B"     % row 5:  ["Lidar_1", "Lidar_8"]
        "A", "E"     % row 6:  ["Lidar_7", "Lidar_8"] REDO
        "I", "F"     % row 7:  ["Lidar_4", "Lidar_5", "Lidar_6"] REDO
        "E", "G"     % row 8:  ["Lidar_2", "Lidar_1", "Lidar_8"] REDO
        "A", "E"     % row 9:  ["Lidar_5", "Lidar_4", "Lidar_3", "Camera_1"]
        "I", "L"     % row 10: ["Lidar_3", "Camera_1", "Lidar_6", "Lidar_7"]
    ];

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
        ["Lidar_8"],                        choiceOfAreas(1, index)             % [HI]
        ["Lidar_1"],                        "-"                                 % [FP]
        ["Lidar_5"],                        choiceOfAreas(2, index)             % [HI]
        ["Camera_1"],                       choiceOfAreas(3, index)             % [HI]
        ["Camera_1", "Lidar_3"],            choiceOfAreas(4, index)             % [-, HI]
        ["Lidar_1", "Lidar_8"],             choiceOfAreas(5, index)             % [-, HI]
        ["Lidar_3","Lidar_7"],              "C"                                 % [-, FP]
        ["Lidar_4","Lidar_1"],              "E"                                 % [FP, -]
        ["Lidar_7","Lidar_8"],              choiceOfAreas(6, index)             % [-, HI]
        ["Camera_1", "Lidar_6", "Lidar_5"], "G"                                 % [-, -, FP]
        ["Lidar_8", "Lidar_1", "Camera_2"], "A"                                 % [-, -, FP]
        ["Lidar_4", "Lidar_5", "Lidar_6"],  choiceOfAreas(7, index)             % [-, -, HI]
        ["Lidar_2", "Lidar_1", "Lidar_8"],  choiceOfAreas(8, index)             % [-, -, HI]
        ["Camera_2", "Lidar_2", "Lidar_5"], "D"                                 % [-, FP, -]
        ["Camera_1", "Lidar_2", "Lidar_1", "Lidar_7"],   "A"                    % [-, -, -, FP]
        ["Lidar_8", "Lidar_1", "Camera_1", "Lidar_3"],   "C"                    % [-, FP, -, -]
        ["Lidar_5", "Lidar_4", "Lidar_3", "Camera_1"],   choiceOfAreas(9, index)    % [-, -, -, HI]
        ["Lidar_3", "Camera_1", "Lidar_6", "Lidar_7"],   choiceOfAreas(10, index)   % [-, -, -, HI]
        ["Lidar_7", "Lidar_8", "Camera_2", "Lidar_1"],  "E"                    % [-, -, FP, -]

      };
end


function [tests] = testEnvironment2(index)
    choiceOfAreas = [
        "T", "A"     % row 1:  ["Lidar_3"]
        "R", "Z"     % row 2:  ["Lidar_14"]
        "C", "E"     % row 3:  ["Camera_9"]
        "C", "B"     % row 4:  ["Lidar_4", "Lidar_2"] REDO
        "D", "C"     % row 5:  ["Lidar_1", "Camera_10"] 
        "P", "S"     % row 6:  ["Camera_6", "Lidar_11"]
        "R", "Z"     % row 7:  ["Lidar_6", "Lidar_8", "Lidar_12"] REDO
        "D", "E"     % row 8:  ["Lidar_5", "Lidar_2", "Lidar_1"]
        "R", "Q"     % row 9:  ["Camera_4", "Lidar_13", "Lidar_12"] REDO
        "D", "E"     % row 10: ["Camera_4", "Lidar_13", "Lidar_2", "Lidar_1"]
        "L", "R"     % row 11: ["Lidar_12", "Lidar_10", "Lidar_9", "Lidar_8"]
    ];
    
    tests = {
        % LOGICAL SEQUENCE
        % [sensors]               "final intruder position"
        ["Camera_9"],                       "D"
        ["Camera_2"],                       "E"
        ["Lidar_7"],                        "L"
        ["Lidar_11"],                       "S"
        ["Lidar_2"],                        "T"

        ["Camera_5", "Lidar_1"],            "A"
        ["Lidar_2", "Lidar_8"],             "M"
        ["Lidar_8","Lidar_14"],             "V"
        ["Lidar_13","Lidar_12"],            "R"
        ["Camera_7", "Lidar_6"],            "I"

        ["Camera_10", "Lidar_2", "Lidar_8"],   "M"
        ["Lidar_1", "Camera_10", "Lidar_2"],   "S"
        ["Camera_6", "Lidar_11", "Lidar_12"],  "P"
        ["Lidar_5", "Lidar_2", "Lidar_1"],     "F"
        ["Lidar_10", "Lidar_9", "Lidar_12"],   "R"

        ["Camera_9", "Camera_10", "Lidar_1", "Camera_5"],  "F"
        ["Lidar_3", "Lidar_4", "Lidar_5", "Lidar_6"],      "S"
        ["Camera_6", "Lidar_11", "Lidar_2", "Camera_10"],  "A"
        ["Lidar_11", "Lidar_10", "Lidar_8", "Lidar_7"],    "L"
        ["Lidar_5", "Lidar_4", "Lidar_2", "Lidar_3"],      "G"

        

        % FP & HI
        % [sensors]                         "final intruder position"          [Type of activation]
        ["Camera_5"],                       "-"                                 % [FP]
        ["Lidar_3"],                        choiceOfAreas(1, index)             % [HI]
        ["Lidar_6"],                        "-"                                 % [FP]
        ["Lidar_14"],                       choiceOfAreas(2, index)             % [HI]
        ["Camera_9"],                       choiceOfAreas(3, index)             % [HI]
        ["Lidar_4", "Lidar_2"],             choiceOfAreas(4, index)             % [-, HI]
        ["Lidar_1","Camera_10"],            choiceOfAreas(5, index)             % [-, HI]
        ["Camera_10", "Lidar_8"],           "A"                                 % [-, FP]
        ["Lidar_5","Camera_1"],             "B"                                 % [FP, -]
        ["Camera_6", "Lidar_11"],           choiceOfAreas(6, index)             % [-, HI]
        ["Camera_6", "Lidar_11", "Lidar_4"],   "R"                              % [-, -, FP]
        ["Lidar_14", "Camera_2", "Lidar_10"],  "N"                              % [-, FP, -]
        ["Lidar_6", "Lidar_8", "Lidar_12"],    choiceOfAreas(7, index)          % [-, -, HI]
        ["Lidar_5", "Lidar_2", "Lidar_1"],     choiceOfAreas(8, index)          % [-, -, HI]
        ["Camera_4", "Lidar_13", "Lidar_12"],  choiceOfAreas(9, index)          % [-, -, HI]
        ["Camera_8", "Camera_10", "Lidar_2", "Lidar_13"],  "T"                  % [-, -, -, FP]
        ["Lidar_7", "Lidar_2", "Lidar_10", "Lidar_11"],    "R"                  % [-, FP, -, -]
        ["Camera_4", "Lidar_13", "Lidar_2", "Lidar_1"],    choiceOfAreas(10, index)   % [-, -, -, HI]
        ["Lidar_12", "Lidar_10", "Lidar_9", "Lidar_8"],    choiceOfAreas(11, index)   % [-, -, -, HI]
        ["Lidar_2", "Camera_10", "Lidar_1", "Camera_2"],   "E"                  % [-, -, FP, -]
      };
end


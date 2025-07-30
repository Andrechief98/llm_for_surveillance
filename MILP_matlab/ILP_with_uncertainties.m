% robot_area_assignment.m
% Script per testare l'allocazione multirobot via MILP in MATLAB
% Dipende da Optimization Toolbox (intlinprog)

clc
clear

rng(42);  % seed per riproducibilità


%% DATA SETUP

n_trials = 3;                                                               % how much times we repet the same test
n_test = 24;                                                                % number of total tests

% 3 seeds (equals to the number of trials). It allows diversity among
% trials but reproducibility among runs
seeds = [44, 55, 66];

% Areas coordinates (that robots use as final goal to reach)
areaCoordinates = struct();

areaCoordinates.A = [4.0, -3.0];
areaCoordinates.B = [3.0, -1.0];
areaCoordinates.C = [5.0,  1.0];
areaCoordinates.D = [4.0,  3.0];
areaCoordinates.E = [0.0,  0.0];
areaCoordinates.F = [-4.0, 3.5];
areaCoordinates.G = [-4.0, 0.5];
areaCoordinates.H = [-3.0, -2.5];

% Sensors information (adjacencies for each sensor)
sensorToAreas.Lidar_1 = {'H'};
sensorToAreas.Lidar_2 = {'A', 'E'};
sensorToAreas.Lidar_3 = {'B', 'C'};
sensorToAreas.Lidar_4 = {'C', 'D'};
sensorToAreas.Lidar_5 = {'D', 'E'};
sensorToAreas.Lidar_6 = {'E', 'G'};
sensorToAreas.Lidar_7 = {'A', 'B'};
sensorToAreas.Lidar_8 = {'G'};
sensorToAreas.Camera_1 = {'B'};
sensorToAreas.Camera_2 = {'F'};

areaNames = fieldnames(areaCoordinates);
sensorNames = fieldnames(sensorToAreas);


success = zeros(n_test,n_trials);                                           % Matrix to save successes or failures

for trial=1:n_trials
    rng(seeds(trial))                                                       % fix the new seed for each trial
    
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
    };



    for test = 1:size(tests,1)
        false_positive = 0;
         
        fprintf("\nTEST %d / %d \n", test, length(tests))
    
    
        % Robots initial positions (the same as the simulation in Gazebo).
        % At the beginning of each test, we consider always the same
        % initial positions
        robotPositions = [
            0, -4.55;
            0, 3.53
            ];
        
        % Retrieving number of available robots and total number of areas
        numRobots = size(robotPositions, 1);
        numAreas = numel(areaNames);                                       
        
        % Initialization vector areas to monitor (all areas)
        areas_to_monitor = ones(1,numAreas);
        
        % Initialization matrix of areas frequencies: every time a sensor
        % is activated, each area adjacent to that sensor will increase the
        % frequency by 1
        A_freq = zeros(size(areas_to_monitor));
    
        % Sequence of activated sensors:
        sensor_seq = tests{test,1};
    
        % Intruder position:
        intruder_pos = tests{test,2};
        
        % Custom print according to the number of activated sensor in the
        % considered sequence
        if length(sensor_seq) == 1
            fprintf("Tested sensors [%s] \n", sensor_seq)
        elseif length(sensor_seq) == 2
            fprintf("Tested sensors [%s, %s] \n", sensor_seq)
        elseif length(sensor_seq) == 3
            fprintf("Tested sensors [%s, %s, %s] \n", sensor_seq)
        end
        
        % We fix a maximum number of iteration for the optimization
        % algorithm. In this case, the script will solve the Integer Linear
        % Problem at the most N_max times.
      
        N_max = round((length(sensor_seq) + 1)/2)+1;                        % round up
        
        % Loop on the sensor sequence
        for sensor = sensor_seq

            % For each sensor in the sequence, it retrieves the asjacent
            % areas
            areas = sensorToAreas.(sensor);
            
            % For each adjacent areas, it update the frequency of the
            % corresponding area in the A_freq matrix. To obtain the
            % corresponding index of the letter (indicating the considered
            % area)l the script use the function "letterToIndex" defined at
            % the end of the script
            for i=1:length(areas)
                A_freq(letterToIndex(areas{i}(1))) = A_freq(letterToIndex(areas{i}(1))) + 1;
            end
        
        end
        
        
        
        % START SIMULATION
        for n_sim = 1:N_max
            
            % Print the current information 
            fprintf("Iteration %d / %d \n", n_sim, N_max)

            % Identify wich areas are active and must be monitored
            activeAreas = find(areas_to_monitor==1);
            
            if isempty(activeAreas)
                fprintf("All areas monitored \n")
                break
            end

            % Matrix to compute the inverse of each area frequency. The
            % element of Psi are then used in the optimization algorithm to
            % promote monitoring in more frequent areas.
            Psi = zeros(size(A_freq));
            
            for i=1:length(A_freq)
                if A_freq(i) ~= 0
                    Psi(i) = 1/A_freq(i);
                else
                    % As the number of simulations increase, we further 
                    % increase alsothe Psi value. This avoids the algorithm 
                    % tries to monitor all areas indiscriminately, enforcing
                    % a criteria to choose to not deploy any robots
                    Psi(i) = 1+n_sim;
                end
            end
            
            
            % Computing actual robot - areas distances:
            D = zeros(numRobots, numAreas);                                 % Matrix of distances
            
            for i = 1:numRobots
                robot_pos = robotPositions(i, :);
                for j = 1:numAreas
                    area_pos = areaCoordinates.(areaNames{j});
                    D(i, j) = norm(robot_pos - area_pos);  
                end
            end

           
            
            %% Optimization problem       
            F = 30;
            nx = numRobots * numAreas;

            % Setup variabili e obiettivo
            nvars  = nx + 1; % + 1 for z
            intcon = 1:nvars;
            mask   = repmat(areas_to_monitor.*Psi, numRobots, 1);
            
            cost = D.*mask;

            for r = 1:numRobots
                for a = 1:numAreas
                    if (cost(r,a) == 0)
                        cost(r,a) = 1e2;                                    % Huge cost for already monitored area
                    end
                end
            end

            format bank                                                     % Used just to save and print values with 2 decimal values
            robotNames = {'Robot1','Robot2'}; 
            T = array2table(cost, ...
                'RowNames',    robotNames, ...
                'VariableNames', areaNames);            
            fprintf("Costs: \n")
            disp(T)
            format short                                                    % To come back to 4 decimal values
            
            
            f = [cost(1,:)'; cost(2,:)'; -F];
            
            % Bounds to ban deployment in non-active areas
            lb = zeros(nvars,1);
            ub = ones(nvars,1);
            for a = 1:numAreas
                if areas_to_monitor(a)==0
                    for r = 1:numRobots
                        idx      = sub2ind([numRobots,numAreas], r, a);
                        ub(idx) = 0;
                    end
                end
            end
            
            % 1) It assigns at most 1 area for each robot (in case of deployment)
            %    sum_a x_{r,a} - z ≤ 0
            A_one_area_per_robot = zeros(numRobots, nvars);
            b_one_area_per_robot = zeros(numRobots, 1);
            
            for r = 1 : numRobots
                % computing index for each x_{r,1..numAreas}
                cols = (r-1)*numAreas + (1:numAreas);

                % computing the sum_a x_{r,a} for all the areas associated to a given robot r 
                A_one_area_per_robot(r, cols) = 1;      

                % subtracting z
                A_one_area_per_robot(r, end) = -1;      
            end
            

            % 2) It assigns at most 1 robot for each area
            %    x_{1,a} + x_{2,a} + … + x_{numRobots,a} - z ≤ 0

            A_one_robot_per_area = zeros(numAreas, nvars);
            b_one_robot_per_area = zeros(numAreas, 1);
            
            for a = 1 : numAreas
                % find all x_{1,a}, x_{2,a}, …, x_{numRobots,a}
                rows = a : numAreas : nx;  
                % computing sum_r x_{r,a} for all robots assigned to a given area a
                A_one_robot_per_area(a, rows) = 1;
                % subtracting z
                A_one_robot_per_area(a, end) = -1;
            end
            

            % 3) It imposes the deployment of K robots (in case of deployment)
            %    sum_{idx=1..nx} x_idx  - K*z ≤ 0
            K = min(numRobots, sum(areas_to_monitor));
            A_deployment = [ones(1, nx),  -K];
            b_deployment = 0;

            
            Aineq = [
                A_one_area_per_robot;  
                A_one_robot_per_area;
            ];
            
            bineq = [
                b_one_area_per_robot;
                b_one_robot_per_area;
            ];

            Aeq = [A_deployment];
            beq = [b_deployment];
          
            
            %% Risoluzione 
            options = optimoptions('intlinprog','Display','none');
            tic;
            [sol, fval, exitflag, output] = intlinprog(f, intcon, Aineq, bineq, Aeq, beq, lb, ub, [], options); % Se si usa MOSEK togliere la cartella dal path perchè ha la medesima funzione che crea conflitto con l'optimization toolbox
            time = toc;
            
            assignments = {}; 

            if exitflag == 1
                % Solution extraction
    
                % x_opt = reshape(sol(1:nvars-1), [numRobots, numAreas]);
    
                x_opt_rob1 = sol(1:(nvars-1)/2);
                x_opt_rob2 = sol((nvars-1)/2+1:end-1);
                x_opt = [x_opt_rob1'; x_opt_rob2'];
    
                z_opt = sol(end);

                if z_opt == 0
                    fprintf("False positive: main areas monitored -  no intruder identified\n")
                    fprintf("Intruder position: %s\n", intruder_pos)
                    false_positive = 1;
                    break
                else
                    if isempty(sol) % If the problem is infeasible or you stopped early with no solution
                        disp('The solver did not return a solution.')
                        return % Stop the script because there is nothing to examine

                    else

                        for r = 1:numRobots
                            for i = 1:numAreas
                                if round(x_opt(r,i)) == 1
                                    % We convert the index of the assigned area to the
                                    % corresponding letter
                                    areaLetter = indexToLetter(i);
        
                                    % Adding data in the row of the cell array   
                                    assignments(end+1, :) = {r, areaLetter, D(r, i), A_freq(i), round(D(r, i)*Psi(i),2)};
                                end
                            end
                        end

                        %% Results visulization
                        activeAreas_letters = cell(size(activeAreas));
                        for i = 1:length(activeAreas)
                            activeAreas_letters{i} = indexToLetter(activeAreas(i));
                        end
                        
                        disp('   Areas to monitor:'); disp(activeAreas_letters);
                        disp('   Assignments:');
                        disp(cell2table(assignments, 'VariableNames', {'Robot','Area','Distance','Frequency', 'Cost'}));
                        % disp(['Costo totale (distanza ponderata): ', num2str(fval)]);
                
                
                        %% Update date for next iteration
                        
                        deployed_robots = length(assignments(:,1));
                        assigned_area_idx = zeros(1,deployed_robots);
                        assigned_area_letter = char(1, deployed_robots);
                
                        for r =1:deployed_robots
                            assigned_area_letter(r) = assignments{r,2};
                            assigned_area_idx(r) = letterToIndex(assigned_area_letter(r));
                            robotPositions(r,:) = areaCoordinates.(assigned_area_letter(r));
                        end
                        
                        areas_to_monitor(assigned_area_idx) = 0;
                
                       
                        if ismember(intruder_pos, assigned_area_letter)
                            success(test,trial) = 1;
                            fprintf("Success: Intruder identified \n")
                            fprintf("Intruder position: %s \n", intruder_pos)
                            fprintf("Areas monitored: %c, %c \n", assigned_area_letter)
                
                            % press ENTER to continue;
                            % input('');
                            break
                        end
                    end

                end
            else
                warning('Solver exit flag: %d', exitflag);
            end
              
        
        end
        
        
        % if success(test,trial) == 0 && false_positive == 0
        %     fprintf("Max number of iteration reached: Intruder not identified \n")
        %     % press ENTER to continue;
        %     % input('');
        % end


        if false_positive == 0
            if success(test,trial) == 0
                fprintf("Max number of iteration reached: Intruder not identified \n")
                fprintf("Intruder position: %s\n", intruder_pos)
                % press ENTER to continue;
                % input('');
            end
        else
            if intruder_pos == "-"
                success(test,trial) = 1;
            end
        end
            
    end
end

%% Computing results
success_mean = mean(success,2);

success_rate_1sens_logical = sum(success_mean(1:4))/length(success_mean(1:4))
success_rate_2sens_logical = sum(success_mean(5:8))/length(success_mean(5:8))
success_rate_3sens_logical = sum(success_mean(9:12))/length(success_mean(9:12))

success_rate_1sens_fp_hi = sum(success_mean(13:16))/length(success_mean(13:16))
success_rate_2sens_fp_hi = sum(success_mean(17:20))/length(success_mean(17:20))
success_rate_3sens_fp_hi = sum(success_mean(21:24))/length(success_mean(21:24))


%%
%%%%%%%%%%

function idx = letterToIndex(letter)
% it translates a letter (A-Z, a-z)  in the corresponding alphabetic index.
% Examples:
%   letterToIndex('A') -> 1
%   letterToIndex('c') -> 3

    if ~ischar(letter) || length(letter) ~= 1 || ~isletter(letter)
        error('Input non valido: inserire una singola lettera (A-Z o a-z).');
    end

    idx = double(upper(letter)) - double('A') + 1;
end

function letter = indexToLetter(idx)
% it translates an alphabetic index (1-26) to the corresponding letter.
% Examples:
%   indexToLetter(1) -> 'A'
%   indexToLetter(3) -> 'C'

    if ~isscalar(idx) || ~isnumeric(idx) || idx < 1 || idx > 26 || idx ~= floor(idx)
        error('Input non valido: inserire un intero tra 1 e 26.');
    end

    letter = char(double('A') + idx - 1);
end

function letter = randLetterFrom(letters)
% It returns a random letter from a given array of letters.
% Example:
%   letter = randLetterFrom(["A","B","C"]) 

    if isempty(letters)
        error('L''array di lettere è vuoto.');
    end

    idx = randi(numel(letters));
    letter = letters(idx);
end

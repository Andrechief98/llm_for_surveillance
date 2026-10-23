% robot_area_assignment.m
% Script per testare l'allocazione multirobot via MILP in MATLAB
% Dipende da Optimization Toolbox (intlinprog)

clc
clear

% rng(42);  % seed per riproducibilità

%% DATA SETUP
% [sensors] and intruder's position
tests = {
    % Logical sequence
    ["Camera_1"],                       "B"
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
    ["Camera_2"],                       "D" % FP
    ["Lidar_1"],                        randLetterFrom(["G"]) % HI
    ["Lidar_2"],                        "B" % FP
    ["Lidar_5"],                        randLetterFrom(["A","C"]) % HI

    ["Camera_1", "Lidar_3"],            randLetterFrom(["F","C"]) % -, HI
    ["Lidar_2", "Lidar_1"],             randLetterFrom(["A","C"]) % -, HI
    ["Lidar_3","Lidar_8"],              "C" % -, FP
    ["Lidar_4","Lidar_2"],              "E" % FP, -

    ["Camera_1", "Lidar_6", "Lidar_5"], "A" % -, -, FP
    ["Lidar_1", "Lidar_2", "Camera_2"], "C" % -, -, FP
    ["Lidar_5", "Lidar_4", "Lidar_6"],  randLetterFrom(["B","C"]) % -, -, HI
    ["Lidar_7", "Lidar_2", "Lidar_1"],  randLetterFrom(["G","C"]) % -, -, HI
};



% Areas information
areaCoordinates = struct();

areaCoordinates.A = [4.0, -3.0];
areaCoordinates.B = [3.0, -1.0];
areaCoordinates.C = [5.0,  1.0];
areaCoordinates.D = [4.0,  3.0];
areaCoordinates.E = [0.0,  0.0];
areaCoordinates.F = [-4.0, 3.5];
areaCoordinates.G = [-4.0, 0.5];
areaCoordinates.H = [-3.0, -2.5];

% Sensors information
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

n_trials = 3;
success = zeros(size(tests,1),n_trials);

for trial=1:n_trials
    for test = 1:size(tests,1)
    
        fprintf("TEST %d / %d \n", test, length(tests))
    
    
        % Robot initial positions
        robotPositions = [0, -4.55;
                          0, 3.53];
        
        
        numRobots = size(robotPositions, 1);
        numAreas = numel(areaNames);
        
        % Inizializzazion vettore aree da monitorare
        areas_to_monitor = zeros(1,numAreas);
        
        % Inizializzazione matrice delle frequenze
        A_freq = zeros(size(areas_to_monitor));
    
        % Sequence of activated sensors:
        sensor_seq = tests{test,1};
    
        % Intruder position:
        intruder_pos = tests{test,2};
        
        if length(sensor_seq) == 1
            fprintf("Tested sensors [%s] \n", sensor_seq)
        elseif length(sensor_seq) == 2
            fprintf("Tested sensors [%s, %s] \n", sensor_seq)
        elseif length(sensor_seq) == 3
            fprintf("Tested sensors [%s, %s, %s] \n", sensor_seq)
        end

        N_max = round((length(sensor_seq) + 1)/2); % arrotonda per eccesso
        
        % In relazione alla sequenza di sensori che si sono attivati vado a
        % creare il vettore delle aree attive
        for sensor = sensor_seq
            areas = sensorToAreas.(sensor);
                
            area_idx = zeros(1,length(areas));
            for i=1:length(areas)
                area_idx(i) = letterToIndex(areas{i}(1));
                A_freq(letterToIndex(areas{i}(1))) = A_freq(letterToIndex(areas{i}(1))) + 1;
            end
        
        
            areas_to_monitor(area_idx) = 1;
        end
        
        % START SIMULATION
        for n_sim = 1:N_max
           
            fprintf("Iteration %d / %d \n", n_sim, N_max)
            activeAreas = find(areas_to_monitor==1);
    
            if isempty(activeAreas)
                fprintf("All areas monitored \n")
                break
            end
    
            
            for i=1:length(A_freq)
                if A_freq(i) ~= 0
                    A_freq(i) = 1/A_freq(i);
                else
                    A_freq(i) = 2;
                end
            end
            
            
            % Calcolo distanze robot - aree da monitorare:
            D = zeros(numRobots, numAreas);
            
            for i = 1:numRobots
                robot_pos = robotPositions(i, :);
                for j = 1:numAreas
                    area_pos = areaCoordinates.(areaNames{j});
                    D(i, j) = norm(robot_pos - area_pos);  
                end
            end
            
            %% Optimization problem       
              
            % Setup variabili e obiettivo
            nvars  = numRobots * numAreas;
            intcon = 1:nvars;
            mask   = repmat(areas_to_monitor.*A_freq, numRobots, 1);
            f      = D(:) .* mask(:);              % zero per aree inattive
            
            % Bounds per vietare le aree non attive
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
            
            % Vincoli di disuguaglianza
            Aineq = zeros(numRobots + numAreas, nvars);
            bineq = ones (numRobots + numAreas, 1);
            
            % 1) ogni robot al più 1 area
            for r = 1:numRobots
                for a = 1:numAreas
                    Aineq(r, sub2ind([numRobots,numAreas], r, a)) = 1;
                end
            end
            
            % 2) ogni area al più 1 robot
            for a = 1:numAreas
                for r = 1:numRobots
                    Aineq(numRobots + a, sub2ind([numRobots,numAreas], r, a)) = 1;
                end
            end
            
            % Vincolo globale di numero di assegnazioni
            M = sum(areas_to_monitor);      % aree attive
            K = min(numRobots, M);          % quanti robot inviare
            Aeq = ones(1, nvars);
            beq = K;
            
            A = [Aineq];
            b = [bineq];
            
            
            %% Risoluzione 
            options = optimoptions('intlinprog','Display','none');
            tic;
            [sol, fval, exitflag, output] = intlinprog(f, intcon, A, b, Aeq, beq, lb, ub, [], options); % Se si usa MOSEK togliere la cartella dal path perchè ha la medesima funzione che crea conflitto con l'optimization toolbox
            time = toc;
            
            if isempty(sol) % If the problem is infeasible or you stopped early with no solution
                disp('The solver did not return a solution.')
                return % Stop the script because there is nothing to examine
            else
                x_opt = reshape(sol(1:nvars), [numRobots, numAreas]);
            end
            
            %% Estrazione soluzione
            
            assignments = {};  % Cell array vuota
            if exitflag == 1
                for r = 1:numRobots
                    for i = 1:numAreas
                        if round(x_opt(r,i)) == 1
                            % Usa indexToLetter per convertire l'indice in lettera
                            areaLetter = indexToLetter(i);
                            % Inserisci i dati come riga nella cell array
                            assignments(end+1, :) = {r, areaLetter, D(r, i), 1/A_freq(i), D(r, i)*A_freq(i)};
                        end
                    end
                end
            else
                warning('Solver exit flag: %d', exitflag);
            end
            
            %% Visualizzazione risultati
            activeAreas_letters = cell(size(activeAreas));
            for i = 1:length(activeAreas)
                activeAreas_letters{i} = indexToLetter(activeAreas(i));
            end
            
            disp('   Active areas:'); disp(activeAreas_letters);
            disp('   Assignments:');
            disp(cell2table(assignments, 'VariableNames', {'Robot','Area','Distanza','Frequenza', 'Costo'}));
            % disp(['Costo totale (distanza ponderata): ', num2str(fval)]);
    
    
            %% Aggiornamento dati per l'iterazione successiva
            
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
    
                % press ENTER to continue;
                % input('');
                break
            end
        
        end
    
        if success(test,trial) == 0
            fprintf("Max number of iteration reached: Intruder not identified \n")
            % press ENTER to continue;
            % input('');
        end
            
    end
end

success = mean(success,2);

success_rate_1sens_logical = sum(success(1:4))/length(success(1:4))
success_rate_2sens_logical = sum(success(5:8))/length(success(5:8))
success_rate_3sens_logical = sum(success(9:12))/length(success(9:12))

success_rate_1sens_fp_hi = sum(success(13:16))/length(success(13:16))
success_rate_2sens_fp_hi = sum(success(17:20))/length(success(17:20))
success_rate_3sens_fp_hi = sum(success(21:24))/length(success(21:24))

%%%%%%%%%%
% Funzione per convertire la lettere nel corrispondente indice
function idx = letterToIndex(letter)
% converte una lettera (A-Z, a-z) nell'indice alfabetico corrispondente.
% Esempi:
%   letterToIndex('A') -> 1
%   letterToIndex('c') -> 3

    if ~ischar(letter) || length(letter) ~= 1 || ~isletter(letter)
        error('Input non valido: inserire una singola lettera (A-Z o a-z).');
    end

    idx = double(upper(letter)) - double('A') + 1;
end

function letter = indexToLetter(idx)
% converte un indice alfabetico (1-26) nella corrispondente lettera maiuscola.
% Esempi:
%   indexToLetter(1) -> 'A'
%   indexToLetter(3) -> 'C'

    if ~isscalar(idx) || ~isnumeric(idx) || idx < 1 || idx > 26 || idx ~= floor(idx)
        error('Input non valido: inserire un intero tra 1 e 26.');
    end

    letter = char(double('A') + idx - 1);
end

function letter = randLetterFrom(letters)
% Restituisce una lettera scelta a caso da un array di lettere.
%   letter = randLetterFrom(["A","B","C"]) restituisce una delle lettere a caso

    % Verifica che l'input sia un array non vuoto
    if isempty(letters)
        error('L''array di lettere è vuoto.');
    end

    % Estrai una lettera a caso
    idx = randi(numel(letters));
    letter = letters(idx);
end

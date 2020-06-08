%% Simulation
close all;
clear;
clc;

% Launch the Main Menu
mode = MainMenu;

% Check which mode has been selected
switch mode
    case 1
        % Run the Simulation
        simulation = Simulation;
    case 2
        % Run Advanced Teach
        advancedTeach = AdvancedTeach;
    case 3
        % Run Visual Servoing away from safety symbol (Safety Demo)
        safetyDemo = SafetyDemo;
    otherwise
        disp("Application Closed");
end
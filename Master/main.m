%% Initialisation
close all;
clc;
clear;
try
    delete(camera);
end
pause(3);
% Create all the objects in the environment
table = EnvironmentObject('Type', 'foundation', 'ModelPath', 'table.ply', 'Pose', transl(0, 0, 0), 'Dimensions', [2.1956 1.0097 0.8911], 'GeneralColour', 'r');
blueCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'blueCrate.ply', 'Pose', transl(0.75, 0.2, table.dimensions(1,3)), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'b');
yellowCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'yellowCrate.ply', 'Pose', transl(0.75, 0, table.dimensions(1,3)), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'g');
redCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'redCrate.ply', 'Pose', transl(0.75, -0.2, table.dimensions(1,3)), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'r');
redPen = EnvironmentObject('Type', 'target', 'ModelPath', 'redPen.ply', 'Pose', transl(0.45, 0.25, table.dimensions(1,3)), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'r');
bluePen = EnvironmentObject('Type', 'target', 'ModelPath', 'bluePen.ply', 'Pose', transl(0, -0.2, table.dimensions(1,3)), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'b');
pencil1 = EnvironmentObject('Type', 'target', 'ModelPath', 'pencil.ply', 'Pose', transl(0.15, 0.2, table.dimensions(1,3)), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'g');
pencil2 = EnvironmentObject('Type', 'target', 'ModelPath', 'pencil.ply', 'Pose', transl(-0.15, -0.25, table.dimensions(1,3)), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'g');

% Create the environment
environment = Environment(blueCrate, yellowCrate, redCrate, redPen, bluePen, pencil1, pencil2, table);

% Create the Dobot
Dobot1 = Dobot('BasePose', eye(4)*transl(0,0,table.dimensions(1,3)));
Dobot1.GenerateLinearRail([-0.45,0,table.dimensions(1,3)]);

% Add the Dobot to the environment
environment.AddRobot(Dobot1);

% Create Camera
camera = RGBCamera('CentrePose', transl(0, 0, 2.5) * troty(pi));

% Create the Master Controller and tell it about the environment
plc = GlobalController(environment, camera);
plc.environment.Display();
% Initialise the simulation
plc.Init();

disp('Initialisation Complete');
plc.Run();
%% Simulation
close all;
clear;
clc;

mode = MainMenu;

switch mode
    case 1
        simulation = Simulation;
    case 2
        advancedTeach = AdvancedTeach;
    case 3
        safetyDemo = SafetyDemo;
    otherwise
        disp("Application Closed");
end
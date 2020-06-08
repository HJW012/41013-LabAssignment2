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
bluePen = EnvironmentObject('Type', 'target', 'ModelPath', 'bluePen.ply', 'Pose', transl(0, -0.2, table.dimensions(1,3)) * trotz(deg2rad(20)), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'b');
pencil1 = EnvironmentObject('Type', 'target', 'ModelPath', 'pencil.ply', 'Pose', transl(0.15, 0.2, table.dimensions(1,3)), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'g');
pencil2 = EnvironmentObject('Type', 'target', 'ModelPath', 'pencil.ply', 'Pose', transl(-0.15, -0.25, table.dimensions(1,3)), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'g');
plc = GlobalController();
plc.Setup();
plc.environment.AddObject(table);

%plc.environment.AddObject(blueCrate);

%plc.environment.AddObject(yellowCrate);

%plc.environment.AddObject(redCrate);

plc.environment.AddObject(redPen);

plc.environment.AddObject(bluePen);

plc.environment.AddObject(pencil1);

plc.environment.AddObject(pencil2);

plc.environment.Display();

% Create the environment
%environment = Environment(blueCrate, yellowCrate, redCrate, redPen, bluePen, pencil1, pencil2, table);

% Create the Dobot
%Dobot1 = Dobot('BasePose', eye(4)*transl(0,0,table.dimensions(1,3)));
%Dobot1.GenerateLinearRail([-0.45,0,table.dimensions(1,3)]);

% Add the Dobot to the environment
%environment.AddRobot(Dobot1);

% Create Camera
%camera = RGBCamera('CentrePose', transl(0, 0, 2.5) * troty(pi));

% Create the Master Controller and tell it about the environment
%plc = GlobalController(environment, camera);
%plc.environment.Display();

% Initialise the simulation
plc.Init();

disp('Initialisation Complete');
plc.Run();
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
%% Initialisation
close all;
clc;
clear;
try
    delete(camera);
end
 pause(5);
% Create all the objects in the environment
blueCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'blueCrate.ply', 'Pose', transl(0.75, 0.2, 0.8911), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'b');
yellowCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'yellowCrate.ply', 'Pose', transl(0.75, 0, 0.8911), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'y');
redCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'redCrate.ply', 'Pose', transl(0.75, -0.2, 0.8911), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'r');
redPen = EnvironmentObject('Type', 'target', 'ModelPath', 'redPen.ply', 'Pose', transl(0.45, 0.25, 0.8911), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'r');
bluePen = EnvironmentObject('Type', 'target', 'ModelPath', 'bluePen.ply', 'Pose', transl(0, -0.2, 0.8911), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'b');
pencil1 = EnvironmentObject('Type', 'target', 'ModelPath', 'pencil.ply', 'Pose', transl(0.15, 0.2, 0.8911), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'y');
pencil2 = EnvironmentObject('Type', 'target', 'ModelPath', 'pencil.ply', 'Pose', transl(-0.15, -0.3, 0.8911), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'y');
table = EnvironmentObject('Type', 'foundation', 'ModelPath', 'table.ply', 'Pose', transl(0, 0, 0), 'Dimensions', [2.1956 1.0097 0.8911], 'GeneralColour', 'r');

% Create the environment
environment = Environment(blueCrate, yellowCrate, redCrate, redPen, bluePen, pencil1, pencil2, table);

% Create the Dobot
Dobot1 = Dobot('BasePose', eye(4)*transl(0,0,table.dimensions(1,3)));
Dobot1.GenerateLinearRail([-0.45,0,0.8911]);

% Add the Dobot to the environment
environment.AddRobot(Dobot1);

% Create Camera
camera = RGBCamera('CentrePose', transl(0, 0, 2.5) * troty(pi));

% Create the Master Controller and tell it about the environment
plc = GlobalController(environment, camera);

% Initialise the simulation
plc.Init();

disp('Initialisation Complete');

plc.Run();

%% Advanced Teach Demo
close all;
clear;
clc;

robot = Dobot();
robot.GenerateLinearRail([0,0,0]);
robot.Display();
camlight;
axis equal;

robot.AdvancedTeach();
%% Safety Demo
close all;
clear;
clc;


blueCrate = EnvironmentObject('ModelPath', 'blueCrate.ply', 'Pose', transl(-0.75, 0, 0.8911), 'Dimensions', [0 0 0.0664], 'GeneralColour', 'b');
redPen = EnvironmentObject('ModelPath', 'redPen.ply', 'Pose', transl(0.75, 0.35, 0.8911), 'Dimensions', [0 0 0.0664], 'GeneralColour', 'r');


camera = RGBCamera('CentrePose', transl(0, 0, 2.5) * troty(pi));
camera.DisplayCamera();
hold on;
camera.AddObject(blueCrate);
camera.AddObject(redPen);
blueCrate.Display();


camera.PlotObjects();
camera.LocateObjects();
%% Simulation
close all;
clear;
clc;

PLC = GlobalController();
PLC.Init(); %Display all objects

disp("Press ENTER to begin simulation");
pause();

PLC.Run();
%% 
% close all;
% clear;
% clc;
% robot = Dobot();
% robot.Display();
% camlight;
% axis equal;

robot = plc.environment.robot;

q0 = robot.model.getpos
EEPose0 = robot.model.fkine(q0)

EEPose1 = EEPose0 * transl(0, 0.05, 0.3)
q1 = robot.GenerateTargetJointAngles(EEPose1)
steps = 50;
s = lspb(0,1,steps);
qMatrix = nan(steps,5);


for i = 1:steps
    qMatrix(i,:) = (1-s(i))*q0 + s(i)*q1;
end

pause();
for i = 1:steps
   robot.model.animate(qMatrix(i, :));
   drawnow();
end

%% Classes
% Dobot
% Object
% Camera
% Environment
% GlobalController


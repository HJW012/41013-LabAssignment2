%% Advanced Teach Demo
close all;
clear;
clc;

robot = Dobot();
robot.GenerateLinearRail();
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
close all;
clear;
clc;
robot = Dobot();
robot.Display();
camlight;
axis equal;

q0 = robot.model.getpos;
EEPose0 = robot.model.fkine(q0);

EEPose1 = transl(0.2, 0, 0);
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


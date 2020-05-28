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


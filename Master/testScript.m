%% 
close all;
clear;
clc;

table = EnvironmentObject('Type', 'foundation', 'ModelPath', 'table.ply', 'Pose', transl(0, 0, 0), 'Dimensions', [2.1956 1.0097 0.8911], 'GeneralColour', 'r');
redPen = EnvironmentObject('Type', 'target', 'ModelPath', 'redPen.ply', 'Pose', transl(0, 0.25, 0), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'r');
%table.Display();

redPen.Display();
hold on;
robot = Dobot('BasePose', eye(4)*transl(0,0,table.dimensions(1,3)));
robot.GenerateLinearRail([-0.3,0,0]);
robot.Display();
robot.model.base(1, 4) = 0;
robot.model.animate(robot.model.getpos);
drawnow();
%pause();
robot.model.base
q = robot.GenerateTargetJointAngles2(robot.model.base, redPen.pose);
rad2deg(q)
robot.model.animate(q);
%robot.MoveToTargetLinearRail2(redPen.pose);
%rad2deg(robot.GenerateTargetJointAngles(redPen.pose))
%q = robot.GenerateTargetJointAngles2(robot.model.base, redPen.pose);
%function MoveToTargetLinearRail2(self, targetPose)
%robot.model.animate(q);
%% %% 
close all;
clear;
clc;

table = EnvironmentObject('Type', 'foundation', 'ModelPath', 'table.ply', 'Pose', transl(0, 0, 0), 'Dimensions', [2.1956 1.0097 0.8911], 'GeneralColour', 'r');
redPen = EnvironmentObject('Type', 'target', 'ModelPath', 'redPen.ply', 'Pose', transl(0, 0.2, 0), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'r');
%table.Display();

redPen.Display();
hold on;
robot = Dobot('BasePose', eye(4)*transl(0,0,0.051));
robot.Display();
q = robot.GenerateTargetJointAngles2(robot.model.base, redPen.pose);
pause();
robot.model.animate(q);
%% Another test     
close all;
clear;
clc;

redPen = EnvironmentObject('Type', 'target', 'ModelPath', 'redPen.ply', 'Pose', transl(0, 0.23, 0), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'r');
redPen.SetPose(transl(0, 0.23, 0) * trotz(pi/4));
redPen.Display();
drawnow();
hold on;
robot = Dobot('BasePose', eye(4)*transl(0,0,0));
robot.Display();
q0 = robot.model.getpos;
upper = redPen.pose * transl(0, 0, 0.065) * trotx(-pi);
[q1 err1 exitflag1] = robot.model.ikcon(upper, q0);
[q2 err2 exitflag2] = robot.model.ikcon(redPen.pose * trotx(-pi), q1);
pose3 = transl(0, 0.2, 0.1) * trotx(-pi);
[q3 err3 exitflag3] = robot.model.ikcon(pose3, q2);
pose4 = transl(0, -.2, 0.1) * trotx(-pi);
[q4 err4 exitflag4] = robot.model.ikcon(pose4, q3);


s = lspb(0, 1, 50);
qMatrix1 = nan(50, 5);
qMatrix2 = nan(50, 5);
qMatrix3 = nan(50, 5);
qMatrix4 = nan(50, 5);
for i = 1:50
    qMatrix1(i,:) = (1 - s(i))*q0 + s(i)*q1;
    qMatrix2(i,:) = (1 - s(i))*q1 + s(i)*q2;
    qMatrix3(i,:) = (1 - s(i))*q2 + s(i)*q3;
    qMatrix4(i,:) = (1 - s(i))*q3 + s(i)*q4;
end

pause();
for i = 1:50
   robot.model.animate(qMatrix1(i, :));
   drawnow(); 
end
pause();
for i = 1:50
   robot.model.animate(qMatrix2(i, :));
   drawnow(); 
end
%% GUI Test
close all;
clear;
clc;

test = gui_class_example();


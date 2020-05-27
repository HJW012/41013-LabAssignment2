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
clear all;
clc;
robot = Dobot();
robot.Display();
camlight;
axis equal;

q0 = robot.model.getpos;
pose1 = transl(0.25, 0, 0.05) * troty(pi/2);
q1 = robot.model.ikcon(pose1, q0);
pause();
robot.model.animate(q1);
drawnow();

%Dobot_1 = Dobot();
%animate(Dobot_1.model,deg2rad( [-90,-60,100,-130,-90, 0]));

close all;
clc;
clear;

% Generate Serial Link based on derived DH parameters
L1 = Link('d',0.135,'a',0,'alpha',-pi/2,'qlim', deg2rad([-135,135]), 'offset', 0);
L2 = Link('d',0,'a',0.139,'alpha',0,'qlim', deg2rad([0,90]), 'offset', -pi/2);
L3 = Link('d',0,'a',0.160,'alpha',0,'qlim', deg2rad([-5,170]), 'offset', 0);
L4 = Link('d',0,'a',0.05,'alpha',-pi/2,'qlim', deg2rad([-90,90]), 'offset', 0);
L5 = Link('d',0.025,'a',0,'alpha',0,'qlim', deg2rad([-85,85]), 'offset', 0);

Dobot_1 = SerialLink([L1 L2 L3 L4 L5],'name', 'MyDobot');
workspace = [-1 1 -1 1 -1 1]; 
scale = 0.5;        
q = deg2rad([0, 60, 65, -35, 0]);                                                     % Create a vector of initial joint angles        
Dobot_1.plot(q,'workspace',workspace,'scale',scale);

Dobot_1.teach();

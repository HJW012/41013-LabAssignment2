close all;
clear;
clc;

tb = TestClass2;
rtt = TestClass1(tb);
%% 
close all;
clear;
clc;

id = 2; % Second controller connected to device - (PS3 controller is 1 so emulated XBox controller is 2)
joy = vrjoystick(id);

tb = TestClass2;
rtt = TestClass1(tb);

avgIndex = 1;
sum = 0;
average = 0;
iteration = 5;
while (1)
    [axes, buttons, povs] = read(joy);
    
   
    if (avgIndex >= iteration)
        average = sum / 5;
        avgIndex = 0;
        sum = 0;
    else
        sum = sum + axes(1);
        avgIndex = avgIndex + 1;
    end
    
    tb.OnStateChange(average);
end
%% Testing RRMC
close all;
clear;
clc;

mdl_puma560;
p560.plot(qn);
J0 = p560.jacob0(qn, 'eul') % Maps joint velocity to end effector spacial velocity exoressed in world coordinate frame
% 'eul' gets angular vels in euler form
%Rows correspond to cartesian Degrees of Freedom (x, y, z, xrot, yrot,
%xrot)
%Columns correspond to joints - they are the end effector spacial
%vellocities

Jn = p560.jacobn(qn); %Expressed in end effector coordinate frame




%% Jacobian for under-actuated robot - Dobot
close all;
clear;
clc;

robot = Dobot();

qn = robot.model.getpos;

J = jacob0(robot.model, qn);
Jxy = J(1:5, :);
qd = inv(Jxy)* [0.1 0 0 0 0]';
xd = J*qd;
xd'
%Jxy = J(1:2)
%% Horizontal J4 movement
close all;
clear;
clc;

robot = Dobot();
q0 = robot.model.getpos;
EEPose0 = robot.model.fkine(q0);

EEPose1 = transl(0.25, 0, 0.1);

%{
a2 = 0.139;
a3 = 0.16;
x = EEPose1(1, 4);
y = EEPose1(2, 4);
z = EEPose1(3, 4);
%}
syms x y z q2 q3 a2 a3;
f1 = a2*sin(q2)+a3*cos(q3) == sqrt(x^2+y^2);
f2 = a2*cos(q2)-a3*sin(q3) == z;
solution = solve([f1,f2],[q2,q3]);
solution.q2
solution.q3

q2sym1 = (subs(solution.q2(1), {x, y, z, a2, a3}, {0.25 0 0.1 0.139 0.16}))
q2sym2 = (subs(solution.q2(2), {x, y, z, a2, a3}, {0.25 0 0.1 0.139 0.16}))
q21 = vpa(q2sym1)
q22 = vpa(q2sym2)
if robot.model.qlim(2, 1) <= q21 && q21 <= robot.model.qlim(2, 2)
   q2 = q21; 
elseif robot.model.qlim(2, 1) <= q22 && q22 <= robot.model.qlim(2, 2)
    q2 = q22;
end

q2
    
EEPose = robot.model.fkine(robot.model.getpos);
targetX = EEPose(1, 4);
targetY = EEPose(2, 4);

theta = atan2(targetY-robot.model.base(2, 4), targetX-robot.model.base(1, 4));
joint4X = targetX - 0.05*cos(theta);
joint4Y = targetY - 0.05*sin(theta);


%% Distance Formula
function distance = dist(x1, y1, x2, y2)
    distance = abs( sqrt( (x1-x2)^2 + (y1-y2)^2 ) );
end
%% Angle Formula
function theta = angle1(x1, y1, x2, y2)
    theta = abs(atan((y2-y1)/(x2-x1)));
end

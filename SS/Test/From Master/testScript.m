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

%% Testing collision Detection
close all;
clc;
clear;


Dobot_1 = Dobot('BasePose', eye(4));

centrePoint = [0,0,0];

radii = zeros(5,3);

for i = 1:6
    if 1 < i
       radii(i, :) = [(0.06 + 0.5*Dobot_1.model.links(i-1).a),0.06,(0.06 + 0.5*Dobot_1.model.links(i-1).d)];
       [X,Y,Z] = ellipsoid( centrePoint(1), centrePoint(2), centrePoint(3), radii(i,1), radii(i,2), radii(i,3) );     
    elseif i <= 1
        radii(i, :) = [(0.06 + 0.5*Dobot_1.model.links(i).a),0.06,(0.06 + 0.5*Dobot_1.model.links(i).d)];
        [X,Y,Z] = ellipsoid( centrePoint(1), centrePoint(2), centrePoint(3), radii(i,1), radii(i,2), radii(i,3) );     
    
    end
   
    Dobot_1.model.points{i} = [X(:),Y(:),Z(:)];
    warning off
    Dobot_1.model.faces{i} = delaunay(Dobot_1.model.points{i});    
    warning on;
end

Dobot_1.model.plot3d(deg2rad([45, 27, 64, -70, 0]));

%Dobot_1.teach();

hold on;
axis equal;

obstacle = EnvironmentObject('Type', 'obstacle', 'ModelPath', 'obstacleBall.ply', 'Pose', transl(0.1, 0.1, 0.2), 'Dimensions', [0.1 0.1 0.1], 'GeneralColour', 'y');
obstacle.SetPose(obstacle.pose);

obstaclePoints = obstacle.modelMesh.Vertices;
         
q1 = deg2rad([45, 27, 64, -70, 0]);
q2 = deg2rad([-25, 70, 23, -10, 0]);

steps = 50;

s = lspb(0,1,steps);
qMatrix = nan(steps,5);
for i = 1:steps
    qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
    
    qMatrix(i,4) = (0.5*pi) - qMatrix(i,2) - qMatrix(i,3);
end

for j = 1:1:size(qMatrix,1)
    position = qMatrix(j,:);
    
    disp([num2str(j), 'th joint angle in trajectoy']);
    
    check = CheckEllipsoidCollision(obstaclePoints, position, Dobot_1, radii);
    
    if check == 1
        disp('Collision detected in trajectory, human intervention required to remove obstruction');
                
        break;
    end
    pause(0.01);
end







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
%% 
close all;
clear;
clc;

pencil2 = EnvironmentObject('Type', 'target', 'ModelPath', 'pencil.ply', 'Pose', transl(0, -0.3, 0), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'y');
robot = Dobot('BasePose', eye(4)*transl(-0.0865,0,0.051));
bluePen = EnvironmentObject('Type', 'target', 'ModelPath', 'bluePen.ply', 'Pose', transl(-0.0865, -0.3012, 0), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'b');

robot.Display();
hold on;
bluePen.Display();

q0 = robot.model.getpos;
wayPoint = bluePen.pose * transl(0, 0, 0.05);
[q1 err exitFlag] = robot.model.ikcon(wayPoint * trotx(pi), q0);
[q2 err exitFlag] = robot.model.ikcon(bluePen.pose * trotx(pi), q1);
s = lspb(0, 1, 50);

qMatrix1 = nan(50, 5);
for i = 1:50
    qMatrix1(i,:) = (1 - s(i))*q0 + s(i)*q1;
    qMatrix2(i,:) = (1 - s(i))*q1 + s(i)*q2;
end

pause();
for i = 1:50
   robot.model.animate( qMatrix1(i, :));
   drawnow();
end

pause();
for i = 1:50
   robot.model.animate( qMatrix2(i, :));
   drawnow();
end
%% GUI Test
close all;
clear;
clc;

test = gui_class_example();



%% GetAlgebraicDist
% determine the algebraic distance given a set of points and the center
% point and radii of an elipsoid
% *Inputs:* 
%
% _points_ (many*(2||3||6) double) x,y,z cartesian point
%
% _centerPoint_ (1 * 3 double) xc,yc,zc of an ellipsoid
%
% _radii_ (1 * 3 double) a,b,c of an ellipsoid
%
% *Returns:* 
%
% _algebraicDist_ (many*1 double) algebraic distance for the ellipsoid

function algebraicDist = GetAlgebraicDist(points, centrePoint, radii)

algebraicDist = ((points(:,1)-centrePoint(1))/radii(1)).^2 ...
              + ((points(:,2)-centrePoint(2))/radii(2)).^2 ...
              + ((points(:,3)-centrePoint(3))/radii(3)).^2;
end


%% CheckEllipsoidCollision

function collisionCheck = CheckEllipsoidCollision(obstaclePoints, jointAngles, robot, radii)
    centrePoint = [0 0 0];
    tr = zeros(4,4,robot.model.n+1);
    tr(:,:,1) = robot.model.base;
    L = robot.model.links;
    for i = 1 : robot.model.n
        tr(:,:,i+1) = tr(:,:,i) * trotz(jointAngles(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
    end

    % Go through each ellipsoid
    for i = 1: size(tr,3)
        pointsAndOnes = [inv(tr(:,:,i)) * [obstaclePoints,ones(size(obstaclePoints,1),1)]']';
        updatedPoints = pointsAndOnes(:,1:3);
        radius = radii(i,:);
        algebraicDist = GetAlgebraicDist(updatedPoints, centrePoint, radius);
        pointsInside = find(algebraicDist < 1);
        if 0 < pointsInside
            collisionCheck = 1;
            disp(['There are ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);
            break;
        else
            collisionCheck= 0;
        end
    
    end
end
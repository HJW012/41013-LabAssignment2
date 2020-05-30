%% Ellipsoid Collision Checking Calculated Trajectory
close all;
clc;
clear;

Dobot_1 = Dobot('BasePose', eye(4)*transl(0, 0, 0));
Dobot_1.Display();
     
q = deg2rad([45, 27, 64, -70, 0]);     
%Dobot_1.plot(q,'workspace',workspace,'scale',scale);
%Dobot_1.base = eye(4) * transl(1, 0.5, 0.8911);

centrePoint = [0,0,0];

radii = zeros(5,3);

for i = 1:6
    if 1 < i
       radii(i, :) = [(0.06 + 0.5*Dobot_1.links(i-1).a),0.06,(0.06 + 0.5*Dobot_1.links(i-1).d)];
       [X,Y,Z] = ellipsoid( centrePoint(1), centrePoint(2), centrePoint(3), radii(i,1), radii(i,2), radii(i,3) );     
    elseif i <= 1
        radii(i, :) = [(0.06 + 0.5*Dobot_1.links(i).a),0.06,(0.06 + 0.5*Dobot_1.links(i).d)];
        [X,Y,Z] = ellipsoid( centrePoint(1), centrePoint(2), centrePoint(3), radii(i,1), radii(i,2), radii(i,3) );     
    
    end
   
    Dobot_1.points{i} = [X(:),Y(:),Z(:)];
    warning off
    Dobot_1.faces{i} = delaunay(Dobot_1.points{i});    
    warning on;
end

Dobot_1.plot3d(deg2rad([45, 27, 64, -70, 0]));

%Dobot_1.teach();

hold on;
axis equal;

% One side of the cube
[Y,Z] = meshgrid(-0.1:0.01:0.1,-0.1:0.01:0.1);
sizeMat = size(Y);
X = repmat(0.1,sizeMat(1),sizeMat(2));

% Combine one surface as a point cloud
cubePoints = [X(:),Y(:),Z(:)];

% Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
cubePoints = [ cubePoints ...
             ; cubePoints * rotz(pi/2)...
             ; cubePoints * rotz(pi) ...
             ; cubePoints * rotz(3*pi/2) ...
             ; cubePoints * roty(pi/2) ...
             ; cubePoints * roty(-pi/2)];    

% Plot the cube's point cloud
cube1Points = cubePoints + repmat([0.3,0,0.2],size(cubePoints,1),1);
cube1_h = plot3(cube1Points(:,1),cube1Points(:,2),cube1Points(:,3),'r.');

cube2Points = cubePoints + repmat([0,-0.3,0.1],size(cubePoints,1),1);
cube2_h = plot3(cube2Points(:,1),cube2Points(:,2),cube2Points(:,3),'b.');

cubes = zeros(2646,3,2);
cubes(:,:,1) = cube1Points;
cubes(:,:,2) = cube2Points;

         
q1 = deg2rad([45, 27, 64, -70, 0]);
q2 = deg2rad([-25, 70, 23, -10, 0]);

steps = 50;

% Method 2: Trapezoidal Velocity Profile
s = lspb(0,1,steps);
qMatrix = nan(steps,5);
for i = 1:steps
    qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
    
    qMatrix(i,4) = (0.5*pi) - qMatrix(i,2) - qMatrix(i,3);
end

for j = 1:1:size(qMatrix,1)
    position = qMatrix(j,:);
    
    disp([num2str(j), 'th joint angle in trajectoy']);
    
    check = CheckEllipsoidCollision(cubes, position, Dobot_1, radii);
    
    if check == 1
        disp('Collision detected in trajectory, human intervention required to remove obstruction');
        
        Dobot_1.plot3d(position);
        
        break;
    end
    pause(0.01);
end

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

function collisionCheck = CheckEllipsoidCollision(cubes, jointAngles, robot, radii)
    centrePoint = [0 0 0];
    tr = zeros(4,4,robot.n+1);
    tr(:,:,1) = robot.base;
    L = robot.links;
    for i = 1 : robot.n
        tr(:,:,i+1) = tr(:,:,i) * trotz(jointAngles(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
    end

    % Go through each ellipsoid
    for i = 1: size(tr,3)
        for j = 1:size(cubes,3)
            cubePointsAndOnes = [inv(tr(:,:,i)) * [cubes(:,:,j),ones(size(cubes(:,:,j),1),1)]']';
            updatedCubePoints = cubePointsAndOnes(:,1:3);
            %plot3(updatedCubePoints(:,1),updatedCubePoints(:,2),updatedCubePoints(:,3),'b.');
            radius = radii(i,:);
            algebraicDist = GetAlgebraicDist(updatedCubePoints, centrePoint, radius);
            pointsInside = find(algebraicDist < 1);
            if 0 < pointsInside
                collisionCheck = 1;
                break;
            else
                collisionCheck= 0;
            end
            %disp(['There are ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);
        end
        if 0 < collisionCheck
            break;
        end       
    end

end
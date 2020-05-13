close all;
clc;
clear;

Dobot_1 = Dobot([0,0,0]);
%animate(Dobot_1.model,deg2rad( [-90,-60,100,-130,-90, 0]));

Dobot_1.model.teach();


%% 
close all;
clc;
clear;

% Generate Serial Link based on derived DH parameters
L1 = Link('d',0.135,'a',0,'alpha',-pi/2,'qlim', deg2rad([-135,135]), 'offset', 0);
L2 = Link('d',0,'a',0.139,'alpha',0,'qlim', deg2rad([5,80]), 'offset', -pi/2);
L3 = Link('d',0,'a',0.160,'alpha',0,'qlim', deg2rad([15,170]), 'offset', 0);
L4 = Link('d',0,'a',0.05,'alpha',-pi/2,'qlim', deg2rad([-90,90]), 'offset', 0);
L5 = Link('d',0.025,'a',0,'alpha',0,'qlim', deg2rad([-85,85]), 'offset', 0);

Dobot_1 = SerialLink([L1 L2 L3 L4 L5],'name', 'MyDobot');
workspace = [-1 1 -1 1 0 1]; 
scale = 0.5;        
q = deg2rad([45, 27, 64, -70, 0]);     
Dobot_1.plot(q,'workspace',workspace,'scale',scale);

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

Dobot_1.teach();

hold on;

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
        disp('Collision detected in trajectory');
        
        Dobot_1.plot3d(position);
        
        break;
    end
    pause(0.01);
end




%%
% 2.2     Define our first end-effector pose as a 4x4 Homogeneous Transformation Matrix: 
T1 = transl(0.5,-0.4,0.5); 
% 2.3     Solve the inverse kinematics to get the required joint angles 
q1 = deg2rad([0, 0, 0, 0, 0]);

% 2.4     Define the second end-effector pose as a 4x4 Homogeneous Transformation Matrix: 
T2 = transl(0.1,0.3,0.4); 
% 2.5     Solve the inverse kinematics to get the required joint angles 
q2 = deg2rad([0, 85, 33, -29, 0]);

steps = 50;

% Method 2: Trapezoidal Velocity Profile
s = lspb(0,1,steps);
qMatrix = nan(steps,5);
for i = 1:steps
    qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
    
    qMatrix(i,4) = (0.5*pi) - qMatrix(i,2) - qMatrix(i,3);
end

Dobot_1.plot(qMatrix,'trail','r-') 

%% 
q1 = deg2rad([0, 0, 0, 0, 0]);

q2 = deg2rad([0, 85, 33, -29, 0]);

q3 = deg2rad([0, 40, 120, -70, 0]);


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
    animate(Dobot_1.model, position)

    drawnow()
    pause(0.01);
end

pause(2);

s = lspb(0,1,steps);
qMatrix = nan(steps,5);
for i = 1:steps
    qMatrix(i,:) = (1-s(i))*q2 + s(i)*q3;
    
    qMatrix(i,4) = (0.5*pi) - qMatrix(i,2) - qMatrix(i,3);
end

for j = 1:1:size(qMatrix,1)
    position = qMatrix(j,:);
    animate(Dobot_1.model, position)

    drawnow()
    pause(0.01);
end

%% 

stepRads = deg2rad(10);
qlim = Dobot_1.qlim;
% Don't need to worry about joint 6
pointCloudeSize = prod(floor((qlim(1:5,2)-qlim(1:5,1))/stepRads + 1));
pointCloud = zeros(pointCloudeSize,3);
counter = 1;
tic

for q1 = qlim(1,1):stepRads:qlim(1,2)
    for q2 = qlim(2,1):stepRads:qlim(2,2)
        for q3 = qlim(3,1):stepRads:qlim(3,2)
            q4 = (0.5*pi) - q2 - q3;
                q5 = 0;
                q = [q1,q2,q3,q4,q5];
                tr = Dobot_1.fkine(q);                        
                pointCloud(counter,:) = tr(1:3,4)';
                counter = counter + 1; 
                if mod(counter/pointCloudeSize * 100,1) == 0
                    disp(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                end
        end
    end
end

plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');

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
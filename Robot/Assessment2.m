close all;
clc;
clear;

Dobot_1 = Dobot([0,0,0]);
animate(Dobot_1.model, deg2rad([45, 27, 85, -22, 0]))

%Dobot_1.model.teach();

%% Environment Initialisation
close all;
clc;
clear;

hold on;

table = Objects('table.ply', [0 0 0], 0.8911);

yellowCrate = Objects('yellowCrate.ply', [-0.75 -0.25 0.8911], 0.0664);
    
blueCrate = Objects('blueCrate.ply', [-0.75 0 0.8911], 0.0664);

redCrate = Objects('redCrate.ply', [-0.75 0.25 0.8911], 0.0664);

redPen = Objects('redPen.ply', [0.75 0.35 0.8911], 0.0664);

bluePen = Objects('bluePen.ply', [0.15 -0.1 0.8911], 0.0664);

pencil1 = Objects('pencil.ply', [0.3 0.2 0.8911], 0.0664);

pencil2 = Objects('pencil.ply', [0 -0.3 0.8911], 0.0664);
    
Dobot_1 = Dobot([0,0,0.8911]);
animate(Dobot_1.model, deg2rad([45, 27, 64, -70, 0]));

% Build Camera
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'OverheadCamera');

% Set centre of camera
Tc0 = eye(4) * transl(0,0,2.5) * troty(pi);

% plot camera
cam.T = Tc0;

% this is the 'external' view of the camera
%plot_sphere(P, 0.05, 'b');
cam.plot_camera('Tcam',Tc0,'label','scale',0.15);

cam.clf();
cam.plot(pencil1.pose(1:3,4), 'Tcam', Tc0, 'o'); % create the camera view
cam.hold(true);
cam.plot(pencil2.pose(1:3,4), 'Tcam', Tc0, 'o'); % create the camera view
cam.plot(redPen.pose(1:3,4), 'Tcam', Tc0, '*'); % create the camera view
cam.plot(bluePen.pose(1:3,4), 'Tcam', Tc0, '*'); % create the camera view

disp(pencil1.pose);
uv = cam.project(pencil1.pose(1:3,4), 'Tcam', Tc0, 'o');

Z = cam.T(3,4) - table.height;

% Calculate object's X and Y position from the centre using the data from
% the camera
posX = cam.T(1,4) - ((uv(1) - cam.pp(1))*Z)/(10000*cam.f);
posY = cam.T(2,4) + ((uv(2) - cam.pp(2))*Z)/(10000*cam.f);

%% Ellipsoid Collision Checking Calculated Trajectory
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

%Dobot_1.teach();

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
        disp('Collision detected in trajectory, human intervention required to remove obstruction');
        
        Dobot_1.plot3d(position);
        
        break;
    end
    pause(0.01);
end


%% Light Curtain
close all;
clc;
clear;

Dobot_1 = Dobot([0,0,0]);
%animate(Dobot_1.model,deg2rad( [-90,-60,100,-130,-90, 0]));

%Dobot_1.model.teach();

tableWidth = 0.6;
tableLength = 1;

hold on;

% [Y,Z] = meshgrid((Dobot_1.model.base(2,4) - (0.5*tableLength)):0.05:(Dobot_1.model.base(2,4) + (0.5*tableLength)),Dobot_1.model.base(3,4):0.05:(Dobot_1.model.base(3,4)+0.8));
% sizeMat = size(Y);
% X = repmat((Dobot_1.model.base(1,4) - (0.5*tableWidth)),sizeMat(1),sizeMat(2));
% surf(X,Y,Z, 'FaceAlpha', 0.5, 'EdgeColor', 'none', 'FaceColor', 'red');

x = (Dobot_1.model.base(1,4) - (0.5*tableWidth));
z = (Dobot_1.model.base(3,4)+0.8);

lightCurtain = [];

for y = (Dobot_1.model.base(2,4) - (0.5*tableLength)):0.02:(Dobot_1.model.base(2,4) + (0.5*tableLength))
    lightCurtain = [lightCurtain; x, y, Dobot_1.model.base(3,4), x, y, z];
end

x = (Dobot_1.model.base(1,4) + (0.5*tableWidth));
for y = (Dobot_1.model.base(2,4) - (0.5*tableLength)):0.02:(Dobot_1.model.base(2,4) + (0.5*tableLength))
    lightCurtain = [lightCurtain; x, y, Dobot_1.model.base(3,4), x, y, z];
end

y = (Dobot_1.model.base(2,4) - (0.5*tableLength));
for x = (Dobot_1.model.base(1,4) - (0.5*tableWidth)):0.02:(Dobot_1.model.base(1,4) + (0.5*tableWidth))
    lightCurtain = [lightCurtain; x, y, Dobot_1.model.base(3,4), x, y, z];
end

y = (Dobot_1.model.base(2,4) + (0.5*tableLength));
for x = (Dobot_1.model.base(1,4) - (0.5*tableWidth)):0.02:(Dobot_1.model.base(1,4) + (0.5*tableWidth))
    lightCurtain = [lightCurtain; x, y, Dobot_1.model.base(3,4), x, y, z];
end

for i = 1:1:size(lightCurtain, 1)
    point1 = [lightCurtain(i,1), lightCurtain(i,2), lightCurtain(i,3)];
    point2 = [lightCurtain(i,4), lightCurtain(i,5), lightCurtain(i,6)];
    curtain_h = plot3([point1(1),point2(1)],[point1(2),point2(2)],[point1(3),point2(3)],'r');
end

% After saving in blender then load the triangle mesh
[faces,v,data] = plyread('hand.ply','tri');

% Get vertex count
handVertexCount = size(v,1);

% Move centre point to origin
midPoint = sum(v)/handVertexCount;
handVerts = v - repmat(midPoint,handVertexCount,1);

% Create a transform to describe the location (at the origin, since it's centered
handPose = eye(4)*transl(-0.3, 0, 0.15);

% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Then plot the trisurf
handMesh_h = trisurf(faces,handVerts(:,1)-0.3,handVerts(:,2), handVerts(:,3)+0.15 ...
    ,'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');

v3d = zeros(size(handVerts,1),3);

for i = 1:size(handVerts,1)
    v3d(i, 1) = handVerts(i,1) + handPose(1,4);
    v3d(i, 2) = handVerts(i,2) + handPose(2,4);
    v3d(i, 3) = handVerts(i,3) + handPose(3,4);
end

faceNormals = zeros(size(faces,1),3);
for faceIndex = 1:size(faces,1)
    v1 = v3d(faces(faceIndex,1)',:);
    v2 = v3d(faces(faceIndex,2)',:);
    v3 = v3d(faces(faceIndex,3)',:);
    faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
end

%Using Rectangular Prism
% centrepnt = [0.3,0,0.3];
% side = 0.15;
% plotOptions.plotFaces = true;
% plotOptions.plotVerts = true;
% plotOptions.plotEdges = true;
% [v,faces,faceNormals] = RectangularPrism(centrepnt-side/2, centrepnt+side/2,plotOptions);

for i = 1:1:size(lightCurtain, 1)
    point1 = [lightCurtain(i,1), lightCurtain(i,2), lightCurtain(i,3)];
    point2 = [lightCurtain(i,4), lightCurtain(i,5), lightCurtain(i,6)];
    
    for faceIndex = 1:size(faces,1)
        vertOnPlane = v3d(faces(faceIndex,1)',:);
        [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,point1,point2); 
        if check == 1 && IsIntersectionPointInsideTriangle(intersectP,v3d(faces(faceIndex,:)',:))
            plot3(intersectP(1),intersectP(2),intersectP(3),'g*');
            disp('Intersection');
        end
    end    
end

axis equal;


%% Driving robot using target joint angles
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
q = deg2rad([-45, 27, 85, -22, 0]);     
Dobot_1.plot(q,'workspace',workspace,'scale',scale);

% 2.2     Define our first end-effector pose as a 4x4 Homogeneous Transformation Matrix: 
T1 = transl(0.5,-0.4,0.5); 
% 2.3     Solve the inverse kinematics to get the required joint angles 
q1 = deg2rad([-45, 27, 85, -22, 0]);

% 2.4     Define the second end-effector pose as a 4x4 Homogeneous Transformation Matrix: 
T2 = transl(0.1,0.3,0.4); 
% 2.5     Solve the inverse kinematics to get the required joint angles 
q2 = deg2rad([-90, 85, 33, -29, 0]);

steps = 50;

% Method 2: Trapezoidal Velocity Profile
s = lspb(0,1,steps);
qMatrix = nan(steps,5);
for i = 1:steps
    qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
    
    qMatrix(i,4) = (0.5*pi) - qMatrix(i,2) - qMatrix(i,3);
end

Dobot_1.plot(qMatrix,'trail','r-') 

%% Driving robot using target joint angles including Emergency Stop Button
close all;
clc;
clear;

hold on;

table = Objects('table.ply', [0 0 0], 0.8911);

yellowCrate = Objects('yellowCrate.ply', [-0.75 -0.25 0.8911], 0.0664);
    
blueCrate = Objects('blueCrate.ply', [-0.75 0 0.8911], 0.0664);

redCrate = Objects('redCrate.ply', [-0.75 0.25 0.8911], 0.0664);

redPen = Objects('redPen.ply', [0.75 0.35 0.8911], 0.0664);

bluePen = Objects('bluePen.ply', [0.15 -0.1 0.8911], 0.0664);

pencil1 = Objects('pencil.ply', [0.3 0.2 0.8911], 0.0664);

pencil2 = Objects('pencil.ply', [0 -0.3 0.8911], 0.0664);
    
Dobot_1 = Dobot([0,0,0.8911]);
animate(Dobot_1.model, deg2rad([45, 27, 64, -70, 0]));

q1 = deg2rad([45, 27, 64, -70, 0]);

q2 = deg2rad([0, 0, 0, 0, 0]);

q3 = deg2rad([0, 85, 33, -29, 0]);

q4 = deg2rad([0, 40, 120, -70, 0]);

q5 = deg2rad([45, 27, 64, -70, 0]);

qTrajectory = [q1; q2; q3; q4; q5];

steps = 50;

emergencyStop = 0;
lightCurtain = 0;

emergencyStop = EStop();

for k = 1:(size(qTrajectory,1) - 1)
    % Method 2: Trapezoidal Profile
    s = lspb(0,1,steps);
    qMatrix = nan(steps,5);
    for i = 1:steps
        qMatrix(i,:) = (1-s(i))*qTrajectory(k, :) + s(i)*qTrajectory(k+1,:);

        qMatrix(i,4) = (0.5*pi) - qMatrix(i,2) - qMatrix(i,3);
    end

    for j = 1:1:size(qMatrix,1)
        position = qMatrix(j,:);
                        
        while ((emergencyStop == 1) || (lightCurtain == 1))
            pause(0.01);
        end
        
        animate(Dobot_1.model, position)

        drawnow()
        pause(0.01);
    end

    pause(2);
        
end



%% Workspace Volume

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

%% IsIntersectionPointInsideTriangle
% Given a point which is known to be on the same plane as the triangle
% determine if the point is 
% inside (result == 1) or 
% outside a triangle (result ==0 )
function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

u = triangleVerts(2,:) - triangleVerts(1,:);
v = triangleVerts(3,:) - triangleVerts(1,:);

uu = dot(u,u);
uv = dot(u,v);
vv = dot(v,v);

w = intersectP - triangleVerts(1,:);
wu = dot(w,u);
wv = dot(w,v);

D = uv * uv - uu * vv;

% Get and test parametric coords (s and t)
s = (uv * wv - vv * wu) / D;
if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
    result = 0;
    return;
end

t = (uv * wu - uu * wv) / D;
if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
    result = 0;
    return;
end

result = 1;                      % intersectP is in Triangle
end
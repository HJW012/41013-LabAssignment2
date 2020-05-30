%% Light Curtain
close all;
clc;
clear;

Dobot_1 = Dobot('BasePose', eye(4)*transl(0,0,0));
Dobot_1.Display();
%animate(Dobot_1.model,deg2rad( [-90,-60,100,-130,-90, 0]));

%Dobot_1.model.teach();

tableWidth = 1.0097;
tableLength = 2.1956;

hold on;

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
handPose = eye(4)*transl(-0.5, 0, 0.15);

% Scale the colours to be 0-to-1 (they are originally 0-to-255
vertexColours = [data.vertex.red, data.vertex.green, data.vertex.blue] / 255;

% Then plot the trisurf
handMesh_h = trisurf(faces,handVerts(:,1)-0.5,handVerts(:,2), handVerts(:,3)+0.15 ...
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
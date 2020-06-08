%Dobot_1 = Dobot();
%animate(Dobot_1.model,deg2rad( [-90,-60,100,-130,-90, 0]));

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
workspace = [-1 1 -1 1 -1 1]; 
scale = 0.5;        
q = deg2rad([0, 60, 65, -35, 0]);                                                     % Create a vector of initial joint angles        
Dobot_1.plot(q,'workspace',workspace,'scale',scale);

Dobot_1.teach();


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
                    display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                end
        end
    end
end

plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');
%% Harrison Test
close all;
clear;
clc;

robot = Dobot();
drawnow();
camlight;
%{
for i = 1:15
   robot.plot(q);
end

for i = 1:15
   q(2) = q(2) - 5;
   robot.plot(q);
end

for i = 1:15
   q(3) = q(3) + 5;
   robot.plot(q);
end

for i = 1:15
   q(3) = q(3) - 5;
   robot.plot(q);
end

pause();
%}
q1 = robot.model.getpos();
pose1 = robot.model.fkine(q1);
pose2 = pose1;
pose2(3, 4) = 0.15;
q2 = robot.model.ikcon(pose2, q1);
pose3 = pose2;
pose3(1, 4) = 0.25;
q3 = robot.model.ikcon(pose3, q2);

verifyPose = true;

pause();

if verifyPose
    steps = 25;
    s = lspb(0, 1, steps);
    QMatrix1 = nan(steps, 5);
    QMatrix2 = nan(steps, 5);
    for i = 1:steps
        QMatrix1(i, :) = (1-s(i))*q1 + s(i)*q2;
        QMatrix2(i, :) = (1-s(i))*q2 + s(i)*q3;
    end

    for i = 1:steps
       robot.model.animate(QMatrix1(i, :));
       drawnow();
    end

    pause();

    for i = 1:steps
       robot.model.animate(QMatrix2(i, :));
       drawnow();
    end 
else
    disp("not gonna work mate");
end

%{

           
           steps = 25;
           s = lspb(0, 1, steps); %Linear segment with parabolic blend - gets trapezoidal path from 0 to 1 in 25 increments
           toAssemblyQMatrix = nan(3 * steps, 6); %Used for all joint states for travelling to target
           for i = 1:steps
               toAssemblyQMatrix(i, :) = (1-s(i))*q1 + s(i)*q2;
               toAssemblyQMatrix(steps + i, :) = (1-s(i))*q2 + s(i)*q3;
               toAssemblyQMatrix(2*steps + i, :) = (1-s(i))*q3 + s(i)*q4;
           end
           
           disp("Robot " + robSel + " moving to target 1");
           self.log.mlog = {self.log.DEBUG, 'Global Controller', "Robot " + robSel + " moving to target 1"};
           [QRows QCols] = size(toT1QMatrix); %Get number of rows (joint states) in total operation
           disp("End Effector pose: ");
           localRob.robot.model.fkine(toT1QMatrix(1, :)) %Display initial pose
           for i = 1:QRows %Animates robot to target 
               localRob.robot.model.animate(toT1QMatrix(i, :));
               EEPose = localRob.robot.model.fkine(toT1QMatrix(i, :));
               self.LogPose("Robot " + robSel + " EE Pose: ", EEPose);
               if self.CheckEstop(robSel)
                   warning("EStop Pressed - Simulation must be stopped and restarted");
                   self.log.mlog = {self.log.DEBUG, 'Global Controller', "EStop Pressed - Simulation must be stopped and restarted"};
                   edit(self.logPath);
                   pause();
               end
               drawnow();
           end
%}
%% Plotting Joints
close all;
clear;
clc;

robot = Dobot();
q = deg2rad([0, 60, 65, -35, 0]);   
robot.model.animate(q);
%robot.model.plot(q);
%% Linear rail
close all;
clear;
clc;

pose = transl(0, 0, 0);

[modelF, modelV, modelData] = plyread("LinearRail.ply", "tri");
                      vertexColours = [modelData.vertex.red, modelData.vertex.green, modelData.vertex.blue] / 255;
                      vertexCount = size(modelV, 1);
modelMesh = trisurf(modelF,modelV(:,1)+ pose(1,4),modelV(:,2) + pose(2,4), modelV(:,3) + pose(3,4),'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
                    
updatedPoints = [pose * [modelV, ones(vertexCount, 1)]']';
modelMesh.Vertices = updatedPoints(:, 1:3);
drawnow();
axis equal;
camlight;

%% Movement on linear rail - Using model downloaded from Dobot support website
close all;
clear;
clc;

linearRailHeight = 0.051;
robotBaseWidth = 0.157852;

linearRailPose = transl(0, 0, 0);
robotBasePose = transl(linearRailPose(1, 4)+robotBaseWidth/2, linearRailPose(2, 4), linearRailPose(3, 4) + linearRailHeight);

[modelF, modelV, modelData] = plyread("LinearRail.ply", "tri");
                      vertexColours = [modelData.vertex.red, modelData.vertex.green, modelData.vertex.blue] / 255;
                      vertexCount = size(modelV, 1);
modelMesh = trisurf(modelF,modelV(:,1)+ linearRailPose(1,4),modelV(:,2) + linearRailPose(2,4), modelV(:,3) + linearRailPose(3,4),'FaceVertexCData',vertexColours,'EdgeColor','interp','EdgeLighting','flat');
                    
updatedPoints = [linearRailPose * [modelV, ones(vertexCount, 1)]']';
modelMesh.Vertices = updatedPoints(:, 1:3);

robot = Dobot();
robot.model.base = robotBasePose;
q = deg2rad([90, 60, 65, -35, 0]);   
robot.model.animate(q);

railLength = 1.125;
totalTravelDist = railLength-robotBaseWidth;
axis equal;
camlight;
CloneFig(1, 2)

pause();

steps = 100;
for i =0:steps
    
    robot.model.base = transl(linearRailPose(1, 4)+robotBaseWidth/2 + i * totalTravelDist/steps, linearRailPose(2, 4), linearRailPose(3, 4) + linearRailHeight);
    q = robot.model.getpos;
    figure(2);
    robot.model.animate(q);
    drawnow();
    
    figure(2);
    robot.model.animate(q);
    drawnow();
end

for i =0:steps
    robot.model.base = transl(linearRailPose(1, 4) + railLength - robotBaseWidth/2 - i * totalTravelDist/steps, linearRailPose(2, 4), linearRailPose(3, 4) + linearRailHeight);
    q = robot.model.getpos;
    figure(2);
    robot.model.animate(q);
    drawnow();
    
    figure(2);
    robot.model.animate(q);
    drawnow();
end

%% Advanced Teach
close all;
clear;
clc;

robot = Dobot();
q = deg2rad([0, 60, 65, -35, 0]);   
robot.model.animate(q);
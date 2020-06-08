%% Global Controller Class
classdef GlobalController < handle
   properties
       environment;
       redDepositLocation;
       greenDepositLocation;
       blueDepositLocation;
       redTargetLocations;
       greenTargetLocations;
       blueTargetLocations;
       camera;
       robot;
       hand;
       obstacle;
       objects;
       emergencyStop = 0;
       targetPlacementWindow; %Window where targets can be placed within reach of Dobot - Need to implement
   end
   
   methods
       function self = GlobalController()% environment, camera)  

       end
       %% Simulation Setup
       function Setup(self)
          % Create fixed objects in the scene which do not move during
          % operation
          table = EnvironmentObject('Type', 'foundation', 'ModelPath', 'table.ply', 'Pose', transl(0, 0, 0), 'Dimensions', [2.1956 1.0097 0.8911]);
          blueCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'blueCrate.ply', 'Pose', transl(0.75, 0.2, 0.8911), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'b');
          yellowCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'yellowCrate.ply', 'Pose', transl(0.75, 0, 0.8911), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'g');
          redCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'redCrate.ply', 'Pose', transl(0.75, -0.2, 0.8911), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'r');
          cone1 = EnvironmentObject('Type', 'safety', 'ModelPath', 'cone.ply', 'Pose', transl(1.25, -0.75, 0));
          cone2 = EnvironmentObject('Type', 'safety', 'ModelPath', 'cone.ply', 'Pose', transl(-1.25, -0.75, 0));
          cone3 = EnvironmentObject('Type', 'safety', 'ModelPath', 'cone.ply', 'Pose', transl(1.25, 0.75, 0));
          cone4 = EnvironmentObject('Type', 'safety', 'ModelPath', 'cone.ply', 'Pose', transl(-1.25, 0.75, 0));
          estop = EnvironmentObject('Type', 'safety', 'ModelPath', 'EStop.ply', 'Pose', transl(0, 0.8, 0));
       
          % Create the Environment object and pass it all the fixed objects
          self.environment = Environment(blueCrate, yellowCrate, redCrate, cone1, cone2, cone3, cone4, estop);
          
          % Create the robot
          self.robot = Dobot(transl(0, 0, 0));
          %Generate the robot's linear rail
          self.robot.GenerateLinearRail([-0.45, 0, 0.8911]);
          
          % Add the robot to the environment
          self.environment.AddRobot(self.robot);
          
          % Add the table to the environment
          self.environment.AddObject(table);
          self.camera = RGBCamera('CentrePose', transl(0, 0, 2.5) * troty(pi));
          
          % Display the objects in the environment.
          self.environment.Display();
       end
       %% Insert Obstacle for collision detection
       % Function used to generate an obstacle to create a forced collision
       % with the robot to avoid
       function InsertObstacle(self, pose)
          % Create the ball obstacle
          self.obstacle = EnvironmentObject('Type', 'obstacle', 'ModelPath', 'obstacleBall.ply', 'Pose', pose, 'Dimensions', [0.1 0.1 0.1], 'GeneralColour', 'y');
          % Display the ball
          self.obstacle.Display();
          
          % Add the obstacle to the environment
          self.environment.AddObject(self.obstacle);
       end
       %% Remove Obstacle
       % Function used to remove an obstacle from the environment
       function RemoveObstacle(self)
          % Call the remove function for the object
          self.obstacle.Remove();
          % Update the pose of the obstacle object in the Environment
          self.environment.obstacleObjects{1}.pose = self.obstacle.pose;
       end
%        %% Add Environment
%        function AddEnvironment(self, environment)
%            self.environment = environment;
%        end
       %% Insert Light Curtain Obstruction
       % Function used to insert an object into the light curtain to
       % demonstrate the safety feature of the light curtain
       function InsertObstruction(self)
          % Create the hand mesh to be displayed inside the light curtain
          self.hand = EnvironmentObject('Type', 'misc', 'ModelPath', 'hand.ply','Pose', transl(-0.2, 0.45, self.environment.foundation.dimensions(1,3)+0.1) * trotz(-pi/2), 'Dimensions', [0.1734 0.0123 0.0124]); 
          % Display the hand
          self.hand.Display();
          
          % Add the object to the Environment
          self.environment.AddObject(self.hand);
       end
       %% Remove Light Curtain Obstruction
       % Function used to remove the light curtain obstruction
       function RemoveObstruction(self)
          % Call the remove function for the hand object
          self.hand.Remove(); 
       end
       %% Insert Target
       % Function called by the GUI to insert the targets into the
       % environment
       function InsertTarget(self, pose, colour)
          if strcmp(colour, 'r')
             object = EnvironmentObject('Type', 'target', 'ModelPath', 'redPen.ply', 'Pose', pose, 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'r');
          end
          
          if strcmp(colour, 'b')
             object = EnvironmentObject('Type', 'target', 'ModelPath', 'bluePen.ply', 'Pose', pose, 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'b');
          end
          
          if strcmp(colour, 'g')
             object = EnvironmentObject('Type', 'target', 'ModelPath', 'pencil.ply', 'Pose', pose, 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'g');
          end
          
          object.Display();
          self.environment.AddObject(object);
       end
       %% Run Simulation
       function Run(self)
           startingPose = self.environment.robot.model.base();
           
           disp('Calculating the Pick Up and Deposit Locations');
           
           % Calculate Pick Up and Deposit locations
           self.camera.LocateObjects();
                     
           % Determining if blobs are targets or deposit locations based on
           % blob area
           self.AssignCentroids();
           
           % Drive to pick up location on linear slide
           redSize = size(self.redTargetLocations);
           if redSize(1) > 0
               for i = 1:1:size(self.redTargetLocations, 3)
                   shortestDistance = 5;
                   objectIndex = 1;

                   targetPosition = self.redTargetLocations(1,4,i);
                   self.MoveToTargetLinearRail(targetPosition);

                   % Collect Object
                   targetPose = self.redTargetLocations(:,:,i);

                   for j = 1:size(self.environment.targets, 2)
                       distance = self.DistanceBetweenPoses(targetPose, self.environment.targets{j}.pose);
                       if distance < shortestDistance
                           objectIndex = j;
                           shortestDistance = distance;
                       end
                   end

                   if targetPose(2,4) < self.environment.robot.model.base(2,4)
                       %waypoint = deg2rad([-90, 60, 65, -35, 0]);
                       q0 = self.environment.robot.model.getpos;
                       waypoint = self.environment.robot.model.ikcon(targetPose * transl(0, 0, 0.1) * trotx(pi), q0);
                       self.MoveRobotArmJointAngles(waypoint);

                       targetPose = self.redTargetLocations(:,:,i) * trotx(pi);

                   else
                       %waypoint = deg2rad([90, 60, 65, -35, 0]);
                       q0 = self.environment.robot.model.getpos;
                       waypoint = self.environment.robot.model.ikcon(targetPose * transl(0, 0, 0.1) * trotx(pi), q0);
                       self.MoveRobotArmJointAngles(waypoint);

                       targetPose = self.redTargetLocations(:,:,i) * trotx(pi);
                   end

                   self.MoveRobotArm(targetPose);

                   disp('Target Pose: ');
                   disp(targetPose);
                   pose = self.environment.robot.model.fkine(self.environment.robot.model.getpos())
                   error = targetPose - pose

                   self.environment.robot.object = self.environment.targets{objectIndex};

                   % Return to deposit location
                   self.MoveRobotArmJointAngles(waypoint);
                   self.MoveRobotArmJointAngles(self.environment.robot.jointAngles);

                   targetPosition = self.redDepositLocation(1,4,1);
                   self.MoveToTargetLinearRail(targetPosition);

                   % Drop object
                   targetPose = self.redDepositLocation(:,:,1);
                    wayPoint = targetPose * transl(0, 0, 0.05);
                   self.MoveRobotArm(wayPoint * trotx(pi));
                   self.MoveRobotArm(targetPose * trotx(pi));

                   pose = self.environment.robot.model.fkine(self.environment.robot.model.getpos())

                   self.environment.robot.object = EnvironmentObject.empty;
                   self.MoveRobotArm(wayPoint * trotx(pi));

                   self.MoveRobotArmJointAngles(self.environment.robot.jointAngles);   
               end
           end    
           
           greenSize = size(self.greenTargetLocations);
           if greenSize(1) > 0
               for i = 1:1:size(self.greenTargetLocations, 3)
                   shortestDistance = 5;
                   objectIndex = 1;

                   targetPosition = self.greenTargetLocations(1,4,i);
                   self.MoveToTargetLinearRail(targetPosition);

                   % Collect Object
                   targetPose = self.greenTargetLocations(:,:,i);

                   for j = 1:size(self.environment.targets, 2)
                       distance = self.DistanceBetweenPoses(targetPose, self.environment.targets{j}.pose);
                       if distance < shortestDistance
                           objectIndex = j;
                           shortestDistance = distance;
                       end
                   end

                   if targetPose(2,4) < self.environment.robot.model.base(2,4)
                       %waypoint = deg2rad([-90, 60, 65, -35, 0]);

                       q0 = self.environment.robot.model.getpos;
                       waypoint = self.environment.robot.model.ikcon(targetPose * transl(0, 0, 0.1) * trotx(pi), q0);
                       self.MoveRobotArmJointAngles(waypoint);
                       targetPose = self.greenTargetLocations(:,:,i) * trotx(pi);

                   else
                       %waypoint = deg2rad([90, 60, 65, -35, 0]);
                       q0 = self.environment.robot.model.getpos;
                       waypoint = self.environment.robot.model.ikcon(targetPose * transl(0, 0, 0.1) * trotx(pi), q0);

                       self.MoveRobotArmJointAngles(waypoint);
                       targetPose = self.greenTargetLocations(:,:,i) * trotx(pi);

                   end

                   self.MoveRobotArm(targetPose);

                   disp('Target Pose: ');
                   disp(targetPose);
                   pose = self.environment.robot.model.fkine(self.environment.robot.model.getpos())
                   error = targetPose - pose

                   self.environment.robot.object = self.environment.targets{objectIndex};

                   % Return to deposit location
                   self.MoveRobotArmJointAngles(waypoint);
                   self.MoveRobotArmJointAngles(self.environment.robot.jointAngles);

                   targetPosition = self.greenDepositLocation(1,4,1);
                   self.MoveToTargetLinearRail(targetPosition);

                   % Drop object
                   targetPose = self.greenDepositLocation(:,:,1);
                   wayPoint = targetPose * transl(0, 0, 0.05);
                   self.MoveRobotArm(wayPoint * trotx(pi));
                   self.MoveRobotArm(targetPose * trotx(pi));

                   pose = self.environment.robot.model.fkine(self.environment.robot.model.getpos())

                   self.environment.robot.object = EnvironmentObject.empty;
                   self.MoveRobotArm(wayPoint * trotx(pi));
                   self.MoveRobotArmJointAngles(self.environment.robot.jointAngles);
               end
           end
           
           
           blueSize = size(self.blueTargetLocations);
           if blueSize(1) > 0
               for i = 1:1:size(self.blueTargetLocations, 3)
                   shortestDistance = 5;
                   objectIndex = 1;
                   targetPosition = self.blueTargetLocations(1,4,i);
                   self.MoveToTargetLinearRail(targetPosition);

                   % Collect Object
                   targetPose = self.blueTargetLocations(:,:,i);

                   for j = 1:size(self.environment.targets, 2)
                       distance = self.DistanceBetweenPoses(targetPose, self.environment.targets{j}.pose);
                       if distance < shortestDistance
                           objectIndex = j;
                           shortestDistance = distance;
                       end
                   end

                   if targetPose(2,4) < self.environment.robot.model.base(2,4)
                       %waypoint = deg2rad([-90, 60, 65, -35, 0]);
                       q0 = self.environment.robot.model.getpos;
                       waypoint = self.environment.robot.model.ikcon(targetPose * transl(0, 0, 0.1) * trotx(pi), q0);
                       self.MoveRobotArmJointAngles(waypoint);
                       targetPose = self.blueTargetLocations(:,:,i) * trotx(pi);

                   else
                       %waypoint = deg2rad([90, 60, 65, -35, 0]);
                       q0 = self.environment.robot.model.getpos;
                       waypoint = self.environment.robot.model.ikcon(targetPose * transl(0, 0, 0.1) * trotx(pi), q0);
                       self.MoveRobotArmJointAngles(waypoint);
                       targetPose = self.blueTargetLocations(:,:,i) * trotx(pi);

                   end

                   self.MoveRobotArm(targetPose);

                   disp('Target Pose: ');
                   disp(targetPose);
                   pose = self.environment.robot.model.fkine(self.environment.robot.model.getpos())
                   error = targetPose - pose

                   self.environment.robot.object = self.environment.targets{objectIndex};

                   % Return to deposit location
                   self.MoveRobotArmJointAngles(waypoint);
                   self.MoveRobotArmJointAngles(self.environment.robot.jointAngles);

                   targetPosition = self.blueDepositLocation(1,4,1);
                   self.MoveToTargetLinearRail(targetPosition);

                   % Drop object
                   targetPose = self.blueDepositLocation(:,:,1);
                   wayPoint = targetPose * transl(0, 0, 0.05);
                   self.MoveRobotArm(wayPoint * trotx(pi));
                   self.MoveRobotArm(targetPose * trotx(pi));

                   pose = self.environment.robot.model.fkine(self.environment.robot.model.getpos())

                   self.environment.robot.object = EnvironmentObject.empty;
                   self.MoveRobotArm(wayPoint * trotx(pi));
                   self.MoveRobotArmJointAngles(self.environment.robot.jointAngles);
               end
           end
                      
           
       end
       %% Initialise Simulation
       % Display the environment and calculate all the necessary target and
       % deposit locations and store them in appropriate variables
       function Init(self)
           self.camera.DisplayCamera();
           
           for i = 1:size(self.environment.targets,2)
               self.camera.AddObject(self.environment.targets{i});
           end
           
           for i = 1:size(self.environment.deposit,2)
               self.camera.AddObject(self.environment.deposit{i});
           end
           
       end
       %% Distance between poses
       function dist = DistanceBetweenPoses(self, pose1, pose2)
            dist = sqrt(((pose1(1,4)-pose2(1,4))^2 + (pose1(2,4)-pose2(2,4))^2 + (pose1(3,4)-pose2(3,4))^2));
       end
       %% Choose which targets are associated with each blob centroid
       function AssignCentroids(self)
           redIndex = 1;
           greenIndex = 1;
           blueIndex = 1;
           [numRows numCols] = size(self.camera.globalCentroids);
           for i = 1:numRows
               if self.camera.globalCentroidColours(i) == 'r'
                  if self.camera.globalCentroidAreas(i) >= 3000
                      self.redDepositLocation(:,:,1) = eye(4) * transl(self.camera.globalCentroids(i,1), self.camera.globalCentroids(i,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(i)));
                  else
                      self.redTargetLocations(:,:,redIndex) = eye(4) * transl(self.camera.globalCentroids(i,1), self.camera.globalCentroids(i,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(i)));
                      redIndex = redIndex + 1;
                  end
               elseif self.camera.globalCentroidColours(i) == 'g'
                  if self.camera.globalCentroidAreas(i) >= 3000
                      self.greenDepositLocation(:,:,1) = eye(4) * transl(self.camera.globalCentroids(i,1), self.camera.globalCentroids(i,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(i)));
                  else
                      self.greenTargetLocations(:,:,greenIndex) = eye(4) * transl(self.camera.globalCentroids(i,1), self.camera.globalCentroids(i,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(i)));
                      greenIndex = greenIndex + 1;
                  end
               else %blue
                  if self.camera.globalCentroidAreas(i) >= 3000
                      self.blueDepositLocation(:,:,1) = eye(4) * transl(self.camera.globalCentroids(i,1), self.camera.globalCentroids(i,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(i)));
                  else
                      self.blueTargetLocations(:,:,blueIndex) = eye(4) * transl(self.camera.globalCentroids(i,1), self.camera.globalCentroids(i,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(i)));
                      blueIndex = blueIndex + 1;
                  end 
               end
           end  
       end
       %% Move Arm
        function MoveRobotArm(self, targetPose)
            q0 = self.environment.robot.model.getpos;
            
            q1 = self.environment.robot.model.ikcon(targetPose, q0);
            
            %q1(4) = pi/2 - (q1(2) + q1(3));
            
            steps = 50;
            s = lspb(0,1,steps);
            qMatrix = nan(steps,5);

            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q0 + s(i)*q1;
            end
            
            if 0 < size(self.environment.obstacleObjects, 2)
                for i = 1:1:size(self.environment.obstacleObjects, 2)
                    checkCollision = self.environment.robot.CollisionCheck(qMatrix, self.environment.obstacleObjects{i}.modelMesh.Vertices);
                    
                    if checkCollision == 1
                        self.environment.robot.collisionDetected = 1;
                        break;
                    end
                end
            end
            while (self.environment.robot.collisionDetected == 1)
               self.environment.robot.collisionDetected = 0;   
               for j = 1:size(self.environment.obstacleObjects, 2)
                   checkCollision = self.environment.robot.CollisionCheck(qMatrix, self.environment.obstacleObjects{j}.modelMesh.Vertices);

                   if checkCollision == 1
                      self.environment.robot.collisionDetected = 1;
                      break;
                   end
               end
               pause(0.01);
            end
            
            for i = 1:steps
               self.environment.robot.stop = 0;
               if 0 < size(self.environment.checkObjects, 2)
                   for j = 1:size(self.environment.checkObjects, 2)
                       checkCurtain = self.environment.lightCurtain.CheckLightCurtain(self.environment.checkObjects{j});

                       if checkCurtain == 1
                          self.environment.robot.stop = 1;
                          break;
                       end
                   end
               end
               while (self.environment.robot.stop == 1 || self.emergencyStop == 1)
                   disp('Robot Stopped');
                   self.environment.robot.stop = 0;   
                   for j = 1:size(self.environment.checkObjects, 2)
                       checkCurtain = self.environment.lightCurtain.CheckLightCurtain(self.environment.checkObjects{j});

                       if checkCurtain == 1
                          self.environment.robot.stop = 1;
                          break;
                       end
                   end
                   pause(0.01);
               end
                
               self.environment.robot.model.animate(qMatrix(i, :));
               
               for k = 1:1:size(self.environment.robot.object, 2)
                    % Set the desired object pose to be at the end effector
                    objectPose = self.environment.robot.model.fkine(self.environment.robot.model.getpos());

                    % Update the object's position
                    %self.environment.robot.object(k).MoveObject(objectPose);
                    self.environment.robot.object(k)
                    self.environment.robot.MoveObject(self.environment.robot.object(k));
               end
               
               drawnow();
            end
        end
        
        %% Move Arm Joint Angles
        function MoveRobotArmJointAngles(self, targetAngles)
            q0 = self.environment.robot.model.getpos;
            
            q1 = targetAngles;
            
            %q1(4) = pi/2 - (q1(2) + q1(3));
            
            steps = 50;
            s = lspb(0,1,steps);
            qMatrix = nan(steps,5);

            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q0 + s(i)*q1;
            end
            
            if 0 < size(self.environment.obstacleObjects, 2)
                for i = 1:1:size(self.environment.obstacleObjects, 2)
                    checkCollision = self.environment.robot.CollisionCheck(qMatrix, self.environment.obstacleObjects{i}.modelMesh.Vertices);
                    
                    if checkCollision == 1
                        self.environment.robot.collisionDetected = 1;
                        break;
                    end
                end
            end
            
            while (self.environment.robot.collisionDetected == 1)
               self.environment.robot.collisionDetected = 0;   
               for j = 1:size(self.environment.obstacleObjects, 2)
                   checkCollision = self.environment.robot.CollisionCheck(qMatrix, self.environment.obstacleObjects{j}.modelMesh.Vertices);

                   if checkCollision == 1
                      self.environment.robot.collisionDetected = 1;
                      break;
                   end
               end
               pause(0.01);
            end
            
            

            for i = 1:steps
               self.environment.robot.stop = 0;
               if 0 < size(self.environment.checkObjects, 2)
                   for j = 1:size(self.environment.checkObjects, 2)
                       checkCurtain = self.environment.lightCurtain.CheckLightCurtain(self.environment.checkObjects{j});

                       if checkCurtain == 1
                          self.environment.robot.stop = 1;
                          break;
                       end
                   end
               end
               
               while (self.environment.robot.stop == 1 || self.emergencyStop == 1)
                   disp('Robot Stopped');
                   self.environment.robot.stop = 0;
                   for j = 1:size(self.environment.checkObjects, 2)
                       checkCurtain = self.environment.lightCurtain.CheckLightCurtain(self.environment.checkObjects{j});

                       if checkCurtain == 1
                          self.environment.robot.stop = 1;
                          break;
                       end
                   end
                   pause(0.01);
               end
                
               self.environment.robot.model.animate(qMatrix(i, :));
               
               for k = 1:1:size(self.environment.robot.object, 2)
                    % Set the desired object pose to be at the end effector
                    objectPose = self.environment.robot.model.fkine(self.environment.robot.model.getpos());

                    % Update the object's position
                    %self.environment.robot.object(k).MoveObject(objectPose);
                    self.environment.robot.object(k)
                    self.environment.robot.MoveObject(self.environment.robot.object(k));
               end
               
               drawnow();
            end
        end
        
        %% Move linear rail to target location
        function MoveToTargetLinearRail(self, x)
           disp("X is : " + x);
           steps = 50;
           distanceToTravel = x - self.environment.robot.model.base(1,4);
            
           if (((self.environment.robot.linearRailPose(1,4) - self.environment.robot.linearRail.modelMidPoint(1,1) + self.environment.robot.baseWidth/2) <= x) && (x + self.environment.robot.baseWidth/2 <= (self.environment.robot.linearRailPose(1,4) - self.environment.robot.linearRail.modelMidPoint(1,1) + self.environment.robot.linearRailTravelDist)))
              increment = distanceToTravel/steps;
              startingPosition = self.environment.robot.model.base(1,4);
              
              disp(1);
              
           elseif (x < (self.environment.robot.linearRailPose(1,4) - self.environment.robot.linearRail.modelMidPoint(1,1) + self.environment.robot.baseWidth/2))
              distance = (self.environment.robot.linearRailPose(1, 4) - self.environment.robot.linearRail.modelMidPoint(1,1) + self.environment.robotbaseWidth/2) - self.environment.robot.model.base(1,4);
              increment = distance/steps;
              
              startingPosition = self.environment.robot.model.base(1,4);
              
              disp(2);
              
           else      
              distance = (self.environment.robot.linearRailPose(1, 4) - self.environment.robot.linearRail.modelMidPoint(1,1) + self.environment.robot.linearRailTravelDist + self.environment.robot.baseWidth/2) - self.environment.robot.model.base(1,4);
              increment = distance/steps;
              
              startingPosition = self.environment.robot.model.base(1,4);

              disp(3);
           end

           
          if 0 < size(self.environment.obstacleObjects, 2)
              for i = 1:1:size(self.environment.obstacleObjects, 2)
                  for j = 1:steps
                    self.environment.robot.model.base(1, 4) = self.environment.robot.model.base(1,4) + increment;
                    checkCollision = self.environment.robot.CollisionCheck(self.environment.robot.model.getpos, self.environment.obstacleObjects{i}.modelMesh.Vertices);

                    if checkCollision == 1
                        self.environment.robot.collisionDetected = 1;
                        break;
                    end
                  end

                  self.environment.robot.model.base(1,4) = startingPosition;

                  if 0 < self.environment.robot.collisionDetected
                      break;
                  end
              end
          end

          while (self.environment.robot.collisionDetected == 1)
               self.environment.robot.collisionDetected = 0;   
               for i = 1:1:size(self.environment.obstacleObjects, 2)
                   for j = 1:steps
                    self.environment.robot.model.base(1, 4) = self.environment.robot.model.base(1,4) + increment;
                    checkCollision = self.environment.robot.CollisionCheck(self.environment.robot.model.getpos, self.environment.obstacleObjects{i}.modelMesh.Vertices);

                    if checkCollision == 1
                        self.environment.robot.collisionDetected = 1;
                        break;
                    end
                  end

                  self.environment.robot.model.base(1,4) = startingPosition;

                  if 0 < self.environment.robot.collisionDetected
                      break;
                  end
               end
               pause(0.01);
          end

          for i = 1:steps
               self.environment.robot.stop = 0;
               if 0 < size(self.environment.checkObjects, 2)
                   for j = 1:size(self.environment.checkObjects, 2)
                       checkCurtain = self.environment.lightCurtain.CheckLightCurtain(self.environment.checkObjects{j});

                       if checkCurtain == 1
                          self.environment.robot.stop = 1;
                          break;
                       end
                   end
               end

               while (self.environment.robot.stop == 1 || self.emergencyStop == 1)
                   disp('Robot Stopped');
                   self.environment.robot.stop = 0;
                   for j = 1:size(self.environment.checkObjects, 2)
                       checkCurtain = self.environment.lightCurtain.CheckLightCurtain(self.environment.checkObjects{j});

                       if checkCurtain == 1
                          self.environment.robot.stop = 1;
                          break;
                       end
                   end
                   pause(0.01);
               end


              self.environment.robot.model.base(1, 4) = self.environment.robot.model.base(1,4) + increment;
              self.environment.robot.model.animate(self.environment.robot.model.getpos);

              for k = 1:1:size(self.environment.robot.object, 2)
                % Set the desired object pose to be at the end effector
                objectPose = self.environment.robot.model.fkine(self.environment.robot.model.getpos());

                % Update the object's position
                %self.environment.robot.object(k).MoveObject(objectPose);
                self.environment.robot.object(k)
                self.environment.robot.MoveObject(self.environment.robot.object(k));
              end
              drawnow();
          end

        end
       
   end
end
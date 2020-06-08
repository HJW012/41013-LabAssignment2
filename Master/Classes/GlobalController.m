%% Global Controller Class
classdef GlobalController < handle
   properties
       environment;             % Environment class object used to store the simulation environment
       redDepositLocation;      % Location of the red deposit crate to store all red pens
       greenDepositLocation;    % Location of green deposit crate to store all pencils
       blueDepositLocation;     % Location of blue deposit crate to store all blue pens
       redTargetLocations;      % Locations of red pens in the environment
       greenTargetLocations;    % Locations of pencils in the environment
       blueTargetLocations;     % Locations of blue pens in the environment
       camera;                  % RGB Camera class object for overhead camera
       hand;                    % Hand object used to break the light curtain
       obstacle;                % Obstacle object used to force a collision
       emergencyStop = 0;       % Emergency stop signal received from E-Stop button in GUI
   end
   
   methods
       % Empty constructor to create the Global Controller Object
       function self = GlobalController() 

       end
       %% Simulation Setup
       % Function used to initialise the simulation by creating all
       % required objects and initialising the environment
       function Setup(self)
          % Create fixed objects in the scene which do not move during
          % operation
          table = EnvironmentObject('Type', 'foundation', 'ModelPath', 'table.ply', 'Pose', transl(0, 0, 0), 'Dimensions', [2.1956 1.0097 0.8911]);
          blueCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'blueCrate.ply', 'Pose', transl(0.7, 0.2, 0.8911), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'b');
          yellowCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'yellowCrate.ply', 'Pose', transl(0.7, 0, 0.8911), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'g');
          redCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'redCrate.ply', 'Pose', transl(0.7, -0.2, 0.8911), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'r');
          cone1 = EnvironmentObject('Type', 'safety', 'ModelPath', 'cone.ply', 'Pose', transl(1.25, -0.75, 0));
          cone2 = EnvironmentObject('Type', 'safety', 'ModelPath', 'cone.ply', 'Pose', transl(-1.25, -0.75, 0));
          cone3 = EnvironmentObject('Type', 'safety', 'ModelPath', 'cone.ply', 'Pose', transl(1.25, 0.75, 0));
          cone4 = EnvironmentObject('Type', 'safety', 'ModelPath', 'cone.ply', 'Pose', transl(-1.25, 0.75, 0));
          estop = EnvironmentObject('Type', 'safety', 'ModelPath', 'EStop.ply', 'Pose', transl(0, 0.8, 0));
       
          % Create the Environment object and pass it all the fixed objects
          self.environment = Environment(table, blueCrate, yellowCrate, redCrate, cone1, cone2, cone3, cone4, estop);
          
          % Create the robot
          self.environment.robot = Dobot('BasePose', transl(0, 0, 0));
          %Generate the robot's linear rail
          self.environment.robot.GenerateLinearRail([-0.45, 0, 0.8911]);

          % Create the overhead camera
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
       % Function called by the GUI to insert the pick up targets into the
       % environment (pens, pencils)
       function InsertTarget(self, pose, colour)
          % Red pen
          if strcmp(colour, 'r')
              % Create red pen object
             object = EnvironmentObject('Type', 'target', 'ModelPath', 'redPen.ply', 'Pose', pose, 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'r');
             disp('Creating Red Pen');
          end
          
          % Blue pen
          if strcmp(colour, 'b')
              % Create blue pen object
             object = EnvironmentObject('Type', 'target', 'ModelPath', 'bluePen.ply', 'Pose', pose, 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'b');
             disp('Creating Blue Pen');
          end
          
          % Pencil
          if strcmp(colour, 'g')
              % Create pencil object
             object = EnvironmentObject('Type', 'target', 'ModelPath', 'pencil.ply', 'Pose', pose, 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'g');
             disp('Creating Pencil');
          end
          
          % Display the object in the environment
          object.Display();
          % Add the object to the Environment class
          self.environment.AddObject(object);
       end
       %% Run Simulation
       % This is the main simulation function used to execute the pick-up
       % and deposit tasks
       function Run(self)
           % Initialise the starting position (to return to later)
           startingPose = self.environment.robot.model.base();
           
           disp('Calculating the Pick Up and Deposit Locations');
           
           % Calculate Pick Up and Deposit locations
           self.camera.LocateObjects();
                     
           % Determining if blobs are targets or deposit locations based on
           % blob area
           self.AssignCentroids();
           
           disp('Commencing Pick-Up Sequence');

           % Calculate number of red targets to pick up
           redSize = size(self.redTargetLocations);
           if redSize(1) > 0
               % Set deposit pose location
               depositPose = self.redDepositLocation(:,:,1);
               
               % Iterate through each red Target
               for i = 1:1:size(self.redTargetLocations, 3)
                   disp(['Collecting Red Target number: ', num2str(i)]);
                   % Set target pose
                   targetPose = self.redTargetLocations(:,:,i);

                   % Pick up and drop target item
                   self.CompleteMovementTasks(targetPose, depositPose);
               end
           end    
           
           % Calculate number of green targets
           greenSize = size(self.greenTargetLocations);
           if greenSize(1) > 0
               % Set deposit pose location
               depositPose = self.greenDepositLocation(:,:,1);
               
               % Iterate through each green target
               for i = 1:1:size(self.greenTargetLocations, 3)
                   disp(['Collecting Green Target number: ', num2str(i)]);
                   % Set target pose
                   targetPose = self.greenTargetLocations(:,:,i);
                    
                   % Pick up and drop target item
                   self.CompleteMovementTasks(targetPose, depositPose);
                   
               end
           end
           
           % Calculate number of blue targets
           blueSize = size(self.blueTargetLocations);
           if blueSize(1) > 0
               % Set deposit pose location
               depositPose = self.blueDepositLocation(:,:,1);
               
               % Iterate through each blue target
               for i = 1:1:size(self.blueTargetLocations, 3)
                   disp(['Collecting Blue Target number: ', num2str(i)]);
                   % Set target pose
                   targetPose = self.blueTargetLocations(:,:,i);

                   % Pick up and drop target item
                   self.CompleteMovementTasks(targetPose, depositPose);
                   
               end
           end
           
           % Return robot to starting position
           self.MoveToTargetLinearRail(startingPose(1,4))
           
       end
       
       %% Complete Movement Tasks
       % Function used to execute pick up and deposit movements
       function CompleteMovementTasks(self, targetPose, depositPose)
           % Initialise shortest distance variable to a high number
           % so that it will always be beaten
           shortestDistance = 5;
           objectIndex = 1;

           % Move along linear rail until in-line with target x
           % position
           self.MoveToTargetLinearRail(targetPose(1,4));

           disp('Target Pose: ');
           disp(targetPose);

           % Determine which of the environment target objects is
           % being collected by iterating through each target
           % object in the environment
           for j = 1:size(self.environment.targets, 2)
               % Calculate distance between target pose and target
               % object pose
               distance = self.DistanceBetweenPoses(targetPose, self.environment.targets{j}.pose);

               % Find the object which has the closest distance to
               % the target pose and save the object index so that
               % it can be correctly moved with the end effector
               % later
               if distance < shortestDistance
                   objectIndex = j;
                   shortestDistance = distance;
               end
           end

           % Get current joint angles
           q0 = self.environment.robot.model.getpos;

           % Calculate the joint angles to reach a waypoint being
           % 10cm above the target object
           wayPoint = targetPose * transl(0, 0, 0.1);

           % Move the robot arm to the waypoint above the target
           % object
           self.MoveRobotArm(wayPoint * trotx(pi));

           % Update target pose to be flipped around the x - axis
           % so that the z axes of the object and the robot align
           targetPose = targetPose * trotx(pi);

           % Move arm to pick up the object
           self.MoveRobotArm(targetPose);

           % Calculate final pose of the end effector
           finalPose = self.environment.robot.model.fkine(self.environment.robot.model.getpos());
           disp('Final End Effector Pose: ')
           disp(finalPose);

           % Calculate error between the target pose and the final
           % end effector pose
           error = targetPose - finalPose;
           disp('Error: ')
           disp(error);

           % Set object to be picked up and carried by the Dobot
           self.environment.robot.object = self.environment.targets{objectIndex};

           disp('Moving object to Deposit Location');

           % Move robot arm back to the waypoint position 10cm
           % above the object
           self.MoveRobotArm(wayPoint * trotx(pi));

           % Move robot arm back to default joint angles
           self.MoveRobotArmJointAngles(self.environment.robot.jointAngles);

           disp('Target Pose: ');
           disp(depositPose);

           % Move along linear rail until the end of the rail
           self.MoveToTargetLinearRail(depositPose(1,4));

           % Set waypoint to be 5cm above deposit location
           wayPoint = depositPose * transl(0, 0, 0.05);

           % Drive robot to waypoint
           self.MoveRobotArm(wayPoint * trotx(pi));

           % Drive robot to deposit location
           self.MoveRobotArm(depositPose * trotx(pi));

           % Calculate final pose of the end effector after
           % completing drop motion
           finalPose = self.environment.robot.model.fkine(self.environment.robot.model.getpos());

           disp('Final End Effector Pose: ')
           disp(finalPose);

           % Calculate error between the target pose and the final
           % end effector pose
           error = depositPose - finalPose;
           disp('Error: ')
           disp(error);

           % Unload object from the robot (drop the object in the
           % deposit location)
           self.environment.robot.object = EnvironmentObject.empty;

           % Move Robot Arm back to waypoint 5cm above deposit
           % location
           self.MoveRobotArm(wayPoint * trotx(pi));

           % Move Robot Arm back to default joint angle position
           self.MoveRobotArmJointAngles(self.environment.robot.jointAngles);  
       end
       
       %% Initialise Simulation
       % Initialisation function used to display the camera and add the
       % target and deposit objects to the camera object
       function Init(self)
           % Display camera in the environment
           self.camera.DisplayCamera();
           
           % Pass each of the target objects to the camera for processing
           for i = 1:size(self.environment.targets,2)
               self.camera.AddObject(self.environment.targets{i});
           end
           
           % Pass each of the deposit objects to the camera for processing
           for i = 1:size(self.environment.deposit,2)
               self.camera.AddObject(self.environment.deposit{i});
           end
           
       end
       %% Distance between poses
       % Simple distance function used to calculate the linear distance between
       % two, 3-dimensional poses.
       function dist = DistanceBetweenPoses(self, pose1, pose2)
            dist = sqrt(((pose1(1,4)-pose2(1,4))^2 + (pose1(2,4)-pose2(2,4))^2 + (pose1(3,4)-pose2(3,4))^2));
       end
       %% Choose which targets are associated with each blob centroid
       % Function used to determine which blob centroid the camera has generated
       % belongs to which target/deposit object
       function AssignCentroids(self)
           % Initialise indices
           redIndex = 1;
           greenIndex = 1;
           blueIndex = 1;
           
           % Calculate number of global centroids the camera has calculated
           [numRows numCols] = size(self.camera.globalCentroids);
           
           % Iterate through each global centroid
           for i = 1:numRows
               % If the blob is red
               if self.camera.globalCentroidColours(i) == 'r'
                  % If the area of the blob is large, then it must be the
                  % red deposit location
                  if self.camera.globalCentroidAreas(i) >= 3000
                      % Store the red deposit location pose
                      self.redDepositLocation(:,:,1) = eye(4) * transl(self.camera.globalCentroids(i,1), self.camera.globalCentroids(i,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(i)));
                  else
                      % Otherwise, store the location as a red pen target
                      self.redTargetLocations(:,:,redIndex) = eye(4) * transl(self.camera.globalCentroids(i,1), self.camera.globalCentroids(i,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(i)));
                      % Increment the index
                      redIndex = redIndex + 1;
                  end
               % If the blob is green
               elseif self.camera.globalCentroidColours(i) == 'g'
                  % If the area of the blob is large, then it must be the
                  % green deposit location
                  if self.camera.globalCentroidAreas(i) >= 3000
                      % Store the green deposit location pose
                      self.greenDepositLocation(:,:,1) = eye(4) * transl(self.camera.globalCentroids(i,1), self.camera.globalCentroids(i,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(i)));
                  else
                      % Otherwise store the location as a pencil target
                      self.greenTargetLocations(:,:,greenIndex) = eye(4) * transl(self.camera.globalCentroids(i,1), self.camera.globalCentroids(i,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(i)));
                      % Increment the index
                      greenIndex = greenIndex + 1;
                  end
               % If the blob is blue
               else
                  % If the area of the blob is large, then it must be the
                  % blue deposit location
                  if self.camera.globalCentroidAreas(i) >= 3000
                      % Store the blue deposit location pose
                      self.blueDepositLocation(:,:,1) = eye(4) * transl(self.camera.globalCentroids(i,1), self.camera.globalCentroids(i,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(i)));
                  else
                      % Otherwise store the location as blue pen target
                      self.blueTargetLocations(:,:,blueIndex) = eye(4) * transl(self.camera.globalCentroids(i,1), self.camera.globalCentroids(i,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(i)));
                      % Increment the index
                      blueIndex = blueIndex + 1;
                  end 
               end
           end  
       end
       %% Move Arm
       % Function used to move the robotic arm to a required target pose
       function MoveRobotArm(self, targetPose)
            % Calculate the current joint angles
            q0 = self.environment.robot.model.getpos;
            
            % Calculate the target joint angles to reach the target pose
            % using the current joint angles as an initial estimate
            q1 = self.environment.robot.model.ikcon(targetPose, q0);
            
            % Update joint 4 to ensure end effector remains perpendicular to the
            % surface of the table
            q1(4) = pi/2 - (q1(2) + q1(3));
            
            % Initialise steps
            steps = 50;
            
            % Calculate trajectory using trapezoidal velocity profile
            s = lspb(0,1,steps);
            qMatrix = nan(steps,5);
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q0 + s(i)*q1;
            end
            
            % If there is an obstacle object check for collisions
            if 0 < size(self.environment.obstacleObjects, 2)
                % Iterate through each obstacle object
                for i = 1:1:size(self.environment.obstacleObjects, 2)
                    % Check if the robot will collide with the obstacle at
                    % any point in the calculated trajectory
                    checkCollision = self.environment.robot.CollisionCheck(qMatrix, self.environment.obstacleObjects{i}.modelMesh.Vertices);
                    
                    % If collision is detected, set the flag to 1 and break
                    % out of the for loop
                    if checkCollision == 1
                        self.environment.robot.collisionDetected = 1;
                        disp('Collision detected in trajectory, human intervention required to remove obstruction');
                        break;
                    end
                end
            end
            % Loop while the collision detected flag is set and keep
            % checking until the obstacle is removed from the planned
            % trajectory
            while (self.environment.robot.collisionDetected == 1)
               % Reset flag 
               self.environment.robot.collisionDetected = 0;
               % Iterate through each obstacle object
               for j = 1:size(self.environment.obstacleObjects, 2)
                   % Check if the robot will collide with the obstacle at
                   % any point in the calculated trajectory
                   checkCollision = self.environment.robot.CollisionCheck(qMatrix, self.environment.obstacleObjects{j}.modelMesh.Vertices);

                   % If collision is detected set flag to 1 and break out
                   % of the for loop
                   if checkCollision == 1
                      self.environment.robot.collisionDetected = 1;
                      break;
                   end
               end
               pause(0.01);
            end
                       
            % Iterate through each step in the trajectory
            for i = 1:steps
               % Initialise flag to be checked later
               eStopCount = 0;
               % Set robot stop flag to zero
               self.environment.robot.stop = 0;
               
               % If there are objects to be checked in the environment
               if 0 < size(self.environment.checkObjects, 2)
                   % Iterate through each object to be checked
                   for j = 1:size(self.environment.checkObjects, 2)
                       % Check light curtain to see if it has been broken
                       checkCurtain = self.environment.lightCurtain.CheckLightCurtain(self.environment.checkObjects{j});

                       % If light curtain has been broken set flag to 1 and
                       % break out of for loop
                       if checkCurtain == 1
                          self.environment.robot.stop = 1;
                          break;
                       end
                   end
               end
               % While either the light curtain has been broken or the
               % emergency stop button has been pressed on the GUI, check
               % the light curtain until obstruction is removed
               while (self.environment.robot.stop == 1 || self.emergencyStop == 1)
                   % Simple check so that display does not flood output
                   if eStopCount < 1
                       disp('Robot Stopped');
                       eStopCount = 1;
                   end
                   
                   % Reset light curtain flag
                   self.environment.robot.stop = 0;   
                   % Iterate through each check object
                   for j = 1:size(self.environment.checkObjects, 2)
                       % Check light curtain to see if it has been broken
                       % by an obstruction
                       checkCurtain = self.environment.lightCurtain.CheckLightCurtain(self.environment.checkObjects{j});

                       % If light curtain has been broken, set flag to 1
                       % and break out of for loop
                       if checkCurtain == 1
                          self.environment.robot.stop = 1;
                          break;
                       end
                   end
                   pause(0.01);
               end
               
               % Animate robot movement to next joint state in the
               % trajectory
               self.environment.robot.model.animate(qMatrix(i, :));
               
               % If the robot is carrying an object, move it with the end
               % effector
               for k = 1:1:size(self.environment.robot.object, 2)
                    % Update the object's position
                    self.environment.robot.object(k);
                    self.environment.robot.MoveObject(self.environment.robot.object(k));
               end
               
               drawnow();
            end
        end
        
        %% Move Arm Joint Angles
        % Function used to move the robotic arm to a given joint angle configuration
        function MoveRobotArmJointAngles(self, targetAngles)
            % Calculate current joint angle configuration
            q0 = self.environment.robot.model.getpos;
            
            % Store target angles in a variable
            q1 = targetAngles;
            
            % Ensure the fourth joint angle makes the end effector
            % perpendicular to the surface of the table
            q1(4) = pi/2 - (q1(2) + q1(3));
            
            % Initialise number of steps in trajectory
            steps = 50;
            
            % Calculate trajectory using trapezoidal velocity profile
            s = lspb(0,1,steps);
            qMatrix = nan(steps,5);
            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q0 + s(i)*q1;
            end
            
            % If there is an obstacle object check for collisions
            if 0 < size(self.environment.obstacleObjects, 2)
                % Iterate through each obstacle object
                for i = 1:1:size(self.environment.obstacleObjects, 2)
                    % Check for collisions with the object along the entire
                    % calculated trajectory
                    checkCollision = self.environment.robot.CollisionCheck(qMatrix, self.environment.obstacleObjects{i}.modelMesh.Vertices);
                    
                    % If a collision is detected, set the flag to 1 and
                    % break out of the for loop
                    if checkCollision == 1
                        self.environment.robot.collisionDetected = 1;
                        disp('Collision detected in trajectory, human intervention required to remove obstruction');
                        break;
                    end
                end
            end
            
            % Loop while a collision has been detected and keep checking
            % until no collisions with any obstacle are detected
            while (self.environment.robot.collisionDetected == 1)
               % Reset collision detection flag
               self.environment.robot.collisionDetected = 0;
               % Iterate through each obstacle object
               for j = 1:size(self.environment.obstacleObjects, 2)
                   % Check for collisions with the object at any point on
                   % the calculated trajectory
                   checkCollision = self.environment.robot.CollisionCheck(qMatrix, self.environment.obstacleObjects{j}.modelMesh.Vertices);

                   % If a collision is detected set the flag to 1 and break
                   % out of the for loop
                   if checkCollision == 1
                      self.environment.robot.collisionDetected = 1;
                      break;
                   end
               end
               pause(0.01);
            end
            
            
            % Iterate through each step in the trajectory
            for i = 1:steps
               % Initialise flag to be checked later
               eStopCount = 0;
               
               % Reset the robot stop flag 
               self.environment.robot.stop = 0;
               
               % If there are objects in the environment which could
               % collide with the light curtain check those objects for
               % obstruction
               if 0 < size(self.environment.checkObjects, 2)
                   % Iterate through each object
                   for j = 1:size(self.environment.checkObjects, 2)
                       % Check if the light curtain is broken
                       checkCurtain = self.environment.lightCurtain.CheckLightCurtain(self.environment.checkObjects{j});

                       % If the light curtain is broken by an object, set
                       % the flag to 1 and break out of the for loop
                       if checkCurtain == 1
                          self.environment.robot.stop = 1;
                          break;
                       end
                   end
               end
               
               % Loop while the light curtain flag or the emergency stop
               % flag from the GUI are set. Continue to check the light
               % curtain until the obstruction has been cleared
               while (self.environment.robot.stop == 1 || self.emergencyStop == 1)
                   % Simple check to prevent the command prompt from being
                   % flooded with messages
                   if eStopCount < 1
                       disp('Robot Stopped');
                       eStopCount = 1;
                   end
                                      
                   % Reset the robot stop flag
                   self.environment.robot.stop = 0;
                   % Iterate through each check object in the environment
                   for j = 1:size(self.environment.checkObjects, 2)
                       % Check the light curtain to see if any of the
                       % objects obstruct the light curtain
                       checkCurtain = self.environment.lightCurtain.CheckLightCurtain(self.environment.checkObjects{j});

                       % If the light curtain has been broken, set the flag
                       % to 1 and break out of the for loop
                       if checkCurtain == 1
                          self.environment.robot.stop = 1;
                          break;
                       end
                   end
                   pause(0.01);
               end
                
               % Move the robot to the next joint angle in the trajectory
               self.environment.robot.model.animate(qMatrix(i, :));
               
               % If the robot is carrying an object, update the object's
               % position to move with the end effector
               for k = 1:1:size(self.environment.robot.object, 2)
                    % Update the object's position
                    self.environment.robot.object(k);
                    self.environment.robot.MoveObject(self.environment.robot.object(k));
               end
               
               drawnow();
            end
        end
        
        %% Move linear rail to target location
        % Function to move the robot along the linear rail.
        function MoveToTargetLinearRail(self, x)          
           % Initialise the number of steps
           steps = 50;
           
           % Calculate the distance to travel between the current base
           % location and the target position
           distanceToTravel = x - self.environment.robot.model.base(1,4);
           
           % Store the starting position of the robot
           startingPosition = self.environment.robot.model.base(1,4);
           
           % If the target position lies within the bounds of the linear
           % rail
           if (((self.environment.robot.linearRailPose(1,4) - self.environment.robot.linearRail.modelMidPoint(1,1) + self.environment.robot.baseWidth/2) <= x) && (x + self.environment.robot.baseWidth/2 <= (self.environment.robot.linearRailPose(1,4) - self.environment.robot.linearRail.modelMidPoint(1,1) + self.environment.robot.linearRailTravelDist)))
              % Calculate the increment to step each time
              increment = distanceToTravel/steps;

           % If the target position is less than the bottom limit of the
           % linear rail
           elseif (x < (self.environment.robot.linearRailPose(1,4) - self.environment.robot.linearRail.modelMidPoint(1,1) + self.environment.robot.baseWidth/2))
              % Update the distance to travel to be between the current
              % robot position and the bottom limit of the linear rail
              distance = (self.environment.robot.linearRailPose(1, 4) - self.environment.robot.linearRail.modelMidPoint(1,1) + self.environment.robotbaseWidth/2) - self.environment.robot.model.base(1,4);
              
              % Calculate the increment to step each time
              increment = distance/steps;
              
           % If the target position is above the upper limit of the linear
           % rail
           else     
              % Update the distance to travel to be between the current
              % robot position and the upper limit of the linear rail
              distance = (self.environment.robot.linearRailPose(1, 4) - self.environment.robot.linearRail.modelMidPoint(1,1) + self.environment.robot.linearRailTravelDist + self.environment.robot.baseWidth/2) - self.environment.robot.model.base(1,4);
              
              % Calculate the increment to step each time
              increment = distance/steps;
              
           end

          % If there is an obstacle object check for collisions
          if 0 < size(self.environment.obstacleObjects, 2)
              % Iterate through each obstacle object
              for i = 1:1:size(self.environment.obstacleObjects, 2)
                  % Iterate through each step of the linear trajectory
                  for j = 1:steps
                    % Update the base location
                    self.environment.robot.model.base(1, 4) = self.environment.robot.model.base(1,4) + increment;
                    
                    % Check if there are any collisions between the
                    % obstacles and the dobot at that point of the linear
                    % trajectory
                    checkCollision = self.environment.robot.CollisionCheck(self.environment.robot.model.getpos, self.environment.obstacleObjects{i}.modelMesh.Vertices);

                    % If there is a collision, set the flag to 1 and break
                    % out of the for loop
                    if checkCollision == 1
                        self.environment.robot.collisionDetected = 1;
                        disp('Collision detected in trajectory, human intervention required to remove obstruction');
                        break;
                    end
                  end

                  % Reset the base location to be the starting position of
                  % the robot to prevent any unexpected movements
                  self.environment.robot.model.base(1,4) = startingPosition;

                  % If a collision was detected, break out of the for loop
                  if 0 < self.environment.robot.collisionDetected
                      break;
                  end
              end
          end

          % Loop while a collision is still detected along the length of
          % the linear trajectory. Continue to check for collisions until
          % the obstacle is removed
          while (self.environment.robot.collisionDetected == 1)
               % Reset the collision detection flag
               self.environment.robot.collisionDetected = 0;   
               
               % Iterate through each obstacle object
               for i = 1:1:size(self.environment.obstacleObjects, 2)
                   % Iterate through each step of the linear trajectory
                   for j = 1:steps
                    % Update the base location of the robot to the next
                    % point in the trajectory
                    self.environment.robot.model.base(1, 4) = self.environment.robot.model.base(1,4) + increment;
                    
                    % Check if there are any collisions between the Dobot
                    % and the obstacles at this point of the trajectory
                    checkCollision = self.environment.robot.CollisionCheck(self.environment.robot.model.getpos, self.environment.obstacleObjects{i}.modelMesh.Vertices);

                    % If there is a collision, set the flag to 1 and break
                    % out of the for loop
                    if checkCollision == 1
                        self.environment.robot.collisionDetected = 1;
                        break;
                    end
                   end

                  % Set the robot base position to be the starting position
                  % to prevent unexpected movement
                  self.environment.robot.model.base(1,4) = startingPosition;

                  % If a collision has been detected, break out of the for
                  % loop
                  if 0 < self.environment.robot.collisionDetected
                      break;
                  end
               end
               pause(0.01);
          end

          % Iterate through each step of the calculated linear trajectory
          for i = 1:steps
               % Initialise flag to be checked later
               eStopCount = 0;
               
               % Reset the flag to zero
               self.environment.robot.stop = 0;
               
               % If there are objects that need to be checked for light
               % curtain obstructions check light curtain
               if 0 < size(self.environment.checkObjects, 2)
                   % Iterate through each check object
                   for j = 1:size(self.environment.checkObjects, 2)
                       % Check if the light curtain has been broken by the
                       % object
                       checkCurtain = self.environment.lightCurtain.CheckLightCurtain(self.environment.checkObjects{j});

                       % If light curtain has been broken, set the flag to
                       % 1 and break out of the for loop
                       if checkCurtain == 1
                          self.environment.robot.stop = 1;
                          break;
                       end
                   end
               end
               
               % Loop while the light curtain flag or emergency stop button
               % on the GUI have been pressed. Continue to check light
               % curtain until the obstruction has been removed
               while (self.environment.robot.stop == 1 || self.emergencyStop == 1)
                   % Simple check to prevent the command prompt from being
                   % flooded
                   if eStopCount < 1
                       disp('Robot Stopped');
                       eStopCount = 1;
                   end
                   
                   % Reset the flag
                   self.environment.robot.stop = 0;
                   
                   % Iterate through each check object
                   for j = 1:size(self.environment.checkObjects, 2)
                       
                       % Check if the light curtain has been obstructed by
                       % the object
                       checkCurtain = self.environment.lightCurtain.CheckLightCurtain(self.environment.checkObjects{j});

                       % If the light curtain has been broken, set the flag
                       % to 1 and break out of the for loop
                       if checkCurtain == 1
                          self.environment.robot.stop = 1;
                          break;
                       end
                   end
                   pause(0.01);
               end

              % Update base location of the robot
              self.environment.robot.model.base(1, 4) = self.environment.robot.model.base(1,4) + increment;
              % Animate the robot to visualise the updated base location
              self.environment.robot.model.animate(self.environment.robot.model.getpos);

              % If the robot is carrying an object, update its location to
              % match the end effector position
              for k = 1:1:size(self.environment.robot.object, 2)
                % Update the object's position
                self.environment.robot.object(k);
                self.environment.robot.MoveObject(self.environment.robot.object(k));
              end
              drawnow();
          end

        end
       
   end
end
%% Object class
classdef GlobalController < handle
   properties
       environment;
       depositLocations;
       redTargetLocations;
       greenTargetLocations;
       blueTargetLocations;
       camera;
       robot;
   end
   
   methods
       function self = GlobalController(environment, camera)  
           self.environment = environment;
           self.camera = camera;
       end
       %% Add Environment
       function AddEnvironment(self, environment)
           self.environment = environment;
       end
%        %% Add Robot
%        function AddRobot(self, robot)
%           self.robot(numel(self.robot)+1) = robot; 
%        end
       %% Run Simulation
       function Run(self)
           disp('Calculating the Pick Up and Deposit Locations');
           
           % Calculate Pick Up and Deposit locations
           self.camera.LocateObjects();
           
           for i = 1:size(self.environment.deposit,2)
               self.depositLocations(i, :) = self.camera.globalCentroids((end - (i-1)), :);
           end
           
           
           
           
           % NEED TO FIND WAY OF DOING THIS USING THE COLOURS OF EACH
           % TARGET OBJECT
           self.redTargetLocations(:,:,1) = eye(4) * transl(self.camera.globalCentroids(1,1), self.camera.globalCentroids(1,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(1)));
           self.greenTargetLocations(:,:,1) = eye(4) * transl(self.camera.globalCentroids(2,1), self.camera.globalCentroids(2,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(2)));
           self.greenTargetLocations(:,:,2) = eye(4) * transl(self.camera.globalCentroids(4,1), self.camera.globalCentroids(4,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(4)));
           self.blueTargetLocations(:,:,1) = eye(4) * transl(self.camera.globalCentroids(3,1), self.camera.globalCentroids(3,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(3)));
           
           % NEED TO FIX THE CODE ABOVE SO THAT IT IS NOT HARD CODED
           
 
           
           
           % Drive to pick up location on linear slide
           for i = 1:1:size(self.redTargetLocations, 3)
               targetPosition = self.redTargetLocations(1,4,i);
               self.environment.robot.MoveToTargetLinearRail(targetPosition);
               
               % Collect Object
               targetPose = self.redTargetLocations(:,:,i);
               self.environment.robot.MoveArm(targetPose * trotz(pi/2));
               
               pose = self.environment.robot.model.fkine(self.environment.robot.model.getpos())
               
               % Return to deposit location
               self.environment.robot.MoveArm(self.environment.robot.model.fkine(self.environment.robot.jointAngles));
               targetPosition = self.depositLocations(1,1);
               self.environment.robot.MoveToTargetLinearRail(targetPosition);

               % Drop object
               
               
           end
           
           for i = 1:1:size(self.greenTargetLocations, 3)
               targetPosition = self.greenTargetLocations(1,4,i);
               self.environment.robot.MoveToTargetLinearRail(targetPosition);
               
               % Collect Object
               targetPose = self.greenTargetLocations(:,:,i);
               self.environment.robot.MoveArm(targetPose * trotz(pi/2));
               
               pose = self.environment.robot.model.fkine(self.environment.robot.model.getpos())
               
               % Return to deposit location
               self.environment.robot.MoveArm(self.environment.robot.model.fkine(self.environment.robot.jointAngles));
               targetPosition = self.depositLocations(1,1);
               self.environment.robot.MoveToTargetLinearRail(targetPosition);

               % Drop object
           end
           
           for i = 1:1:size(self.blueTargetLocations, 3)
               targetPosition = self.blueTargetLocations(1,4,i);
               self.environment.robot.MoveToTargetLinearRail(targetPosition);
               
               % Collect Object
               targetPose = self.blueTargetLocations(:,:,i);
               self.environment.robot.MoveArm(targetPose * trotz(pi/2));
               
               pose = self.environment.robot.model.fkine(self.environment.robot.model.getpos())
               
               % Return to deposit location
               self.environment.robot.MoveArm(self.environment.robot.model.fkine(self.environment.robot.jointAngles));
               targetPosition = self.depositLocations(1,1);
               self.environment.robot.MoveToTargetLinearRail(targetPosition);

               % Drop object
           end
                      
           
       end
       %% Initialise Simulation
       % Display the environment and calculate all the necessary target and
       % deposit locations and store them in appropriate variables
       function Init(self)
           self.environment.Display();
           self.camera.DisplayCamera();
           
           for i = 1:size(self.environment.targets,2)
               self.camera.AddObject(self.environment.targets{i});
           end
           
           for i = 1:size(self.environment.deposit,2)
               self.camera.AddObject(self.environment.deposit{i});
           end
           
       end
       %% Init Environment
       function InitEnvironment(self)
           
       end
   end
end
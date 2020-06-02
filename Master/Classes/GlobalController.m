%% Object class
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
           startingPose = self.environment.robot.model.base();
           
           disp('Calculating the Pick Up and Deposit Locations');
           
           % Calculate Pick Up and Deposit locations
           self.camera.LocateObjects();
           
           
           
           
                     
           % NEED TO FIND WAY OF DOING THIS USING THE COLOURS OF EACH
           % TARGET OBJECT
           self.redDepositLocation(:,:,1) = eye(4) * transl(self.camera.globalCentroids(1,1), self.camera.globalCentroids(1,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(1)));
           self.redTargetLocations(:,:,1) = eye(4) * transl(self.camera.globalCentroids(4,1), self.camera.globalCentroids(4,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(4)));
           self.greenDepositLocation(:,:,1) = eye(4) * transl(self.camera.globalCentroids(2,1), self.camera.globalCentroids(2,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(2)));
           self.greenTargetLocations(:,:,1) = eye(4) * transl(self.camera.globalCentroids(5,1), self.camera.globalCentroids(5,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(5)));
           self.greenTargetLocations(:,:,2) = eye(4) * transl(self.camera.globalCentroids(7,1), self.camera.globalCentroids(7,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(7)));
           self.blueDepositLocation(:,:,1) = eye(4) * transl(self.camera.globalCentroids(3,1), self.camera.globalCentroids(3,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(3)));
           self.blueTargetLocations(:,:,1) = eye(4) * transl(self.camera.globalCentroids(6,1), self.camera.globalCentroids(6,2), 0.8911) * trotz(deg2rad(self.camera.globalOrientations(6)));
           
           % NEED TO FIX THE CODE ABOVE SO THAT IT IS NOT HARD CODED
           
 
           
           
           
           % Drive to pick up location on linear slide
           for i = 1:1:size(self.redTargetLocations, 3)
               targetPosition = self.redTargetLocations(1,4,i);
               self.environment.robot.MoveToTargetLinearRail(targetPosition);
               
               % Collect Object
               targetPose = self.redTargetLocations(:,:,i);
               
               if targetPose(2,4) < self.environment.robot.model.base(2,4)
                   self.environment.robot.MoveArmJointAngles(deg2rad([-90, 60, 65, -35, 0]));
               else
                   self.environment.robot.MoveArmJointAngles(deg2rad([90, 60, 65, -35, 0]));
               end

               self.environment.robot.MoveArm(targetPose * trotz(deg2rad(80)) * trotx(pi));
               
                             
               disp('Target Pose: ');
               disp(targetPose * trotz(deg2rad(80)) * trotx(pi));
               pose = self.environment.robot.model.fkine(self.environment.robot.model.getpos())
               error = targetPose - pose
               
               % Return to deposit location
               self.environment.robot.MoveArm(self.environment.robot.model.fkine(self.environment.robot.jointAngles));
               targetPosition = startingPose(1,4);
               self.environment.robot.MoveToTargetLinearRail(targetPosition);

               % Drop object
               targetPose = self.redDepositLocation(:,:,1);
               self.environment.robot.MoveArm(targetPose * trotz(deg2rad(80)));
               
               pose = self.environment.robot.model.fkine(self.environment.robot.model.getpos())
               
               self.environment.robot.MoveArm(self.environment.robot.model.fkine(self.environment.robot.jointAngles));

           end
           
           for i = 1:1:size(self.greenTargetLocations, 3)
               targetPosition = self.greenTargetLocations(1,4,i);
               self.environment.robot.MoveToTargetLinearRail(targetPosition);
               
               % Collect Object
               targetPose = self.greenTargetLocations(:,:,i);
               
               if targetPose(2,4) < self.environment.robot.model.base(2,4)
                   self.environment.robot.MoveArmJointAngles(deg2rad([-90, 60, 65, -35, 0]));
               else
                   self.environment.robot.MoveArmJointAngles(deg2rad([90, 60, 65, -35, 0]));
               end
               
               self.environment.robot.MoveArm(targetPose * trotz(deg2rad(80)) * trotx(pi));
               
               disp('Target Pose: ');
               disp(targetPose * trotz(deg2rad(80)) * trotx(pi));
               pose = self.environment.robot.model.fkine(self.environment.robot.model.getpos())
               error = targetPose - pose
               
               % Return to deposit location
               self.environment.robot.MoveArm(self.environment.robot.model.fkine(self.environment.robot.jointAngles));
               targetPosition = startingPose(1,4);
               self.environment.robot.MoveToTargetLinearRail(targetPosition);

               % Drop object
               targetPose = self.greenDepositLocation(:,:,1);
               self.environment.robot.MoveArm(targetPose * trotz(deg2rad(80)));
               
               pose = self.environment.robot.model.fkine(self.environment.robot.model.getpos())
               
               self.environment.robot.MoveArm(self.environment.robot.model.fkine(self.environment.robot.jointAngles));

           end
           
           for i = 1:1:size(self.blueTargetLocations, 3)
               targetPosition = self.blueTargetLocations(1,4,i);
               self.environment.robot.MoveToTargetLinearRail(targetPosition);
               
               % Collect Object
               targetPose = self.blueTargetLocations(:,:,i);
               
               if targetPose(2,4) < self.environment.robot.model.base(2,4)
                   self.environment.robot.MoveArmJointAngles(deg2rad([-90, 60, 65, -35, 0]));
               else
                   self.environment.robot.MoveArmJointAngles(deg2rad([90, 60, 65, -35, 0]));
               end
               
               self.environment.robot.MoveArm(targetPose * trotz(deg2rad(80)) * trotx(pi));
               
               disp('Target Pose: ');
               disp(targetPose * trotz(deg2rad(80)) * trotx(pi));
               pose = self.environment.robot.model.fkine(self.environment.robot.model.getpos())
               error = targetPose - pose
               
               % Return to deposit location
               self.environment.robot.MoveArm(self.environment.robot.model.fkine(self.environment.robot.jointAngles));
               targetPosition = startingPose(1,4);
               self.environment.robot.MoveToTargetLinearRail(targetPosition);

               % Drop object
               targetPose = self.blueDepositLocation(:,:,1);
               self.environment.robot.MoveArm(targetPose * trotz(deg2rad(80)));
               
               pose = self.environment.robot.model.fkine(self.environment.robot.model.getpos())
               
               self.environment.robot.MoveArm(self.environment.robot.model.fkine(self.environment.robot.jointAngles));

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
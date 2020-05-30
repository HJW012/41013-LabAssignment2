%% Object class
classdef GlobalController < handle
   properties
       environment;
       depositLocations;
       redTargetLocations;
       greenTargetLocations;
       blueTargetLocations;
       robot;
   end
   
   methods
       function self = GlobalController()  
           
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
           % Calculate deposit locations
           
           % Calculate Pick Up locations
           
           % Drive to pick up location
           
           % Collect object
           
           % Drive to deposit location
           
           % Drop object
           
           % Loop until all objects have been collected
       end
       %% Initialise Simulation
       function Init(self)
           self.InitEnvironment();
           
       end
       %% Init Environment
       function InitEnvironment(self)
           self.AddEnvironment(Environment());
           
           blueCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'blueCrate.ply', 'Pose', transl(-0.75, 0.35, 0.8911), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'b');
           self.environment.AddObject(blueCrate);
           
           yellowCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'yellowCrate.ply', 'Pose', transl(-0.75, 0, 0.8911), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'y');
           self.environment.AddObject(yellowCrate);
           
           redCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'redCrate.ply', 'Pose', transl(-0.75, -0.35, 0.8911), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'r');
           self.environment.AddObject(redCrate);
           
           redPen = EnvironmentObject('Type', 'target', 'ModelPath', 'redPen.ply', 'Pose', transl(0.75, 0.35, 0.8911), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'r');
           self.environment.AddObject(redPen);
           
           bluePen = EnvironmentObject('Type', 'target', 'ModelPath', 'bluePen.ply', 'Pose', transl(0.15, -0.1, 0.8911), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'b');
           self.environment.AddObject(bluePen);
           
           pencil1 = EnvironmentObject('Type', 'target', 'ModelPath', 'pencil.ply', 'Pose', transl(0.3, 0.2, 0.8911), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'y');
           self.environment.AddObject(pencil1);
           
           pencil2 = EnvironmentObject('Type', 'target', 'ModelPath', 'pencil.ply', 'Pose', transl(0, -0.3, 0.8911), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'y');
           self.environment.AddObject(pencil2);
           
           table = EnvironmentObject('Type', 'foundation', 'ModelPath', 'table.ply', 'Pose', transl(0, 0, 0), 'Dimensions', [2.1956 1.0097 0.8911], 'GeneralColour', 'r');
           self.environment.AddObject(table);
           
           newRobot = Dobot('BasePose', eye(4)*transl(0,0,table.dimensions(1,3)));
           self.environment.AddRobot(newRobot);
           self.environment.robot.GenerateLinearRail([0,0,0.8911]);
           
           self.environment.Display();

       end
   end
end
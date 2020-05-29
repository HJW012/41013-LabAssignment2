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
       %% Add Robot
       function AddRobot(self, robot)
          self.robot(numel(self.robot)+1) = robot; 
       end
       %% Run Simulation
       function Run(self)
           
       end
       %% Initialise Simulation
       function Init(self)
           % Add Robot
           self.environment.AddRobot(self.robot(1));
           
           % Add Object
           self.environment.AddObject
           
       end
       %% Init Environment
       function InitEnvironment(self)
           
       end
   end
end
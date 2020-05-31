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
       function self = GlobalController(environment)  
           self.environment = environment;
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
       % Display the environment and calculate all the necessary target and
       % deposit locations and store them in appropriate variables
       function Init(self)
           self.environment.Display();
           
       end
       %% Init Environment
       function InitEnvironment(self)
           
       end
   end
end
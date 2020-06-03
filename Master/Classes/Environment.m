%% Environment Class
classdef Environment < handle
   properties
       foundation = EnvironmentObject.empty;
       targets;
       deposit;
       miscObjects;
       targetIndex = 1;
       depositIndex = 1;
       miscObjectIndex = 1;
       lightCurtain;
       robot;
       checkObjects;
       checkObjectIndex = 1;
       obstacleObjects;
       obstacleObjectIndex = 1;
   end
   
   methods 
       function self = Environment(varargin)
           for i = 1:1:nargin
               object = varargin{i};
               self.AddObject(object);
           end
           
           if (self.lightCurtain ~= EnvironmentObject.empty)
               self.lightCurtain = LightCurtain(self.foundation);
           else
               disp("Environment: No table present. Expecting table through AddObject");
           end
           
       end
       %% Add generic object
       function AddObject(self, object)
           if strcmp(object.type, 'foundation') 
               self.foundation = object;
               self.lightCurtain = LightCurtain(self.foundation);
           end
           
           if strcmp(object.type, 'target')
               self.targets{1, self.targetIndex} = object;
               self.targetIndex = self.targetIndex + 1;
           end
           
           if strcmp(object.type, 'deposit')
               self.deposit{1, self.depositIndex} = object;
               self.depositIndex = self.depositIndex + 1;
           end
           
           if strcmp(object.type, 'misc') 
               self.miscObjects{self.miscObjectIndex} = object;
               self.miscObjectIndex = self.miscObjectIndex + 1;
               self.checkObjects{self.checkObjectIndex} = object;
               self.checkObjectIndex = self.checkObjectIndex + 1;
           end
           
           if strcmp(object.type, 'obstacle') 
               self.obstacleObjects{self.obstacleObjectIndex} = object;
               self.obstacleObjectIndex = self.obstacleObjectIndex + 1;
           end
           
           
       end
       %% Add Robot
       function AddRobot(self, robot)
           %self.robot(numel(self.robot)+1) = robot;
           self.robot = robot;
           
           
           %self.robotControllers{1, self.robotControllerIndex} = robotController;
           %self.robotControllerIndex = self.robotControllerIndex + 1;
       end
       %% Display Environment - only run once!
       function Display(self)         
           for i = 1:size(self.foundation,2)
               self.foundation(i).Display();
           end
           
           hold on;
           
           for i = 1:size(self.targets,2)
               self.targets{i}.Display();
           end
           
           for i = 1:size(self.deposit,2)
               self.deposit{i}.Display();
           end
           
           self.robot.Display();
           
           self.lightCurtain.Display();
                      
           axis equal;
           camlight;
       end
   end
end
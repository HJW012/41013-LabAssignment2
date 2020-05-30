%% Environment class - Tier 2 class to control environment - table, objects - initialised in Sumulation object
classdef Environment < handle
   properties
       foundation;
       targets;
       deposit;
       targetIndex = 1;
       depositIndex = 1;
       robot;
   end
   
   methods 
       function self = Environment(varargin)
           
       end
       %% Add generic object
       function AddObject(self, object)
           if strcmp(object.type, 'foundation') 
               self.foundation = object;
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
           
           self.robot.PlotAndColourRobot();
           
           axis equal;
           camlight;
       end
   end
end
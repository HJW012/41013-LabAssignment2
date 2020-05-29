%% Environment class - Tier 2 class to control environment - table, objects - initialised in Sumulation object
classdef Environment < handle
   properties
       foundation;
       robots = [];
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
               self.deposit = object;
           end
           
           if strcmp(object.type, 'misc') 
               self.miscObjects{self.miscObjectIndex} = object;
               self.miscObjectIndex = self.miscObjectIndex + 1;
           end
       end
       %% Add Robot
       function AddRobot(self, robot)
           self.robotControllers{1, self.robotControllerIndex} = robotController;
           self.robotControllerIndex = self.robotControllerIndex + 1;
       end
       %% Display Environment - only run once!
       function Display(self)   
           
           
           axis equal;
           camlight;
       end
   end
end
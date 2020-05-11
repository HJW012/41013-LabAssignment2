%% Environment class - Tier 2 class to control environment - table, objects - initialised in Sumulation object
classdef Environment < handle
   properties
       foundation;
       
       targets = [];
       targetCount = 1;
       miscObjects = [];
       miscObjectCount = 1;
       target1;
       target2;
       target3;
   end
   
   methods 
       function self = Environment(varargin)
           
       end
       %% Add target - adds targets to environment
       function AddTarget(self, object) 
           if strcmp(object.type, 'target1')
               self.target1 = object;
               self.targets{1, self.targetCount} = object;
               self.targetCount = self.targetCount + 1;
           end
           
           if strcmp(object.type, 'target2')
               self.target2 = object;
               self.targets{1, self.targetCount} = object;
               self.targetCount = self.targetCount + 1;
           end
           
           if strcmp(object.type, 'target3')
               self.target3 = object;
               self.targets{1, self.targetCount} = object;
               self.targetCount = self.targetCount + 1;
           end
       end
       %% Add foundation - adds foundation object (table) to environment
       function AddFoundation(self, object)
           self.foundation = object;
       end
       %% Set foundation pose
       function SetFoundationPose(self, pose)
           
       end
       %% Add generic object
       function AddObject(self, object)
           if strcmp(object.type, 'foundation') 
               self.AddFoundation(object);
           end
           
           if strcmp(object.type, 'target1') || strcmp(object.type, 'target2') || strcmp(object.type, 'target3') 
               self.AddTarget(object);
           end
           
           if strcmp(object.type, 'misc') 
               self.miscObjects{self.miscObjectCount} = object;
               self.miscObjectCount = self.miscObjectCount + 1;
           end
       end
       %% Add Robot
       function AddRobot(self, robot)
           
       end
       %% Set dump zone - sets location for targets to be placed upon completion
       function SetDumpZone(transform)
           
       end
       %% Move targets - NOT ANIMATED - moves targets to desired location, if no inputs, move to random locations within workspace
       function MoveTargets(top, bottom, pcb)
           
       end
       %% Optimise Base pose - bonus mark - optimum robot locations for given target poses
       function OptimiseBasePose(self)
           
       end
       %% Map Targets - decide which robot to use for which target based on distance from end effector to target
       function AssignTargets()
           
       end
       %% Show Volume - show working volume of arm - maybe have same function in environment class
       function ShowWorkingVolume()
            
       end
       %% Display Environment - displays foundation, targets
       function Display(self)   
           self.foundation.Display();
           
           for i = 1:numel(self.miscObjects)
              self.miscObjects{i}.Display(); 
           end
           
           
           self.target1.Display();
           self.target2.Display();
           self.target3.Display();
           
       end
       %% Set Misc Object Poses - cones etc
       function SetMiscObjectPoses(self)
          for i = 1:numel(self.miscObjects)
              self.miscObjects{i}.pose(1, 4) = self.miscObjects{i}.pose(1, 4) + self.foundation.pose(1, 4);
              self.miscObjects{i}.pose(2, 4) = self.miscObjects{i}.pose(2, 4) + self.foundation.pose(2, 4);
              self.miscObjects{i}.pose(3, 4) = self.miscObjects{i}.pose(3, 4) + self.foundation.pose(3, 4);
          end
       end
   end
end
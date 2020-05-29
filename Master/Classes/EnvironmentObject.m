%% Object class
classdef EnvironmentObject < handle
   properties
       type; %foundation, target1, target2, target3, deposit, misc
       pose;      
       dimensions; %length, width, height - probably hardcoded for simplicity's sake
       graspState = false;
       grabRobot;
       modelMesh;
       modelF;
       modelV;
       modelData;
       modelMidPoint;
       vertexColours;
       vertexCount;
       translate;
       generalColour;
       cameraPlot;
       cameraK;
   end
   
   methods
       function self = EnvironmentObject(varargin)  
           if 0 < nargin && mod(nargin, 2) == 0
              for i = 1:2:nargin
                  knownParam = 0;
                  
                  if strcmp(varargin{i}, 'ModelPath')
                      [self.modelF, self.modelV, self.modelData] = plyread(varargin{i+1}, "tri");
                      self.vertexColours = [self.modelData.vertex.red, self.modelData.vertex.green, self.modelData.vertex.blue] / 255;
                      self.vertexCount = size(self.modelV, 1);
                      self.modelMidPoint = sum(self.modelV)/self.vertexCount;
                      self.modelV = self.modelV-repmat(self.modelMidPoint, self.vertexCount, 1);
                      
                      knownParam = 1;
                  end
                  
                  if strcmp(varargin{i}, 'Pose')
                      self.pose = varargin{i+1};
                      knownParam = 1;
                  end
                  
                  if strcmp(varargin{i}, 'Type')
                      self.type = varargin{i+1};                      
                      knownParam = 1;
                  end
                  
                  if strcmp(varargin{i}, 'Dimensions')
                      self.dimensions = varargin{i+1};
                     knownParam = 1; 
                  end
                  
                  if strcmp(varargin{i}, 'GeneralColour')
                      self.generalColour = varargin{i+1};
                     knownParam = 1; 
                  end
                  
                  if knownParam == 0
                     warning("Unknown Param: " + varargin{i+1});
                  end
              end
              
           else
               warning("Warning - Object Class: Too Few Inputs");
           end
       end
       %% Display object in environment
       function Display(self)
           self.modelMesh = trisurf(self.modelF,self.modelV(:,1)+ self.pose(1,4),self.modelV(:,2) + self.pose(2,4), self.modelV(:,3) + self.pose(3,4) ...
                        ,'FaceVertexCData',self.vertexColours,'EdgeColor','interp','EdgeLighting','flat');
           self.SetPose(self.pose);
           drawnow();
       end
       %% Set Pose - move object to new pose and display
       function SetPose(self, pose)
           self.pose = pose;
           updatedPoints = [self.pose * [self.modelV, ones(self.vertexCount, 1)]']';
           self.modelMesh.Vertices = updatedPoints(:, 1:3);
       end
   end
end
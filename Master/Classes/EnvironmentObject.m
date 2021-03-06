%% Environment Object Class
classdef EnvironmentObject < handle
   properties
       type;            % foundation, target, deposit, misc, safety, obstacle
       pose;            % pose of the object  
       dimensions;      % length, width, height
       modelMesh;       % Mesh of the model containing Vertices and Faces
       modelF;          % Faces of the model
       modelV;          % Vertices of the model
       modelData;       % Model Data
       modelMidPoint;   % MidPoint of the model
       vertexColours;   % Colours of the vertices
       vertexCount;     % Number of vertices in the model
       generalColour;   % General colour of the object (assigned in object creation)
       cameraPlot;      % Plot of the object on the camera
       cameraK;         % Convhull results of the object produced in the camera object
   end
   
   methods
       % Constructor accepts variable number of arguments and used to
       % create the object from the ply file and initialise the properties
       % of the object
       function self = EnvironmentObject(varargin)  
           if 0 < nargin && mod(nargin, 2) == 0
              for i = 1:2:nargin
                  knownParam = 0;
                  
                  % Path to the ply file
                  if strcmp(varargin{i}, 'ModelPath')
                      % Read the ply file and store relevant data in
                      % appropriate object properties
                      [self.modelF, self.modelV, self.modelData] = plyread(varargin{i+1}, "tri");
                      self.vertexColours = [self.modelData.vertex.red, self.modelData.vertex.green, self.modelData.vertex.blue] / 255;
                      self.vertexCount = size(self.modelV, 1);
                      
                      % Calculate MidPoint of the model
                      self.modelMidPoint = sum(self.modelV)/self.vertexCount;
                      self.modelV = self.modelV-repmat(self.modelMidPoint, self.vertexCount, 1);
                      
                      knownParam = 1;
                  end
                  
                  % Pose of the object
                  if strcmp(varargin{i}, 'Pose')
                      self.pose = varargin{i+1};
                      knownParam = 1;
                  end
                  
                  % Type of object being created
                  if strcmp(varargin{i}, 'Type')
                      self.type = varargin{i+1};                      
                      knownParam = 1;
                  end
                  
                  % Dimensions of the object
                  if strcmp(varargin{i}, 'Dimensions')
                      self.dimensions = varargin{i+1};
                     knownParam = 1; 
                  end
                  
                  % General colour of the object
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
       % Simple function used to display the model in the environment.
       function Display(self)
           self.modelMesh = trisurf(self.modelF, (self.modelV(:,1) + self.pose(1,4)), (self.modelV(:,2) + self.pose(2,4) + self.modelMidPoint(1,2)), (self.modelV(:,3) + self.pose(3,4) + self.modelMidPoint(1,3)) ...
                        ,'FaceVertexCData',self.vertexColours,'EdgeColor','interp','EdgeLighting','flat');
           
           % Set the object pose
           self.SetPose(self.pose * transl(0,self.modelMidPoint(1,2),self.modelMidPoint(1,3)));         
           drawnow();
       end
       %% Set Pose - move object to new pose and display
       function SetPose(self, pose)
           % Set pose
           self.pose = pose;
           
           % Calculate updated model points and assign these points to the
           % vertices
           updatedPoints = [self.pose * [self.modelV, ones(self.vertexCount, 1)]']';
           self.modelMesh.Vertices = updatedPoints(:, 1:3);
       end
       %% Remove object
       % Function used to remove the object from visibility in the
       % simulation
       function Remove(self)
          % Move object to the origin
          self.SetPose(transl(0, 0, 0));
          
          % Set the transparency of the object to be completely transparent
          % (make the object invisible)
          self.modelMesh.FaceAlpha = 0;
          self.modelMesh.EdgeAlpha = 0;
          self.modelData.vertex.alpha(:) = 0;
       end
   end
end
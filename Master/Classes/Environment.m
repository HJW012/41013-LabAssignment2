%% Environment Class
classdef Environment < handle
   properties
       foundation = EnvironmentObject.empty;    % Foundation object (table) on which everything will sit
       targets;                                 % Matrix of the target objects to be collected and deposited
       deposit;                                 % Matrix of the deposit objects to which the targets must be moved
       miscObjects;                             % Matrix of miscelllaneous objects in the environment
       targetIndex = 1;                         % Index used to track the number of target objects
       depositIndex = 1;                        % Index used to track the number of deposit objects
       miscObjectIndex = 1;                     % Index used to track the number of misc objects
       lightCurtain = LightCurtain.empty;       % Light Curtain object surrounding the table
       robot;                                   % Dobot object in environment
       checkObjects;                            % Matrix of objects that need to be checked do not obstruct the light curtain
       checkObjectIndex = 1;                    % Index used to track the number of check objects
       obstacleObjects;                         % Matrix of objects that need to be checked do not collide with the Dobot
       obstacleObjectIndex = 1;                 % Index used to track the number of obstacles
       safetyObject;                            % Matrix of the safety objects installed in the environment
       safetyObjectIndex = 1;                   % Index used to track the number of safety objects
   end
   
   methods 
       % Constructor which accepts a variable number of arguments and used
       % to store the objects in the environment
       function self = Environment(varargin)
           % Iterate through each argument and add the object to the
           % environment
           for i = 1:1:nargin
               object = varargin{i};
               self.AddObject(object);
           end
           
           % Check if a light curtain does not exist, if not, create a
           % light curtain around the table
           if isequal(self.lightCurtain, LightCurtain.empty)
               self.lightCurtain = LightCurtain(self.foundation(1));             
           end
           
       end
       %% Add generic object
       % Function used to add objects into the environment
       function AddObject(self, object)
           % If a table has been passed, store it in the foundation object
           % and generate a light curtain
           if strcmp(object.type, 'foundation') 
               self.foundation = object;
               self.lightCurtain = LightCurtain(self.foundation);
           end
           
           % If a target object has been passed, store the object in the
           % targets matrix
           if strcmp(object.type, 'target')
               self.targets{1, self.targetIndex} = object;
               self.targetIndex = self.targetIndex + 1;
           end
           
           % If a deposit object has been passed, store the object in the
           % deposit matrix
           if strcmp(object.type, 'deposit')
               self.deposit{1, self.depositIndex} = object;
               self.depositIndex = self.depositIndex + 1;
           end
           
           % If a misc object has been passed, store the object in the
           % check objects matrix
           if strcmp(object.type, 'misc') 
               self.miscObjects{self.miscObjectIndex} = object;
               self.miscObjectIndex = self.miscObjectIndex + 1;
               self.checkObjects{self.checkObjectIndex} = object;
               self.checkObjectIndex = self.checkObjectIndex + 1;
           end
           
           % If an obstacle has been passed, store the object in the
           % obstacle objects matrix
           if strcmp(object.type, 'obstacle') 
               self.obstacleObjects{self.obstacleObjectIndex} = object;
               self.obstacleObjectIndex = self.obstacleObjectIndex + 1;
           end
           
           % If a safety object has been passed, store the object in the
           % safety object matrix
           if strcmp(object.type, 'safety')
              self.safetyObject{self.safetyObjectIndex} = object;
              self.safetyObjectIndex = self.safetyObjectIndex + 1;
           end
           
       end
       %% Draw Background
       % Function used to generate the background images
       function DrawBackground(self)
          % Read images into variables
          backgroundImg1 = imread('background1.jpg');
          backgroundImg2 = imread('background2.jpg');
          groundImg = imread('ground.jpg');
          
          % Generate backgrounds as surfaces in the environment
          % Background 2
          xImage = [-1.5 1.5; -1.5 1.5];
          yImage = [-1 -1; -1 -1];
          zImage = [1.5 1.5; 0 0];
          surf(xImage, yImage, zImage, 'CData', backgroundImg2, 'FaceColor', 'texturemap');
          
          % Generate backgrounds as surfaces in the environment
          % Background 1
          xImage = [-1.5 -1.5; -1.5 -1.5];
          yImage = [-1 1; -1 1];
          zImage = [1.5 1.5; 0 0];
          surf(xImage, yImage, zImage, 'CData', backgroundImg1, 'FaceColor', 'texturemap');
          
          % Generate backgrounds as surfaces in the environment
          % Ground
          xImage = [-1.5 -1.5; 1.5 1.5];
          yImage = [-1 1; -1 1];
          zImage = [0 0; 0 0];
          surf(xImage, yImage, zImage, 'CData', groundImg, 'FaceColor', 'texturemap');
       end
       %% Add Robot
       % Add robot into the environment
       function AddRobot(self, robot)
           self.robot = robot;
       end
       %% Display Environment
       % Function used to display all objects in the environment
       function Display(self)         
           % Display table
           for i = 1:size(self.foundation,2)
               self.foundation(i).Display();
           end
           
           hold on;
           
           % Display targets
           for i = 1:size(self.targets,2)
               self.targets{i}.Display();
           end
           
           % Display deposit locations
           for i = 1:size(self.deposit,2)
               self.deposit{i}.Display();
           end
           
           % Display safety objects
           for i = 1:size(self.safetyObject, 2)
              self.safetyObject{i}.Display(); 
           end
           
           % Display Dobot
           self.robot.Display();

           % Display the light curtain
           self.lightCurtain.Display();
              
           % Display the background images to create the realistic
           % environment
           self.DrawBackground();
           axis equal;
           camlight;
       end
   end
end
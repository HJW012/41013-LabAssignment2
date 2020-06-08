%% Dobot Class
classdef Dobot < handle
    properties
        model;                                              % Stores SerialLink model        
        linearRail;                                         % Attach linear rail for lateral movement - EnvironmentObject object
        linearRailAttached = false;                         % True after generating linear rail
        linearRailPose = transl(0, 0, 0);                   % Pose for LinearRail
        linearRailTravelDist = 1.125-0.157852;              % Total rail length - dobot base width
        remoteController;
        remoteControllerID;
        basePose = transl(0, 0, 0);                         % Base pose of Dobot
        baseWidth = 0.157852;                               % Width of Dobot base
        jointAngles = deg2rad([0, 60, 65, -35, 0]);         % Default robot joint angles
        remoteControllerAttached = false;
        teachDistance = 0.2;
        linkLength = [0.135, 0.139, 0.16, 0.05, 0.0625];    % Length of each SerialLink link
        stop = 0;                                           % Used for light curtain
        collisionDetected = 0;                              % used for collision detection
        object;                                             % Object being picked
        grabbedObject = EnvironmentObject.empty;
        objectOffset;
        
    end
    
    methods
        % Constructor with takes a variable number of arguments in and
        % assigns those arguments which it can differentiate
        function self = Dobot(varargin)
            if 0 < nargin && mod(nargin, 2) == 0
                for i = 1:2:nargin
                    knownParam = 0;
                    
                    % Check if the parameter is the base pose and assign
                    % the value to the robot
                    if strcmp(varargin{i}, 'BasePose')
                        self.basePose = varargin{i+1};
                        knownParam = 1;
                        disp('Info - Dobot Class: Base Pose set');
                    end
                    
                    % Check if the parameter is the default joint angles of
                    % the robot and assign the value to the default joint
                    % angles parameter
                    if strcmp(varargin{i}, 'JointAngles')
                        self.jointAngles = varargin{i+1};
                        knownParam = 1;
                        disp('Info - Dobot Class: Joint Angles set');
                    end
                    
                    % If a linear rail has been passed to the robot assign
                    % it to the linear rail parameter
                    if strcmp(varargin{i}, 'LinearRail')
                        self.linearRail = varargin{i+1};
                        knownParam = 1;
                        disp('Info - Dobot Class: Linear Rail set');
                    end
                    
                end
            end          
            % Generate Serial Link
            self.GetDobotRobot();

            % Set the base position of the robot on top of the table
            self.model.base = self.basePose;            
        end

        %% GetDobotRobot
        % Create and return a Dobot robot model - values from Dobot
        % reference document - verified on 3D CAD model
        function GetDobotRobot(self)
            pause(0.001);
            name = ['Dobot_',datestr(now,'yyyymmddTHHMMSSFFF')];

            % Generate Serial Link based on derived DH parameters
            L1 = Link('d',0.135,'a',0,'alpha',-pi/2,'qlim', deg2rad([-135,135]), 'offset', 0);
            L2 = Link('d',0,'a',0.139,'alpha',0,'qlim', deg2rad([5,80]), 'offset', -pi/2);
            L3 = Link('d',0,'a',0.160,'alpha',0,'qlim', deg2rad([15,170]), 'offset', 0);
            L4 = Link('d',0,'a',0.05,'alpha',-pi/2,'qlim', deg2rad([-90,90]), 'offset', 0);
            L5 = Link('d',0.0625,'a',0,'alpha',0,'qlim', deg2rad([-85,85]), 'offset', 0);

            % Create Serial link
            self.model = SerialLink([L1 L2 L3 L4 L5],'name',name);
            
            % Initialise robot to not be carrying any objects
            self.object = EnvironmentObject.empty;
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        % Loading models downloaded from Dobot support website
        function PlotAndColourRobot(self) % - Function derived from Peter Corke UR10 class constructor
            % Import ply files and assign to each robot link
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'notiles');
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight;
            end  
            self.model.delay = 0; % 0 to make animate fast

            % Try to correctly colour the arm (if colours are in ply file data)
            for linkIndex = 0:self.model.n
                handles = findobj('Tag', self.model.name);
                h = get(handles,'UserData');
                try 
                    h.link(linkIndex+1).Children.FaceVertexCData = [plyData{linkIndex+1}.vertex.red ...
                                                                  , plyData{linkIndex+1}.vertex.green ...
                                                                  , plyData{linkIndex+1}.vertex.blue]/255;
                    h.link(linkIndex+1).Children.FaceColor = 'interp';
                catch ME_1
                    disp(ME_1);
                    continue;
                end
            end
            self.model.animate(self.jointAngles); %Setting robot to initial joint angles
        end
        %% Display - Display robot in figure
        % Function to display the robot in the environment
        function Display(self, varargin)
           % Apply the colours and features to the model
           self.PlotAndColourRobot();
           hold on;
           % Display the linear rail and move the dobot to be mounted on
           % top of the rail
           if (self.linearRailAttached)
                q = self.model.getpos;
                self.model.animate(q);
                self.linearRail.Display();
                drawnow();
           end
        end
        %% Generate Linear Rail - does not display linear rail yet
        % Function used to generate the linear rail object
        function GenerateLinearRail(self, linearRailPose)
           % Set pose of linear rail
           self.linearRailPose = eye(4) * transl(linearRailPose);
           
           % Generate Environment Object and read the ply file of the
           % linear rail into the object
           self.linearRail = EnvironmentObject('ModelPath', 'LinearRail.ply', 'Pose', self.linearRailPose, 'Dimensions', [0 0 0.051]);
           
           % Update the base pose of the Dobot to be mounted on the top of
           % the linear rail at the lower limit position
           self.basePose = transl(self.linearRailPose(1, 4)+self.baseWidth/2 - self.linearRail.modelMidPoint(1,1), self.linearRailPose(2, 4), self.linearRailPose(3, 4) + self.linearRail.dimensions(3));
           self.model.base = self.basePose;
           
           % Set linear rail flag
           self.linearRailAttached = true;
        end
        %% Collision Check
        % This function was derived from work completed in '41013 -
        % Robotics' Laboratory 6 Exercises and modified to suit this
        % application.
        %
        %
        % Function used to check if any of the Dobot links collid with the
        % obstacle. Receives the calculated robot trajectory and the
        % vertices of the obstacle object being checked against.
        function check = CollisionCheck(self, qMatrix, obstaclePoints)
            % Set centre point of ellipse
            centrePoint = [0,0,0];
            
            % Initialise check flag to indicate that a collision has not
            % occurred
            check = 0;

            % Initialise the radii variable
            radii = zeros(5,3);

            % Iterate through each link of the Dobot
            for i = 1:6
                if 1 < i
                   % Set the radii to be influenced by the a and d
                   % parameters of the Dobot Serial Link. This allows for
                   % the ellipsoids to match the orientations of the joints
                   % at each iteration. Minimum radius of 6 cm was
                   % calculated by measuring the CAD model and providing a
                   % minimum Factor of Safety. This creates a generous
                   % ellipsoid around the robot, providing an envelope.
                   radii(i, :) = [(0.06 + 0.5*self.model.links(i-1).a),0.06,(0.06 + 0.5*self.model.links(i-1).d)];
                   
                   % Generate ellipsoid
                   [X,Y,Z] = ellipsoid(centrePoint(1), centrePoint(2), centrePoint(3), radii(i,1), radii(i,2), radii(i,3) );     
                elseif i <= 1
                    % Set the radii to be influenced by the a and d
                    % parameters of the Dobot Serial Link.
                    radii(i, :) = [(0.06 + 0.5*self.model.links(i).a),0.06,(0.06 + 0.5*self.model.links(i).d)];
                    % Generate ellipsoid
                    [X,Y,Z] = ellipsoid( centrePoint(1), centrePoint(2), centrePoint(3), radii(i,1), radii(i,2), radii(i,3) );     

                end

                % Update model points to match the ellipsoids
                self.model.points{i} = [X(:),Y(:),Z(:)];
                warning off;
                self.model.faces{i} = delaunay(self.model.points{i});    %Triangulation
                warning on;
            end

            % Iterate through each joint angle in the calculated trajectory
            for j = 1:1:size(qMatrix,1)
                % Update the latest joint position
                position = qMatrix(j,:);

                % Check for collision between the ellipsoids of the links
                % and the obstacle vertices.
                check = self.CheckEllipsoidCollision(obstaclePoints, position, radii);

                % If a collision has been detected break out of the for
                % loop
                if check == 1
                    break;
                end
                pause(0.01);
            end
        end
        %% Distance Formula
        % Simple function used to calculate the distance between two,
        % 3-dimensional poses
        function dist = poseDist(self, pose1, pose2)
           dist = abs(sqrt( (pose1(1, 4) - pose2(1, 4))^2 + (pose1(2, 4) - pose2(2, 4))^2 + (pose1(3, 4) - pose2(3, 4))^2 )); 
        end
        %% MoveObject - Move object with Robot
        function MoveObject(self, obj)
            % Get current EE Pose
            EEPose = self.model.fkine(self.model.getpos);
            
            % If current picked object isn't set in class properties - this
            % prevents calculating new relative transform in every loop
            % cycle
            if ~isequal(obj, self.grabbedObject)
               self.grabbedObject = obj;
               
               % Use offset transform to keep relative pose between picked
               % object and end effector - works with translation AND
               % rotation
               self.objectOffset = inv(EEPose) * obj.pose;
            end
            
            % Set new object location
            obj.SetPose(EEPose * self.objectOffset);
        end
        
        %% GetAlgebraicDist
        % This function was taken from '41013 - Robotics' Laboratory 6
        % Exercises as it was directly relevant and needed for this
        % application. This function has previously been developed by Gavin
        % Paul. 
        %
        %
        % Calculates the algebraic distance between points and the centre
        % of an ellipsoid, later used to determine whether or not the
        % points lay inside the ellipsoid
        function algebraicDist = GetAlgebraicDist(self, points, centrePoint, radii)
            algebraicDist = ((points(:,1)-centrePoint(1))/radii(1)).^2 ...
                          + ((points(:,2)-centrePoint(2))/radii(2)).^2 ...
                          + ((points(:,3)-centrePoint(3))/radii(3)).^2;
        end


        %% CheckEllipsoidCollision
        % This function was derived from work completed in '41013 - Robotics' 
        % Laboratory 6 Exercises and modified to suit this application
        %
        % Checks to see if an obstacle will collide with any of the links of the Dobot arm 
        % Determines whether the points of an obstacle object lay inside
        % any of the ellipsoids mapped to the robot links. If any points
        % lay inside the ellipsoids a collision has been detected and will
        % return 1, otherwise it will return 0
        function collisionCheck = CheckEllipsoidCollision(self, obstaclePoints, jointAngles, radii)
            % Set the centre point of the ellipsoid
            centrePoint = [0 0 0];
            
            % Generate a matrix of the forward kinematics of the
            % Dobot using the DH parameters of the SerialLink
            fkMatrix = zeros(4,4,self.model.n+1);
            fkMatrix(:,:,1) = self.model.base;
            L = self.model.links;
            for i = 1 : self.model.n
                fkMatrix(:,:,i+1) = fkMatrix(:,:,i) * trotz(jointAngles(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            end

            % Iterate through each point of the forward kinematic chain
            for i = 1: size(fkMatrix,3)
                % Transform the obstacle Vertices to the ith link position
                pointsAndOnes = [inv(fkMatrix(:,:,i)) * [obstaclePoints,ones(size(obstaclePoints,1),1)]']';
                % Update the points to check against the ellipse
                updatedPoints = pointsAndOnes(:,1:3);
                
                % Get the relevant radii
                radius = radii(i,:);
                
                % Calculate the algebraic distance between the updated
                % points and the ellipsoid
                algebraicDist = self.GetAlgebraicDist(updatedPoints, centrePoint, radius);
                
                % Calculate the points inside the ellipsoid
                pointsInside = find(algebraicDist < 1);
                
                % If there are more than zero points inside the ellipsoid
                % the robot has collided with the obstacle, set flag to 1
                % and break out of the for loop
                if 0 < pointsInside
                    collisionCheck = 1;
                    break;
                else
                    collisionCheck= 0;
                end

            end
        end
    end
end
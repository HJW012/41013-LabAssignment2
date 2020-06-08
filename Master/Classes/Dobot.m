classdef Dobot < handle
    properties
        model; % Stores SerialLink model
                
        linearRail; %Attach linear rail for lateral movement - EnvironmentObject object
        
        linearRailAttached = false; % True after generating linear rail
        
        linearRailPose = transl(0, 0, 0); % Pose for LinearRail
        
        linearRailTravelDist = 1.125-0.157852; %Total rail length - dobot base width
        
        remoteController;
        
        remoteControllerID;
        
        basePose = transl(0, 0, 0); % Base pose of Dobot
        
        baseWidth = 0.157852; % Width of Dobot base
        
        jointAngles = deg2rad([0, 60, 65, -35, 0]); % Default robot joint angles
        
        remoteControllerAttached = false;
        
        teachDistance = 0.2;
        
        linkLength = [0.135, 0.139, 0.16, 0.05, 0.0625]; % Length of each SerialLink link
        
        stop = 0; % Used for light curtain
        
        collisionDetected = 0; % used for collision detection
        
        object; %Object being picked
        
        grabbedObject = EnvironmentObject.empty;
        
        objectOffset;
        
    end
    
    methods
        function self = Dobot(varargin)
            if 0 < nargin && mod(nargin, 2) == 0
                for i = 1:2:nargin
                    knownParam = 0;
                    
                    if strcmp(varargin{i}, 'BasePose')
                        self.basePose = varargin{i+1};
                        knownParam = 1;
                        disp('Info - Dobot Class: Base Pose set');
                    end
                    
                    if strcmp(varargin{i}, 'JointAngles')
                        self.jointAngles = varargin{i+1};
                        knownParam = 1;
                        disp('Info - Dobot Class: Joint Angles set');
                    end
                    
                    if strcmp(varargin{i}, 'LinearRail')
                        self.linearRail = varargin{i+1};
                        knownParam = 1;
                        disp('Info - Dobot Class: Linear Rail set');
                    end
                    
                    if knownParam == 0
                        %warning("Warning - Dobot Class: Unknown Input: " + varargin{i+1});
                    end
                end
            else
                %warning("Warning - Dobot Class: Too Few Inputs");
            end          
            % Generate Serial Link
            self.GetDobotRobot();

            % Set the base position of the robot on top of the table
            self.model.base = self.basePose;            
        end

        %% GetDobotRobot
        % Create and return a Dobot robot model - values from Dobot
        % reference document - verified on 3d model
        function GetDobotRobot(self)
            pause(0.001);
            name = ['Dobot_',datestr(now,'yyyymmddTHHMMSSFFF')];

            % Generate Serial Link based on derived DH parameters
            L1 = Link('d',0.135,'a',0,'alpha',-pi/2,'qlim', deg2rad([-135,135]), 'offset', 0);
            L2 = Link('d',0,'a',0.139,'alpha',0,'qlim', deg2rad([5,80]), 'offset', -pi/2);
            L3 = Link('d',0,'a',0.160,'alpha',0,'qlim', deg2rad([15,170]), 'offset', 0);
            L4 = Link('d',0,'a',0.05,'alpha',-pi/2,'qlim', deg2rad([-90,90]), 'offset', 0);
            L5 = Link('d',0.0625,'a',0,'alpha',0,'qlim', deg2rad([-85,85]), 'offset', 0);

            self.model = SerialLink([L1 L2 L3 L4 L5],'name',name);
            
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
        function Display(self, varargin)
           self.PlotAndColourRobot();
           hold on;
           if (self.linearRailAttached)
                q = self.model.getpos;
                self.model.animate(q);
                self.linearRail.Display();
                drawnow();
           end
        end
        %% Generate Linear Rail - does not display linear rail yet
        function GenerateLinearRail(self, linearRailPose)
           self.linearRailPose = eye(4) * transl(linearRailPose);
           self.linearRail = EnvironmentObject('ModelPath', 'LinearRail.ply', 'Pose', self.linearRailPose, 'Dimensions', [0 0 0.051]);
           self.basePose = transl(self.linearRailPose(1, 4)+self.baseWidth/2 - self.linearRail.modelMidPoint(1,1), self.linearRailPose(2, 4), self.linearRailPose(3, 4) + self.linearRail.dimensions(3));
           self.model.base = self.basePose;
           self.linearRailAttached = true;
        end
        %% Collision Check
        function check = CollisionCheck(self, qMatrix, obstaclePoints)
            centrePoint = [0,0,0];
            check = 0;

            radii = zeros(5,3);

            for i = 1:6
                if 1 < i
                   radii(i, :) = [(0.06 + 0.5*self.model.links(i-1).a),0.06,(0.06 + 0.5*self.model.links(i-1).d)];
                   [X,Y,Z] = ellipsoid(centrePoint(1), centrePoint(2), centrePoint(3), radii(i,1), radii(i,2), radii(i,3) );     
                elseif i <= 1
                    radii(i, :) = [(0.06 + 0.5*self.model.links(i).a),0.06,(0.06 + 0.5*self.model.links(i).d)];
                    [X,Y,Z] = ellipsoid( centrePoint(1), centrePoint(2), centrePoint(3), radii(i,1), radii(i,2), radii(i,3) );     

                end

                self.model.points{i} = [X(:),Y(:),Z(:)];
                warning off;
                self.model.faces{i} = delaunay(self.model.points{i});    %Triasngulation

            end

            for j = 1:1:size(qMatrix,1)
                position = qMatrix(j,:);

                check = self.CheckEllipsoidCollision(obstaclePoints, position, radii);

                if check == 1
                    break;
                end
                pause(0.01);
            end
        end
        
        
        
        %% Attach Remote Controller
        function AttachRemoteControl(self, varargin)
            if 0 < nargin && mod(nargin, 2) == 0
                for i = 1:2:nargin
                    knownParam = 0;
                    
                    if strcmp(varargin{i}, 'ID')
                        self.remoteControllerID = varargin{i+1};
                        knownParam = 1;
                        disp('Info - Dobot Class: Remote Controller ID set');
                    end
                    
                    if knownParam == 0
                        warning("Warning - Dobot Class: Unknown Input: " + varargin{i+1});
                    end
                end
            else
                warning("Warning - Dobot Class: Too Few Inputs");
            end  
            
            self.remoteControllerAttached = true;
            self.remoteController = vrjoystick(self.remoteControllerID);
        end
        %% Distance Formula
        function dist = poseDist(self, pose1, pose2)
           dist = abs(sqrt( (pose1(1, 4) - pose2(1, 4))^2 + (pose1(2, 4) - pose2(2, 4))^2 + (pose1(3, 4) - pose2(3, 4))^2 )); 
        end
        %% Move Object
        function MoveObject(self, obj)
            EEPose = self.model.fkine(self.model.getpos);
            if ~isequal(obj, self.grabbedObject)
               self.grabbedObject = obj;
               self.objectOffset = inv(EEPose) * obj.pose;
            end
            
            obj.SetPose(EEPose * self.objectOffset);
        end
        
        %% GetAlgebraicDist
        % This function was derived from Laboratory 6 exercises
        % Calculates the algebraic distance between points and the centre
        % of an ellipsoid, later used to determine whether or not the
        % points lay inside the ellipsoid
        function algebraicDist = GetAlgebraicDist(self, points, centrePoint, radii)
            algebraicDist = ((points(:,1)-centrePoint(1))/radii(1)).^2 ...
                          + ((points(:,2)-centrePoint(2))/radii(2)).^2 ...
                          + ((points(:,3)-centrePoint(3))/radii(3)).^2;
        end


        %% CheckEllipsoidCollision
        % This function was derived from Laboratory 6 Exercises
        % Checks to see if an obstacle will collide with any of the links of the Dobot arm 
        % Determines whether the points of an obstacle object lay inside
        % any of the ellipsoids mapped to the robot links. If any points
        % lay inside the ellipsoids a collision has been detected and will
        % return 1, otherwise it will return 0
        function collisionCheck = CheckEllipsoidCollision(self, obstaclePoints, jointAngles, radii)
            centrePoint = [0 0 0];
            tr = zeros(4,4,self.model.n+1);
            tr(:,:,1) = self.model.base;
            L = self.model.links;
            for i = 1 : self.model.n
                tr(:,:,i+1) = tr(:,:,i) * trotz(jointAngles(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            end

            % Go through each ellipsoid
            for i = 1: size(tr,3)
                pointsAndOnes = [inv(tr(:,:,i)) * [obstaclePoints,ones(size(obstaclePoints,1),1)]']';
                updatedPoints = pointsAndOnes(:,1:3);
                radius = radii(i,:);
                algebraicDist = self.GetAlgebraicDist(updatedPoints, centrePoint, radius);
                pointsInside = find(algebraicDist < 1);
                if 0 < pointsInside
                    collisionCheck = 1;
                   % disp(['There are ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);
                    break;
                else
                    collisionCheck= 0;
                end

            end
        end
    end
end
classdef Dobot < handle
    properties
        model; 
        
        gripperLength = 0.0625; %Length of gripper - cant directly include in DH params
        
        linearRail; %Attach linear rail for lateral movement
        
        linearRailAttached = false;
        
        linearRailPose = transl(0, 0, 0);
        
        linearRailTravelDist = 1.125-0.157852; %Total rail length - dobot base width
        
        linearRailLength = 1.125;
        
        linearRailTargetPose = 0;
        
        remoteController;
        
        remoteControllerID;
        
        basePose = transl(0, 0, 0);
        
        baseWidth = 0.157852;
        
        jointAngles = deg2rad([0, 60, 65, -35, 0]);
        
        remoteControllerAttached = false;
        
        teachDistance = 0.2;
        
        linkLength = [0.135, 0.139, 0.16, 0.05, 0.0625];
        
        stop = false;
        
        
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
                        warning("Warning - Dobot Class: Unknown Input: " + varargin{i+1});
                    end
                end
            else
                warning("Warning - Dobot Class: Too Few Inputs");
            end          
            % Generate Serial Link
            self.GetDobotRobot();

            % Set the base position of the robot on top of the table
            self.model.base = self.basePose;            
        end

        %% GetDobotRobot
        % Create and return a Dobot robot model
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
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
        % Loading models downloaded from Dobot support website
        function PlotAndColourRobot(self)
            % Import ply files and assign to each robot link
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            % Display robot
            self.model.plot3d(zeros(1,self.model.n),'notiles');
            if isempty(findobj(get(gca,'Children'),'Type','Light'))
                camlight
            end  
            self.model.delay = 0;

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
            self.model.animate(self.jointAngles);
        end
        %% Plot - used to keep end effector parallel to ground
        function plot(self, q)
           localQ = q;
           localQ(5) = 0;
           %Need to calculate q4
           %Q1 is irrelevent
           % q2 is in relation to vertical axis
           omega = localQ(2) + localQ(3);
           sigma = 90 - omega;
           localQ(4) = sigma;
           self.model.plot(deg2rad(localQ));
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
        %% Generate Target Joint Angles
        function q = GenerateTargetJointAngles(self, targetPose)
            targetX = targetPose(1, 4) - self.model.base(1, 4);
            targetY = targetPose(2, 4) - self.model.base(2, 4);
            targetZ = self.model.base(3, 4) - targetPose(3, 4)
            theta = atan2(targetY, targetX) + -asin(self.model.base(1, 2));
            q(1) = theta;
            joint4X = targetX - self.linkLength(4) * cos(theta) % Get required joint 4 pose
            joint4Y = targetY - self.linkLength(4) * sin(theta)
            joint4Z = targetZ + self.linkLength(5)
            h = joint4Z - self.linkLength(1);
            
            syms x y z q2 q3 a2 a3; % Symbolic algebraic solution to find q2
            f1 = a2*sin(q2)+a3*cos(q3) == sqrt(x^2+y^2);
            f2 = a2*cos(q2)-a3*sin(q3) == z;
            solution = solve([f1,f2],[q2,q3]);
            q2sym1 = (subs(solution.q2(1), {x, y, z, a2, a3}, {joint4X joint4Y h self.linkLength(2) self.linkLength(3)}));
            q2sym2 = (subs(solution.q2(2), {x, y, z, a2, a3}, {joint4X joint4Y h self.linkLength(2) self.linkLength(3)}));
            q21 = vpa(q2sym1)
            q22 = vpa(q2sym2)

            
            if self.model.qlim(2, 1) <= q21 && q21 <= self.model.qlim(2, 2)
               q(2) = q21
            elseif self.model.qlim(2, 1) <= q22 && q22 <= self.model.qlim(2, 2)
               q(2) = q22
            end
            
            joint1X = self.model.base(1, 4);
            joint1Y = self.model.base(2, 4);
            joint1Z = self.linkLength(1);

            l = sqrt( (joint4X)^2 + (joint4Y)^2 + (joint4Z-joint1Z)^2 );
            sigma = acos((self.linkLength(2)^2 + self.linkLength(3)^2 - l^2)/(2 * self.linkLength(2) * self.linkLength(3)));
            
            q(3) = pi - sigma;
            q(4) = pi/2 - (q(2) + q(3));
            q(5) = acos(targetPose(1, 1));
            
            withinLimits = true;
            for i = 1:5
                if (q(i) > self.model.qlim(i, 2) || q(i) < self.model.qlim(i, 1))
                   withinLimits = false;
                end
            end
            if ~withinLimits
                disp('Joint Angles Outside of Joint Limits');
            end
        end
        %% Generate Target Joint Angles 2 - Using personal calculations - may only work with base facing in positive X direction
        function q = GenerateTargetJointAngles2(self, basePose, targetPose)
            targetX = targetPose(1, 4);
            targetY = targetPose(2, 4);
            targetZ = targetPose(3, 4);
            baseX = basePose(1, 4);
            baseY = basePose(2, 4);
            baseZ = basePose(3, 4);
            
            % 1.0 - Determining q1
            theta = atan(abs((targetY - baseY)/(targetX - baseX)));
            
            % 1.1 - Case 1 - Target greater in X and Y
            if targetX > baseX && targetY > baseY
                J4x = targetX - self.linkLength(4) * cos(theta);
                J4y = targetY - self.linkLength(4) * sin(theta);
                q(1) = theta;
                
                disp("Case 1.1");
            end
            
            % 1.2 - Case 2 - Target greater in Y and lesser in X
            if targetX < baseX && targetY > baseY
                J4x = targetX + self.linkLength(4) * cos(theta);
                J4y = targetY - self.linkLength(4) * sin(theta);
                q(1) = theta + pi/2;
                
                disp("Case 1.2");
            end
            
            % 1.3 - Case 3 - Target lesser in X and Y
            if targetX < baseX && targetY < baseY
                J4x = targetX + self.linkLength(4) * cos(theta);
                J4y = targetY + self.linkLength(4) * sin(theta);
                q(1) = -pi/2 - theta;
                
                disp("Case 1.3");
            end
            
            % 1.4 - Case 4 - Target greater in X and lesser in Y
            if targetX > baseX && targetY < baseY
                J4x = targetX - self.linkLength(4) * cos(theta);
                J4y = targetY + self.linkLength(4) * sin(theta);
                q(1) = -theta;
                
                disp("Case 1.4");
            end
            
            % 1.5 - Case 5 - Target equal in X and lesser in Y
            if targetX == baseX && targetY < baseY
                q(1) = -pi/2;
                J4x = targetX;
                J4y = targetY + self.linkLength(4);
                
                disp("Case 1.5");
            end
            
            % 1.6 - Case 6 - Target equal in X and greater in Y
            if targetX == baseX && targetY > baseY
                q(1) = pi/2;
                J4x = targetX
                J4y = targetY - self.linkLength(4)
                
                disp("Case 1.6");
            end
            
            % 1.7 - Case 7 - Target lesser in X and equal in Y
            if targetX < baseX && targetY == baseY
                q(1) = pi;
                J4x = targetX + self.linkLength(4);
                J4y = targetY;
                
                disp("Case 1.7");
            end
            
            % 1.8 - Case 8 - Target greater in X and equal in Y
            if targetX > baseX && targetY == baseY
                q(1) = 0;
                J4x = targetX - self.linkLength(4);
                J4y = targetY;
                
                disp("Case 1.8");
            end
            
            J4z = targetZ + self.linkLength(5)
            
            % 2.0 - Determine remaining joint angles
            J2x = baseX
            J2y = baseY
            J2z = baseZ + self.linkLength(1)
            length = abs(sqrt( (J2x - J4x)^2 + (J2y - J4y)^2 + (J2z - J4z)^2 ))
            alpha = acos( ((self.linkLength(2))^2 + (self.linkLength(3))^2 + length^2) / 2 * self.linkLength(2) * self.linkLength(3) )
            gamma = acos( ((self.linkLength(2))^2 + (length)^2 - (self.linkLength(3))^2)/(2 * self.linkLength(2) * length) );
            h = abs(J4z - J2z)
            
            % 2.1 - Joint 4 Z greater than Joint 2 Z
            if J4z > J2z
                tempZ = J4z - h;
                length2 = sqrt((J2x - J4x)^2 + (J2y - J4y)^2 + (J2z - tempZ)^2);
                delta = atan(length2, h);
                q(2) = pi - pi/2 - delta - gamma;
                
                
                
                disp("Case 2.1");
            end
            
            % 2.2 - Joint 4 Z lower than Joint 2 Z
            if J4z < J2z
                tempZ = J4z - h;
                length2 = sqrt((J2x - J4x)^2 + (J2y - J4y)^2 + (J2z - tempZ)^2)
                delta = atan2(length2, h);
                q(2) = pi - delta - gamma;
                length * sin(delta)
                disp("Case 2.2");
            end
            
            % 2.3 - Joint 4 Z equal to Joint 2 Z
            if J4z == J2z
                q(2) = pi - pi/2 - gamma;
                
                disp("Case 2.3");
            end
            
            
            q(3) = pi - alpha;
            sigma = q(2) + q(3);
            q(4) = -(sigma - pi/2);
            
            %NEED TO HANDLE q5
            q(5) = 0;
        end
        %% Generate Linear Rail - does not display linear rail yet
        function GenerateLinearRail(self, linearRailPose)
           self.linearRailPose = eye(4) * transl(linearRailPose);
           self.linearRail = EnvironmentObject('ModelPath', 'LinearRail.ply', 'Pose', self.linearRailPose, 'Dimensions', [0 0 0.051]);
           self.basePose = transl(self.linearRailPose(1, 4)+self.baseWidth/2 - self.linearRail.modelMidPoint(1,1), self.linearRailPose(2, 4), self.linearRailPose(3, 4) + self.linearRail.dimensions(3));
           self.model.base = self.basePose;
           self.linearRailAttached = true;
        end
        %% Move on linear rail
        function MoveLinearRail(self, dist, dir)
            if strcmp(dir, "fwd")
               if (self.model.base(1, 4) < (self.linearRail.pose(1, 4) + self.baseWidth/2 + self.linearRailTravelDist))
                   if (self.model.base(1, 4) + dist >= (self.linearRail.pose(1, 4) + self.baseWidth/2 + self.linearRailTravelDist))
                       self.model.base(1, 4) = self.linearRail.pose(1, 4) + self.baseWidth/2 + self.linearRailTravelDist;
                   else
                       self.model.base = self.model.base * transl(dist, 0, 0);   
                   end
                   self.model.animate(self.model.getpos);
               end
            elseif strcmp(dir, "back")
                
            else
                disp("Direction can only be fwd or back");
            end
        end
        %% Move linear rail to target location
        function MoveToTargetLinearRail(self, x)
            disp("X is : " + x);
            steps = 50;
            distanceToTravel = x - self.model.base(1,4);
           if (abs(distanceToTravel) + self.baseWidth/2 <= self.linearRailLength)
              increment = distanceToTravel/steps;
              for i = 1:steps
                  self.model.base(1, 4) = self.model.base(1,4) + increment;
                  self.model.animate(self.model.getpos);
                  drawnow();
              end
              disp(1);
           end
           
           if (distanceToTravel + self.baseWidth/2 < -1*self.linearRailLength)
              distance = (self.linearRailPose(1, 4) - self.linearRail.modelMidPoint(1,1) + self.baseWidth/2) - self.model.base(1,4);
              increment = distance/steps;
              
              for i = 1:steps
                  self.model.base(1, 4) = self.model.base(1,4) + increment;
                  self.model.animate(self.model.getpos);
                  drawnow();
              end
              disp(2);
           end
           
           if (distanceToTravel + self.baseWidth/2 > self.linearRailLength)
              distance = (self.linearRailPose(1, 4) - self.linearRail.modelMidPoint(1,1) + self.linearRailTravelDist + self.baseWidth/2) - self.model.base(1,4);
              increment = distance/steps;
              
              for i = 1:steps
                  self.model.base(1, 4) = self.model.base(1,4) + increment;
                  self.model.animate(self.model.getpos);
                  drawnow();
              end
              disp(3);
           end
        end
        %% MoveToTargetLinearRail2 - receives pose and chooses optimal point of on linear rail to move to
        function MoveToTargetLinearRail2(self, targetPose)
            LRBasePose = self.linearRail.pose;
            minLRX = LRBasePose(1, 4) + self.baseWidth/2;
            maxLRX = LRBasePose(1, 4) + self.linearRailTravelDist - self.baseWidth/2;
            targetX = targetPose(1, 4);
            targetY = targetPose(2, 4);
            targetZ = targetPose(3, 4);
            baseX = self.model.base(1, 4);
            baseY = self.model.base(2, 4);
            baseZ = self.model.base(3, 4);
            
            if (targetX < minLRX)
                possibleBaseX = minLRX;
            elseif (targetX > maxLRX)
                possibleBaseX = maxLRX;
            else
                possibleBaseX = targetX;
            end
            
            possibleBase = transl(possibleBaseX, baseY, baseZ)
            possibleQ = self.GenerateTargetJointAngles2(possibleBase, targetPose);
            offsetX = possibleBase(1, 4) - baseX;
            
            possibleEEPose = self.model.fkine(possibleQ);
            possibleEEPose(1, 4) = possibleEEPose(1, 4) + offsetX;
            error = self.poseDist(possibleEEPose, possibleBase)
            if error < 0.01
                self.model.base = possibleBase;
                self.model.animate(possibleQ);
            else %If possible base pose doesnt work
                if targetX > minLRX && targetX < maxLRX 
                    %If not at linear rail extents, 
                    %Check how close target Y value is to linear rail and
                    %move away to make picking possible
                end
            end
        end
        %% Move Arm
        function MoveArm(self, targetPose)
            q0 = self.model.getpos

            EEPose1 = targetPose
            q1 = self.GenerateTargetJointAngles2(EEPose1)
            steps = 50;
            s = lspb(0,1,steps);
            qMatrix = nan(steps,5);

            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q0 + s(i)*q1;
            end

            for i = 1:steps
               self.model.animate(qMatrix(i, :));
               drawnow();
            end
        end
        
        %% Collision Check
        function CollisionCheck(self, qMatrix)
            centrePoint = [0,0,0];

            radii = zeros(5,3);

            for i = 1:6
                if 1 < i
                   radii(i, :) = [(0.06 + 0.5*self.model.links(i-1).a),0.06,(0.06 + 0.5*self.model.links(i-1).d)];
                   [X,Y,Z] = ellipsoid( centrePoint(1), centrePoint(2), centrePoint(3), radii(i,1), radii(i,2), radii(i,3) );     
                elseif i <= 1
                    radii(i, :) = [(0.06 + 0.5*self.model.links(i).a),0.06,(0.06 + 0.5*self.model.links(i).d)];
                    [X,Y,Z] = ellipsoid( centrePoint(1), centrePoint(2), centrePoint(3), radii(i,1), radii(i,2), radii(i,3) );     

                end

                self.model.points{i} = [X(:),Y(:),Z(:)];
                warning off
                self.model.faces{i} = delaunay(Dobot_1.points{i});    
                warning on;
            end
            
            % One side of the cube. NEED TO WORK OUT HOW TO CREATE THESE
            % CUBES AROUND THE OBJECTS ON THE TABLE
            [Y,Z] = meshgrid(-0.1:0.01:0.1,-0.1:0.01:0.1);
            sizeMat = size(Y);
            X = repmat(0.1,sizeMat(1),sizeMat(2));

            % Combine one surface as a point cloud
            cubePoints = [X(:),Y(:),Z(:)];

            % Make a cube by rotating the single side by 0,90,180,270, and around y to make the top and bottom faces
            cubePoints = [ cubePoints ...
                         ; cubePoints * rotz(pi/2)...
                         ; cubePoints * rotz(pi) ...
                         ; cubePoints * rotz(3*pi/2) ...
                         ; cubePoints * roty(pi/2) ...
                         ; cubePoints * roty(-pi/2)];    

            % Plot the cube's point cloud
            cube1Points = cubePoints + repmat([0.3,0,0.2],size(cubePoints,1),1);
            cube1_h = plot3(cube1Points(:,1),cube1Points(:,2),cube1Points(:,3),'r.');

            cube2Points = cubePoints + repmat([0,-0.3,0.1],size(cubePoints,1),1);
            cube2_h = plot3(cube2Points(:,1),cube2Points(:,2),cube2Points(:,3),'b.');

            cubes = zeros(2646,3,2);
            cubes(:,:,1) = cube1Points;
            cubes(:,:,2) = cube2Points;
            for j = 1:1:size(qMatrix,1)
                position = qMatrix(j,:);

                check = CheckEllipsoidCollision(cubes, position, self.model, radii);

                if check == 1
                    disp('Collision detected in trajectory, human intervention required to remove obstruction');
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
        %% Advanced Teach function
        function AdvancedTeach(self, varargin)
            bgcol = [250 100 100] / 255;
            height = 0.06;
            
            opt.deg = true;
            opt.orientation = {'rpy', 'eul', 'approach'};
            opt.callback = [];    
            [opt,args] = tb_optparse(opt, varargin);
            
            handles.orientation = opt.orientation;
            handles.callback = opt.callback;
            handles.opt = opt;
            
            % we need to have qlim set to finite values for a prismatic joint
            qlim = self.model.qlim;
            if any(isinf(qlim))
                error('RTB:advancedTeach:badarg', 'Must define joint coordinate limits for prismatic axes, set qlim properties for prismatic Links');
            end

            if isempty(args)
                q = [];
            else
                q = args{1};
            end

            % set up scale factor, from actual limits in radians/metres to display units
            qscale = ones(self.model.n,1);
            for j=1:self.model.n
                L=self.model.links(j);
                if opt.deg && L.isrevolute
                    qscale(j) = 180/pi;
                end
            end

            handles.qscale = qscale;
            handles.robot = self.model;
            
            %---- install the panel at the side of the figure
    
            % find the right figure to put it in
            c = findobj(gca, 'Tag', self.model.name);  % check the current axes
            if isempty(c)
                % doesn't exist in current axes, look wider
                c = findobj(0, 'Tag', self.model.name);  % check all figures
                if isempty(c)
                    % create robot in arbitrary pose
                    self.model.plot( zeros(1, self.model.n) );
                    ax = gca;
                else
                    ax = get(c(1), 'Parent'); % get first axis holding the robot
                end
            else
                % found it in current axes
                ax = gca;
            end
            handles.fig = get(ax, 'Parent');  % get the figure that holds the axis

            % shrink the current axes to make room
            %   [l b w h]
            set(ax, 'OuterPosition', [0.25 0.25 0.70 0.7])

            handles.curax = ax;

            % create the panel itself
            panel = uipanel(handles.fig, ...
                'Title', 'advancedTeach', ...
                'BackGroundColor', bgcol,...
                'Position', [0 0 .25 1]);
            set(panel, 'Units', 'pixels'); % stop automatic resizing
            
            panel2 = uipanel(handles.fig, ...
                'BackGroundColor', bgcol, ...
                'Position', [0.25 0 0.75 0.25]);
            set(panel2, 'Units', 'pixels');
            
            handles.panel1 = panel;
            handles.panel2 = panel2;
            set(handles.fig, 'Units', 'pixels');
            set(handles.fig, 'ResizeFcn', @(src,event) self.resize_callback(self.model, handles));
            
            
             %---- get the current robot state
    
            if isempty(q)
                % check to see if there are any graphical robots of this name
                rhandles = findobj('Tag', self.model.name);

                % find the graphical element of this name
                if isempty(rhandles)
                    error('RTB:advancedTeach:badarg', 'No graphical robot of this name found');
                end
                % get the info from its Userdata
                info = get(rhandles(1), 'UserData');

                % the handle contains current joint angles (set by plot)
                if ~isempty(info.q)
                    q = info.q;
                end
            else
            self.model.plot(q);
            end
            handles.q = q;
            T6 = self.model.fkine(q);
                          
            %---- now make the sliders
            n = self.model.n;
            for j=1:n
                % slider label
                uicontrol(panel, 'Style', 'text', ...
                    'Units', 'normalized', ...
                    'BackgroundColor', bgcol, ...
                    'Position', [0 height*(n-j+2) 0.15 height], ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.5, ...
                    'String', sprintf('q%d', j));

                % slider itself
                q(j) = max( qlim(j,1), min( qlim(j,2), q(j) ) ); % clip to range
                handles.slider(j) = uicontrol(panel, 'Style', 'slider', ...
                    'Units', 'normalized', ...
                    'Position', [0.15 height*(n-j+2) 0.65 height], ...
                    'Min', qlim(j,1), ...
                    'Max', qlim(j,2), ...
                    'Value', q(j), ...
                    'Tag', sprintf('Slider%d', j));

                % text box showing slider value, also editable
                handles.edit(j) = uicontrol(panel, 'Style', 'edit', ...
                    'Units', 'normalized', ...
                    'Position', [0.80 height*(n-j+2)+.01 0.20 0.9*height], ...
                    'BackgroundColor', bgcol, ...
                    'String', num2str(qscale(j)*q(j), 3), ...
                    'HorizontalAlignment', 'left', ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.4, ...
                    'Tag', sprintf('Edit%d', j));
            end

            %---- set up the position display box

            % X
            uicontrol(panel, 'Style', 'text', ...
                'Units', 'normalized', ...
                'BackgroundColor', bgcol, ...
                'Position', [0.05 1-height 0.2 height], ...
                'FontUnits', 'normalized', ...
                'FontSize', 0.9, ...
                'HorizontalAlignment', 'left', ...
                'String', 'x:');

            handles.t6.t(1) = uicontrol(panel, 'Style', 'text', ...
                'Units', 'normalized', ...
                'Position', [0.3 1-height 0.6 height], ...
                'FontUnits', 'normalized', ...
                'FontSize', 0.8, ...
                'String', sprintf('%.3f', T6(1,4)), ...
                'Tag', 'T6');

            % Y
            uicontrol(panel, 'Style', 'text', ...
                'Units', 'normalized', ...
                'BackgroundColor', bgcol, ...
                'Position', [0.05 1-2*height 0.2 height], ...
                'FontUnits', 'normalized', ...
                'FontSize', 0.9, ...
                'HorizontalAlignment', 'left', ...
                'String', 'y:');

            handles.t6.t(2) = uicontrol(panel, 'Style', 'text', ...
                'Units', 'normalized', ...
                'Position', [0.3 1-2*height 0.6 height], ...
                'FontUnits', 'normalized', ...
                'FontSize', 0.8, ...
                'String', sprintf('%.3f', T6(2,4)));

            % Z
            uicontrol(panel, 'Style', 'text', ...
                'Units', 'normalized', ...
                'BackgroundColor', bgcol, ...
                'Position', [0.05 1-3*height 0.2 height], ...
                'FontUnits', 'normalized', ...
                'FontSize', 0.9, ...
                'HorizontalAlignment', 'left', ...
                'String', 'z:');

            handles.t6.t(3) = uicontrol(panel, 'Style', 'text', ...
                'Units', 'normalized', ...
                'Position', [0.3 1-3*height 0.6 height], ...
                'FontUnits', 'normalized', ...
                'FontSize', 0.8, ...
                'String', sprintf('%.3f', T6(3,4)));

            % Orientation
            switch opt.orientation
                case 'approach'
                    labels = {'ax:', 'ay:', 'az:'};
                case 'eul'
                    labels = {[char(hex2dec('3c6')) ':'], [char(hex2dec('3b8')) ':'], [char(hex2dec('3c8')) ':']}; % phi theta psi
                case'rpy'
                    labels = {'R:', 'P:', 'Y:'};
            end

            %---- set up the orientation display box

            % AX
            uicontrol(panel, 'Style', 'text', ...
                'Units', 'normalized', ...
                'BackgroundColor', bgcol, ...
                'Position', [0.05 1-5*height 0.2 height], ...
                'FontUnits', 'normalized', ...
                'FontSize', 0.9, ...
                'HorizontalAlignment', 'left', ...
                'String', labels(1));

            handles.t6.r(1) = uicontrol(panel, 'Style', 'text', ...
                'Units', 'normalized', ...
                'Position', [0.3 1-5*height 0.6 height], ...
                'FontUnits', 'normalized', ...
                'FontSize', 0.8, ...
                'String', sprintf('%.3f', T6(1,3)));

            % AY
            uicontrol(panel, 'Style', 'text', ...
                'Units', 'normalized', ...
                'BackgroundColor', bgcol, ...
                'Position', [0.05 1-6*height 0.2 height], ...
                'FontUnits', 'normalized', ...
                'FontSize', 0.9, ...
                'HorizontalAlignment', 'left', ...
                'String', labels(2));

            handles.t6.r(2) = uicontrol(panel, 'Style', 'text', ...
                'Units', 'normalized', ...
                'Position', [0.3 1-6*height 0.6 height], ...
                'FontUnits', 'normalized', ...
                'FontSize', 0.8, ...
                'String', sprintf('%.3f', T6(2,3)));

            % AZ
            uicontrol(panel, 'Style', 'text', ...
                'Units', 'normalized', ...
                'BackgroundColor', bgcol, ...
                'Position', [0.05 1-7*height 0.2 height], ...
                'FontUnits', 'normalized', ...
                'FontSize', 0.9, ...
                'HorizontalAlignment', 'left', ...
                'String', labels(3));

            handles.t6.r(3) = uicontrol(panel, 'Style', 'text', ...
                'Units', 'normalized', ...
                'Position', [0.3 1-7*height 0.6 height], ...
                'FontUnits', 'normalized', ...
                'FontSize', 0.8, ...
                'String', sprintf('%.3f', T6(3,3)));   

            %---- add buttons
            uicontrol(panel, 'Style', 'pushbutton', ...
                'Units', 'normalized', ...
                'Position', [0.80 height*(0)+.01 0.15 height], ...
                'FontUnits', 'normalized', ...
                'FontSize', 0.7, ...
                'CallBack', @(src,event) self.quit_callback(self.model, handles), ...
                'BackgroundColor', 'white', ...
                'ForegroundColor', 'red', ...
                'String', 'X');
            

            
            uicontrol(panel2, 'Style', 'pushbutton', ...
                'Units', 'normalized', ...
                'Position', [0.54 0.6 0.1 0.2], ...
                'FontUnits', 'normalized', ...
                'FontSize', 0.7, ...
                'CallBack', @(src,event) self.X_callback('+', handles), ...
                'BackgroundColor', 'white', ...
                'ForegroundColor', 'red', ...
                'String', 'X+');
            
           tempTarget = handles.robot.fkine(handles.robot.getpos);
           
                
            %EE Y Slider
            handles.targetY = tempTarget(2, 4);
            uicontrol(panel2, 'Style', 'text', ...
                    'Units', 'normalized', ...
                    'BackgroundColor', bgcol, ...
                    'Position', [0.05 0.375 0.06 0.1], ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.5, ...
                    'String','Y');

                % slider itself
            handles.YSlider = uicontrol(panel2, 'Style', 'slider', ...
                    'Units', 'normalized', ...
                    'Position', [0.1 0.4 0.3 0.1], ...
                    'Min', -5, ...
                    'Max', 5, ...
                    'Value', handles.targetY)
                    

             % text box showing slider value, also editable
             handles.YSliderEdit = uicontrol(panel2, 'Style', 'edit', ...
                    'Units', 'normalized', ...
                    'Position', [0.4 0.4 0.05 0.1], ...
                    'BackgroundColor', bgcol, ...
                    'String', num2str(handles.targetY), ...
                    'HorizontalAlignment', 'left', ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.4)
                
            %EE Z Slider
            handles.targetZ = tempTarget(3, 4);
            uicontrol(panel2, 'Style', 'text', ...
                    'Units', 'normalized', ...
                    'BackgroundColor', bgcol, ...
                    'Position', [0.05 0.225 0.06 0.1], ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.5, ...
                    'String','Z');

                % slider itself
            handles.ZSlider = uicontrol(panel2, 'Style', 'slider', ...
                    'Units', 'normalized', ...
                    'Position', [0.1 0.25 0.3 0.1], ...
                    'Min', -5, ...
                    'Max', 5, ...
                    'Value', handles.targetZ)
                    

             % text box showing slider value, also editable
             handles.ZSliderEdit = uicontrol(panel2, 'Style', 'edit', ...
                    'Units', 'normalized', ...
                    'Position', [0.4 0.25 0.05 0.1], ...
                    'BackgroundColor', bgcol, ...
                    'String', num2str(handles.targetZ), ...
                    'HorizontalAlignment', 'left', ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.4)
                
            %Linear Rail Slider
            self.linearRailTargetPose = handles.robot.base(1, 4);
            uicontrol(panel2, 'Style', 'text', ...
                    'Units', 'normalized', ...
                    'BackgroundColor', bgcol, ...
                    'Position', [0.05 0.075 0.06 0.1], ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.5, ...
                    'String','LR');

                % slider itself
            handles.LRSlider = uicontrol(panel2, 'Style', 'slider', ...
                    'Units', 'normalized', ...
                    'Position', [0.1 0.1 0.3 0.1], ...
                    'Min', 0, ...
                    'Max', self.linearRailLength, ...
                    'Value', self.linearRailTargetPose)
                    

             % text box showing slider value, also editable
             handles.LRSliderEdit = uicontrol(panel2, 'Style', 'edit', ...
                    'Units', 'normalized', ...
                    'Position', [0.4 0.1 0.05 0.1], ...
                    'BackgroundColor', bgcol, ...
                    'String', num2str(self.linearRailTargetPose), ...
                    'HorizontalAlignment', 'left', ...
                    'FontUnits', 'normalized', ...
                    'FontSize', 0.4)
                    
                
                


            
            uicontrol(panel2, 'Style', 'pushbutton', ...
                'Units', 'normalized', ...
                'Position', [0.54 0.4 0.1 0.2], ...
                'FontUnits', 'normalized', ...
                'FontSize', 0.7, ...
                'CallBack', @(src,event) self.quit_callback(self.model, handles), ...
                'BackgroundColor', 'white', ...
                'ForegroundColor', 'red', ...
                'String', 'ESTOP');

            % the record button
            handles.record = [];
            if ~isempty(opt.callback)
            uicontrol(panel, 'Style', 'pushbutton', ...
                'Units', 'normalized', ...
                'Position', [0.1 height*(0)+.01 0.30 height], ...
                'FontUnits', 'normalized', ...
                'FontSize', 0.6, ...
                'CallBack', @(src,event) self.record_callback(self.model, handles), ...
                'BackgroundColor', 'red', ...
                'ForegroundColor', 'white', ...
                'String', 'REC');
            end

            %---- now assign the callbacks
            for j=1:n
                % text edit box
                set(handles.edit(j), ...
                    'Interruptible', 'off', ...
                    'Callback', @(src,event)self.advancedTeach_callback(src, self.model.name, j, handles));

                % slider
                set(handles.slider(j), ...
                    'Interruptible', 'off', ...
                    'BusyAction', 'queue', ...
                    'Callback', @(src,event)self.advancedTeach_callback(src, self.model.name, j, handles));
            end
            
                
            set(handles.YSlider, ...
                    'Interruptible', 'off', ...
                    'BusyAction', 'queue', ...
                    'Callback', @(src,event)self.Y_callback(src, self.model.name, handles.targetY, handles));
            set(handles.YSliderEdit, ...
                    'Interruptible', 'off', ...
                    'BusyAction', 'queue', ...
                    'Callback', @(src,event)self.Y_callback(src, self.model.name, handles.targetY, handles));
                
            set(handles.ZSlider, ...
                    'Interruptible', 'off', ...
                    'BusyAction', 'queue', ...
                    'Callback', @(src,event)self.Z_callback(src, self.model.name, handles.targetZ, handles));
            set(handles.ZSliderEdit, ...
                    'Interruptible', 'off', ...
                    'BusyAction', 'queue', ...
                    'Callback', @(src,event)self.Z_callback(src, self.model.name, handles.targetZ, handles));
            
            set(handles.LRSlider, ...
                    'Interruptible', 'off', ...
                    'BusyAction', 'queue', ...
                    'Callback', @(src,event)self.LR_callback(src, self.model.name, self.linearRailTargetPose, handles));
            set(handles.LRSliderEdit, ...
                    'Interruptible', 'off', ...
                    'BusyAction', 'queue', ...
                    'Callback', @(src,event)self.LR_callback(src, self.model.name, self.linearRailTargetPose, handles));
                
        end
        %% 
        function advancedTeach_callback(self, src, name, j, handles)
            % called on changes to a slider or to the edit box showing joint coordinate
            %
            % src      the object that caused the event
            % name     name of the robot
            % j        the joint index concerned (1..N)
            % slider   true if the

            qscale = handles.qscale;

            switch get(src, 'Style')
                case 'slider'
                    % slider changed, get value and reflect it to edit box
                    newval = get(src, 'Value');
                    set(handles.edit(j), 'String', num2str(qscale(j)*newval, 3));
                case 'edit'
                    % edit box changed, get value and reflect it to slider
                    newval = str2double(get(src, 'String')) / qscale(j);
                    set(handles.slider(j), 'Value', newval);
            end
            %fprintf('newval %d %f\n', j, newval);



            % find all graphical objects tagged with the robot name, this is the
            % instancs of that robot across all figures

            h = findobj('Tag', name);


            % find the graphical element of this name
            if isempty(h)
                error('RTB:advancedTeach:badarg', 'No graphical robot of this name found');
            end
            % get the info from its Userdata
            info = get(h(1), 'UserData');

            % update the stored joint coordinates
            info.q(j) = newval;
            % and save it back to the graphical object
            set(h(1), 'UserData', info);

            % update all robots of this name
            animate(handles.robot, info.q);


            % compute the robot tool pose
            T6 = handles.robot.fkine(info.q);

            % convert orientation to desired format
            switch handles.orientation
                case 'approach'
                    orient = T6(:,3);    % approach vector
                case 'eul'
                    orient = tr2eul(T6, 'setopt', handles.opt);
                case'rpy'
                    orient = tr2rpy(T6, 'setopt', handles.opt);
            end

            % update the display in the advancedTeach window
            for i=1:3
                set(handles.t6.t(i), 'String', sprintf('%.3f', T6(i,4)));
                set(handles.t6.r(i), 'String', sprintf('%.3f', orient(i)));
            end

            if ~isempty(handles.callback)
                handles.callback(handles.robot, info.q);
            end

            %notify(handles.robot, 'Moved');

        end
        
        function LR_callback(self, src, name, j, handles)
            switch get(src, 'Style')
                case 'slider'
                    % slider changed, get value and reflect it to edit box
                    newval = get(src, 'Value');
                    set(handles.LRSliderEdit, 'String', num2str(newval));
                case 'edit'
                    % edit box changed, get value and reflect it to slider
                    newval = str2double(get(src, 'String'));
                    set(handles.LRSlider, 'Value', newval);
            end

            self.MoveToTargetLinearRail(newval);
            T6 = handles.robot.fkine(handles.robot.getpos);
            for i=1:3
                set(handles.t6.t(i), 'String', sprintf('%.3f', T6(i,4)));
            end
        end
        function X_callback(self, dir, handles)
            T = self.model.fkine(self.model.getpos)
            if strcmp(dir, '+')
                T(1, 4) = T(1, 4) + 0.001
            elseif strcmp(dir, '-')
                
            end

            q = self.GenerateTargetJointAngles(T);
            self.model.animate(q);
            drawnow();
            T6 = handles.robot.fkine(handles.robot.getpos);
            switch handles.orientation
                case 'approach'
                    orient = T6(:,3);    % approach vector
                case 'eul'
                    orient = tr2eul(T6, 'setopt', handles.opt);
                case'rpy'
                    orient = tr2rpy(T6, 'setopt', handles.opt);
            end

            % update the display in the advancedTeach window
            for i=1:3
                set(handles.t6.t(i), 'String', sprintf('%.3f', T6(i,4)));
                set(handles.t6.r(i), 'String', sprintf('%.3f', orient(i)));
            end
        end
        
        function Y_callback(self, src, name, j, handles)
            switch get(src, 'Style')
                case 'slider'
                    % slider changed, get value and reflect it to edit box
                    newval = get(src, 'Value');
                    set(handles.YSliderEdit, 'String', num2str(newval));
                case 'edit'
                    % edit box changed, get value and reflect it to slider
                    newval = str2double(get(src, 'String'));
                    set(handles.YSlider, 'Value', newval);
            end
            
            T = self.model.fkine(self.model.getpos);
            T(2, 4) = newval;

            q = self.GenerateTargetJointAngles(T);
            self.model.animate(q);
            drawnow();
            T6 = handles.robot.fkine(handles.robot.getpos);
            switch handles.orientation
                case 'approach'
                    orient = T6(:,3);    % approach vector
                case 'eul'
                    orient = tr2eul(T6, 'setopt', handles.opt);
                case'rpy'
                    orient = tr2rpy(T6, 'setopt', handles.opt);
            end

            % update the display in the advancedTeach window
            for i=1:3
                set(handles.t6.t(i), 'String', sprintf('%.3f', T6(i,4)));
                set(handles.t6.r(i), 'String', sprintf('%.3f', orient(i)));
            end
        end
        
        function Z_callback(self, src, name, j, handles)
            switch get(src, 'Style')
                case 'slider'
                    % slider changed, get value and reflect it to edit box
                    newval = get(src, 'Value');
                    set(handles.ZSliderEdit, 'String', num2str(newval));
                case 'edit'
                    % edit box changed, get value and reflect it to slider
                    newval = str2double(get(src, 'String'));
                    set(handles.ZSlider, 'Value', newval);
            end
            
            T = self.model.fkine(self.model.getpos);
            T(3, 4) = newval;

            q = self.GenerateTargetJointAngles(T);
            self.model.animate(q);
            drawnow();
            T6 = handles.robot.fkine(handles.robot.getpos);
            switch handles.orientation
                case 'approach'
                    orient = T6(:,3);    % approach vector
                case 'eul'
                    orient = tr2eul(T6, 'setopt', handles.opt);
                case'rpy'
                    orient = tr2rpy(T6, 'setopt', handles.opt);
            end

            % update the display in the advancedTeach window
            for i=1:3
                set(handles.t6.t(i), 'String', sprintf('%.3f', T6(i,4)));
                set(handles.t6.r(i), 'String', sprintf('%.3f', orient(i)));
            end
        end
        
        

        function record_callback(self, robot, handles)

            if ~isempty(handles.callback)
                handles.callback(h.q);
            end
        end

        function quit_callback(self, robot, handles)
            set(handles.fig, 'ResizeFcn', '');
            delete(handles.panel1);
            delete(handles.panel2);
            set(handles.curax, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1])
        end
        
        function LRFwd_callback(self, robot, handles)
           self.MoveLinearRail(self.teachDistance, "fwd")
           disp("Advanced Teach: Moving Linear Rail Forward");
        end

        function resize_callback(self, robot, handles)

            % come here on figure resize events
            fig = gcbo;   % this figure (whose callback is executing)
            fs = get(fig, 'Position');  % get size of figure
            ps1 = get(handles.panel1, 'Position');  % get position of the panel
            ps2 = get(handles.panel2, 'Position');  % get position of the panel
            
            % keep the panel anchored to the top left corner
            set(handles.panel1, 'Position', [0 0 0.25 * fs(3) 1 * fs(4)]);
            set(handles.panel2, 'Position', [0.25 * fs(3) 0 0.75 * fs(3) 0.25 * fs(4)]);
            
            ps1 = get(handles.panel1, 'Position');  % get position of the panel
            ps2 = get(handles.panel2, 'Position');  % get position of the panel
            
            % update dimensions of the axis area
            set(handles.curax, 'Units', 'pixels', ...
                'OuterPosition', [ps1(3) ps2(4) ps2(3) ps1(4)-ps2(4)]);
        end
        
        %% GetAlgebraicDist
        % determine the algebraic distance given a set of points and the center
        % point and radii of an elipsoid
        % *Inputs:* 
        %
        % _points_ (many*(2||3||6) double) x,y,z cartesian point
        %
        % _centerPoint_ (1 * 3 double) xc,yc,zc of an ellipsoid
        %
        % _radii_ (1 * 3 double) a,b,c of an ellipsoid
        %
        % *Returns:* 
        %
        % _algebraicDist_ (many*1 double) algebraic distance for the ellipsoid

        function algebraicDist = GetAlgebraicDist(points, centrePoint, radii)

        algebraicDist = ((points(:,1)-centrePoint(1))/radii(1)).^2 ...
                      + ((points(:,2)-centrePoint(2))/radii(2)).^2 ...
                      + ((points(:,3)-centrePoint(3))/radii(3)).^2;
        end


        %% CheckEllipsoidCollision

        function collisionCheck = CheckEllipsoidCollision(cubes, jointAngles, robot, radii)
            centrePoint = [0 0 0];
            tr = zeros(4,4,robot.n+1);
            tr(:,:,1) = robot.base;
            L = robot.links;
            for i = 1 : robot.n
                tr(:,:,i+1) = tr(:,:,i) * trotz(jointAngles(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            end

            % Go through each ellipsoid
            for i = 1: size(tr,3)
                for j = 1:size(cubes,3)
                    cubePointsAndOnes = [inv(tr(:,:,i)) * [cubes(:,:,j),ones(size(cubes(:,:,j),1),1)]']';
                    updatedCubePoints = cubePointsAndOnes(:,1:3);
                    %plot3(updatedCubePoints(:,1),updatedCubePoints(:,2),updatedCubePoints(:,3),'b.');
                    radius = radii(i,:);
                    algebraicDist = GetAlgebraicDist(updatedCubePoints, centrePoint, radius);
                    pointsInside = find(algebraicDist < 1);
                    if 0 < pointsInside
                        collisionCheck = 1;
                        break;
                    else
                        collisionCheck= 0;
                    end
                    %disp(['There are ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);
                end
                if 0 < collisionCheck
                    break;
                end       
            end

        end
    end
end

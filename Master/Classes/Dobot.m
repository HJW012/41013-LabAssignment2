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
        
        stop = 0;
        
        collisionDetected = 0;
        
        object;     
        
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
            
            self.object = EnvironmentObject.empty;
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
            
           if (((self.linearRailPose(1,4) - self.linearRail.modelMidPoint(1,1) + self.baseWidth/2) <= x) && (x + self.baseWidth/2 <= (self.linearRailPose(1,4) - self.linearRail.modelMidPoint(1,1) + self.linearRailTravelDist)))
              increment = distanceToTravel/steps;
              for i = 1:steps
                  self.model.base(1, 4) = self.model.base(1,4) + increment;
                  self.model.animate(self.model.getpos);
                  
                   for k = 1:1:size(self.object, 2)
                        % Set the desired object pose to be at the end effector
                        objectPose = self.model.fkine(self.model.getpos());

                        % Update the object's position
                        self.object(k).SetPose(objectPose);
                   end
                  
                  drawnow();
              end
              disp(1);
           end
           
           if (x < (self.linearRailPose(1,4) - self.linearRail.modelMidPoint(1,1) + self.baseWidth/2))
              distance = (self.linearRailPose(1, 4) - self.linearRail.modelMidPoint(1,1) + self.baseWidth/2) - self.model.base(1,4);
              increment = distance/steps;
              
              for i = 1:steps
                  self.model.base(1, 4) = self.model.base(1,4) + increment;
                  self.model.animate(self.model.getpos);
                  
                  for k = 1:1:size(self.object, 2)
                    % Set the desired object pose to be at the end effector
                    objectPose = self.model.fkine(self.model.getpos());

                    % Update the object's position
                    self.object(k).SetPose(objectPose);
                  end
                  
                  drawnow();
              end
              disp(2);
           end
           
           if (x + self.baseWidth/2 > (self.linearRailPose(1,4) - self.linearRail.modelMidPoint(1,1) + self.linearRailTravelDist))
              distance = (self.linearRailPose(1, 4) - self.linearRail.modelMidPoint(1,1) + self.linearRailTravelDist + self.baseWidth/2) - self.model.base(1,4);
              increment = distance/steps;
              
              for i = 1:steps
                  self.model.base(1, 4) = self.model.base(1,4) + increment;
                  self.model.animate(self.model.getpos);
                  
                  
                  for k = 1:1:size(self.object, 2)
                    % Set the desired object pose to be at the end effector
                    objectPose = self.model.fkine(self.model.getpos());

                    % Update the object's position
                    self.object(k).SetPose(objectPose);
                  end
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
            q0 = self.model.getpos;

            %EEPose1 = targetPose
            %q1 = self.GenerateTargetJointAngles2(self.model.base, EEPose1)
            
            q1 = self.model.ikcon(targetPose, q0);
            
            %q1(4) = pi/2 - (q1(2) + q1(3));
            
            steps = 50;
            s = lspb(0,1,steps);
            qMatrix = nan(steps,5);

            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q0 + s(i)*q1;
            end

            for i = 1:steps
               self.model.animate(qMatrix(i, :));
               
               for k = 1:1:size(self.object, 2)
                    % Set the desired object pose to be at the end effector
                    objectPose = self.model.fkine(self.model.getpos());

                    % Update the object's position
                    self.object(k).SetPose(objectPose);
               end
               
               drawnow();
            end
        end
        
        %% Move Arm Joint Angles
        function MoveArmJointAngles(self, targetAngles)
            q0 = self.model.getpos;

            %q1 = self.GenerateTargetJointAngles2(self.model.base, EEPose1)
            
            q1 = targetAngles;
            
            %q1(4) = pi/2 - (q1(2) + q1(3));
            
            steps = 50;
            s = lspb(0,1,steps);
            qMatrix = nan(steps,5);

            for i = 1:steps
                qMatrix(i,:) = (1-s(i))*q0 + s(i)*q1;
            end

            for i = 1:steps
                
               while (self.stop == 1)
                   pause(0.01);
               end
                
               self.model.animate(qMatrix(i, :));
               
               for k = 1:1:size(self.object, 2)
                    % Set the desired object pose to be at the end effector
                    objectPose = self.model.fkine(self.model.getpos());

                    % Update the object's position
                    self.object(k).SetPose(objectPose);
               end
               
               drawnow();
            end
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
                warning off
                self.model.faces{i} = delaunay(self.model.points{i});    
                warning on;
            end

            for j = 1:1:size(qMatrix,1)
                position = qMatrix(j,:);

                check = self.CheckEllipsoidCollision(obstaclePoints, position, radii);

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

        function algebraicDist = GetAlgebraicDist(self, points, centrePoint, radii)

        algebraicDist = ((points(:,1)-centrePoint(1))/radii(1)).^2 ...
                      + ((points(:,2)-centrePoint(2))/radii(2)).^2 ...
                      + ((points(:,3)-centrePoint(3))/radii(3)).^2;
        end


        %% CheckEllipsoidCollision
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
                    disp(['There are ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);
                    break;
                else
                    collisionCheck= 0;
                end

            end
        end
    end
end
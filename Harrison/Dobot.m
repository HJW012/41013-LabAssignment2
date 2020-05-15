classdef Dobot < handle
    properties
        %> Robot model
        model;
        
        defaultJointState = deg2rad([0, 60, 65, -35, 0]);  
        
        %> workspace
        workspace = [-1.5 1.5 -1.5 1.5 0 2.5];   
               
        %> If we have a tool model which will replace the final links model, combined ply file of the tool model and the final link models
        toolModelFilename = []; % Available are: 'DabPrintNozzleTool.ply';        
        toolParametersFilename = []; % Available are: 'DabPrintNozzleToolParameters.mat';
        
        objects; % Objects which will be picked up and collected by the robot
        
        gripperLength = 0.0625; %Length of gripper - cant directly include in DH params
    end
    
    methods%% Class for Dobot robot simulation
        function self = Dobot(robotPose, toolModelAndTCPFilenames)
            if 1 < nargin
                if length(toolModelAndTCPFilenames) ~= 2
                    error('Please pass a cell with two strings, toolModelFilename and toolCenterPointFilename');
                end
                self.toolModelFilename = toolModelAndTCPFilenames{1};
                self.toolParametersFilename = toolModelAndTCPFilenames{2};
            end
            
            % Generate Serial Link
            self.GetDobotRobot();
            
            % Import ply models and create a realistic simulation of the
            % robot
            self.PlotAndColourRobot();
            
            
            % Set the base position of the robot on top of the table
            self.model.base = transl(0, 0, 0);

            % Show the robot
            drawnow();
            
            
            
        end

        %% GetUR3Robot
        % Create and return a UR3 robot model
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
            
            % Initialise the objects being carried by the robot as empty
            %self.objects = Environment.Objects.empty;
            
        end

        %% PlotAndColourRobot
        % Given a robot index, add the glyphs (vertices and faces) and
        % colour them in if data is available 
       
        function PlotAndColourRobot(self)
            % Import ply files and assign to each robot link
            for linkIndex = 0:self.model.n
                [ faceData, vertexData, plyData{linkIndex + 1} ] = plyread(['link',num2str(linkIndex),'.ply'],'tri'); %#ok<AGROW>
                self.model.faces{linkIndex + 1} = faceData;
                self.model.points{linkIndex + 1} = vertexData;
            end

            if ~isempty(self.toolModelFilename)
                [ faceData, vertexData, plyData{self.model.n + 1} ] = plyread(self.toolModelFilename,'tri'); 
                self.model.faces{self.model.n + 1} = faceData;
                self.model.points{self.model.n + 1} = vertexData;
                toolParameters = load(self.toolParametersFilename);
                self.model.tool = toolParameters.tool;
                self.model.qlim = toolParameters.qlim;
                warning('Please check the joint limits. They may be unsafe')
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
            self.model.animate(self.defaultJointState);
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
        %% GoToPose - use to drive end effector to pose
        function possible = GoToPose(pose)
            
        end
    end
end

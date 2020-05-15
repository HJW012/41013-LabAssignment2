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
        
        linearRail; %Attach linear rail for lateral movement
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
        %% Advanced Teach funciton
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
            set(ax, 'OuterPosition', [0.25 0 0.70 1])

            handles.curax = ax;

            % create the panel itself
            panel = uipanel(handles.fig, ...
                'Title', 'advancedTeach', ...
                'BackGroundColor', bgcol,...
                'Position', [0 0 .25 1]);
            set(panel, 'Units', 'pixels'); % stop automatic resizing
            handles.panel = panel;
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
            
            %---- Add BUttonr
            
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

        function record_callback(self, robot, handles)

            if ~isempty(handles.callback)
                handles.callback(h.q);
            end
        end

        function quit_callback(self, robot, handles)
            set(handles.fig, 'ResizeFcn', '');
            delete(handles.panel);
            set(handles.curax, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1])
        end

        function resize_callback(self, robot, handles)

            % come here on figure resize events
            fig = gcbo;   % this figure (whose callback is executing)
            fs = get(fig, 'Position');  % get size of figure
            ps = get(handles.panel, 'Position');  % get position of the panel
            % update dimensions of the axis area
            set(handles.curax, 'Units', 'pixels', ...
                'OuterPosition', [ps(3) 0 fs(3)-ps(3) fs(4)]);
            % keep the panel anchored to the top left corner
            set(handles.panel, 'Position', [1 fs(4)-ps(4) ps(3:4)]);
        end
    end
end

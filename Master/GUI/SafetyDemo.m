function varargout = SafetyDemo(varargin)
% SAFETYDEMO MATLAB code for SafetyDemo.fig
%      SAFETYDEMO, by itself, creates a new SAFETYDEMO or raises the existing
%      singleton*.
%
%      H = SAFETYDEMO returns the handle to a new SAFETYDEMO or the handle to
%      the existing singleton*.
%
%      SAFETYDEMO('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SAFETYDEMO.M with the given input arguments.
%
%      SAFETYDEMO('Property','Value',...) creates a new SAFETYDEMO or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SafetyDemo_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SafetyDemo_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SafetyDemo

% Last Modified by GUIDE v2.5 06-Jun-2020 16:15:05

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SafetyDemo_OpeningFcn, ...
                   'gui_OutputFcn',  @SafetyDemo_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT


% --- Executes just before SafetyDemo is made visible.
function SafetyDemo_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SafetyDemo (see VARARGIN)

% Choose default command line output for SafetyDemo
handles.output = hObject;

% Detect connected joystick - ID of 2 due to vjoy
id = 2;
try
    handles.joy = vrjoystick(id);
end

% Make sure to plot to the axes object within figure
axes(handles.axes_Safety)
hold on;

% Variables
handles.startRetreating = false;
handles.stopSignInserted = false;
handles.linearMultiplier = 0.01;
handles.robot = Dobot();
handles.robot.Display();
EEPose1 = transl(0.23, 0, 0.1) * trotx(pi); % Good starting EE pose
q = handles.robot.model.ikcon(EEPose1, handles.robot.model.getpos);
handles.robot.model.animate(q);

% Global coordinates of stop sign corners (row 1 x, row 2 y, row 3 z)
handles.P = [0.175, 0.25, 0.2, 0.2, 0.25, 0.275;
             0, 0.025, 0.025, -0.025, -0.025, 0;
            -0.1, -0.1, -0.1, -0.1, -0.1, -0.1];
        
% Load stop sign model at average P location
handles.stopSign = EnvironmentObject('Type', 'misc', 'ModelPath', 'stopsign.ply', 'Pose', transl(mean(handles.P(1, :)), mean(handles.P(2, :)), mean(handles.P(3, :))));

% Target points on image plane
handles.pStar = [212 662 362 362 662 812; 512 362 362 662 662 512];

% Establish camera at end effector location
handles.cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
            'resolution', [1024 1024], 'centre', [512 512], 'name', 'Dobot Camera');
        
% Frames per second determining how often image plane updates
handles.fps = 25;

% End Effector Velocity Gain
handles.lambda = 2;

% Depth of each point in camera plane - guess and check to find best value
handles.depth = 0.15;

% Disable controls until robot starts retreating
handles.btn_Retreat.Enable = 'off';
handles.check_Controller.Enable = 'off';

guidata(hObject, handles);

% UIWAIT makes SafetyDemo wait for user response (see UIRESUME)
%uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = SafetyDemo_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in check_Controller.
function check_Controller_Callback(hObject, eventdata, handles)
% hObject    handle to check_Controller (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of check_Controlle
disp('Check box change');
guidata(hObject, handles);
while get(hObject, 'Value') == 1.0 && handles.startRetreating
    handles = guidata(hObject);

    % Read controller inputs
    [axes, buttons, povs] = read(handles.joy);
    xChange = axes(2) * handles.linearMultiplier;
    yChange = axes(1) * handles.linearMultiplier;
    zChange = axes(3) * handles.linearMultiplier;

    % Move feature points and stop sign model
    for i = 1:6
        handles.P(1, i) = handles.P(1, i) + xChange;
        handles.P(2, i) = handles.P(2, i) + yChange;
        handles.P(3, i) = handles.P(3, i) + zChange;
    end

    handles.stopSign.SetPose(handles.stopSign.pose * transl(xChange, yChange, zChange));

    % Plot both points and targets points on image plane
    % Reset image plane every loop to prevent messy image plane
    handles.cam.hold(false);
    handles.cam.plot(handles.pStar, '*');
    handles.cam.hold(true);
    handles.uv = handles.cam.plot(handles.P);

    % Compute error between points and target points - 2 row matrix for
    % each point
    handles.e = handles.pStar-handles.uv;

    % Convert matrix to single column vector
    handles.e = handles.e(:);

    % Depth of points below camera is known, therefore calculate image
    % jacobian using points in cam plane, and known depth
    % - Jacobian returns image-plane point velocity in terms of camera
    % spacial velocity
    % - Image Jacobian also known as feature sensitivity matrix or
    % interaction matrix - not technically a Jacobian
    % - Velocity of each point is a function of its coordinate, its depth,
    % and the cameras internal parameters
    % - Each column of Jacobian indicates velocity of the point with
    % respect to the corresponding component of the velocity vector
    handles.J = handles.cam.visjac_p(handles.uv, handles.depth );
    fig1 = figure(3);
    handles.cam.flowfield([1 1 1 1 1 1]);
    
    % Compute velocity of camera in camera plane using simple linear
    % controller
    % - pStarVel = lambda(pstar - p) OR
    % - cam vel = lambda * inv(image jacobian) * (pstar - p)
    % - pstar - p = error
    % - As long as there are more than 3 points, jacobians can be stacked
    % for all features, camera motion can be solved using pseudo-inverse
    try
        % Pinv can be used to invert a matrix even if it is not square
        handles.v = handles.lambda * pinv(handles.J) * handles.e;
    catch
        return;
    end

    % Get robot jacobian and invert
    % - qdot = inv(J(q)) * v
    % - This maps desired cartesian velocity to joint velocity without
    % requireing inverse kinematics
    handles.J2 = handles.robot.model.jacobn(handles.q0);
    handles.Jinv = pinv(handles.J2);
    handles.qp = handles.Jinv*handles.v;
    % Not handling jacobian singularity here. Could be solved by replacing
    % the inverse with a damped inverse Jacobian
    % - qdot = inv(J(q) + pI) * v
    
    
     %Maximum angular velocity cannot exceed 320 degrees/s - user guide
     %says this is limit
     handles.ind=find(handles.qp>deg2rad(320));
     if ~isempty(handles.ind)
         handles.qp(handles.ind)=deg2rad(320);
     end
     handles.ind=find(handles.qp<-deg2rad(320));
     if ~isempty(handles.ind)
         handles.qp(handles.ind)=-deg2rad(320);
     end

    % Update each joint based on fps and joint velocity
    % - Decreasing fps will result in smaller movements each frame
    handles.q = handles.q0 + (1/handles.fps)*handles.qp;
    handles.robot.model.animate(handles.q');

    % Update camera position to new EE pose
    handles.Tc = handles.robot.model.fkine(handles.q);
    handles.cam.T = handles.Tc;

    drawnow;


     pause(1/handles.fps);

    %update current joint position
    handles.q0 = handles.q;
    
    guidata(hObject, handles);
end

while get(handles.check_Controller, 'Value') == 0 && handles.startRetreating
    % Plot both points and targets points on image plane
    % Reset image plane every loop to prevent messy image plane
    handles.cam.hold(false);
    handles.cam.plot(handles.pStar, '*');
    handles.cam.hold(true);
    handles.uv = handles.cam.plot(handles.P);

    % Compute error between points and target points - 2 row matrix for
    % each point
    handles.e = handles.pStar-handles.uv;

    % Convert matrix to single column vector
    handles.e = handles.e(:);

    % Depth of points below camera is known, therefore calculate image
    % jacobian using points in cam plane, and known depth
    % - Jacobian returns image-plane point velocity in terms of camera
    % spacial velocity
    % - Image Jacobian also known as feature sensitivity matrix or
    % interaction matrix - not technically a Jacobian
    % - Velocity of each point is a function of its coordinate, its depth,
    % and the cameras internal parameters
    % - Each column of Jacobian indicates velocity of the point with
    % respect to the corresponding component of the velocity vector
    handles.J = handles.cam.visjac_p(handles.uv, handles.depth );
    fig2 = figure(3);
    handles.cam.flowfield([1 1 1 0 0 0]);
    
    % Compute velocity of camera in camera plane using simple linear
    % controller
    % - pStarVel = lambda(pstar - p) OR
    % - cam vel = lambda * inv(image jacobian) * (pstar - p)
    % - pstar - p = error
    % - As long as there are more than 3 points, jacobians can be stacked
    % for all features, camera motion can be solved using pseudo-inverse
    try
        % Pinv can be used to invert a matrix even if it is not square
        handles.v = handles.lambda * pinv(handles.J) * handles.e;
    catch
        return;
    end

    % Get robot jacobian and invert
    % - qdot = inv(J(q)) * v
    % - This maps desired cartesian velocity to joint velocity without
    % requireing inverse kinematics
    handles.J2 = handles.robot.model.jacobn(handles.q0);
    handles.Jinv = pinv(handles.J2);
    handles.qp = handles.Jinv*handles.v;
    % Not handling jacobian singularity here. Could be solved by replacing
    % the inverse with a damped inverse Jacobian
    % - qdot = inv(J(q) + pI) * v
    
    
     %Maximum angular velocity cannot exceed 320 degrees/s - user guide
     %says this is limit
     handles.ind=find(handles.qp>deg2rad(320));
     if ~isempty(handles.ind)
         handles.qp(handles.ind)=deg2rad(320);
     end
     handles.ind=find(handles.qp<-deg2rad(320));
     if ~isempty(handles.ind)
         handles.qp(handles.ind)=-deg2rad(320);
     end

    % Update each joint based on fps and joint velocity
    % - Decreasing fps will result in smaller movements each frame
    handles.q = handles.q0 + (1/handles.fps)*handles.qp;
    handles.robot.model.animate(handles.q');

    % Update camera position to new EE pose
    handles.Tc = handles.robot.model.fkine(handles.q);
    handles.cam.T = handles.Tc;

    drawnow;


     pause(1/handles.fps);

    %update current joint position
    handles.q0 = handles.q;
    
    guidata(hObject, handles);
end


guidata(hObject, handles);

% --- Executes on button press in btn_StopSign.
function btn_StopSign_Callback(hObject, eventdata, handles)
% hObject    handle to btn_StopSign (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Inserts Stop Sign model into scene under robot end effector
if (~handles.stopSignInserted)
    % Plot camera at end effector pose
    handles.q0 = handles.robot.model.getpos';
    handles.Tc0 = handles.robot.model.fkine(handles.robot.model.getpos);
    handles.cam.T = handles.Tc0;
    handles.cam.plot_camera('Tcam',handles.Tc0, 'label','scale',0.015);

    % Display stop sign model
    hold on;
    handles.stopSign.Display();

    % Plot stop sign corner points on image plane
    handles.p = handles.cam.plot(handles.P, 'Tcam', handles.Tc0, 'o');
    handles.cam.hold(true);
    
    
    % Plot target points on image plane
    handles.cam.plot(handles.pStar, '*');
    handles.cam.hold(true);

    % Set GUI controls as necessary
    handles.btn_Retreat.Enable = 'on';
    handles.check_Controller.Enable = 'on';
    handles.btn_StopSign.Enable = 'off';

    % Prevent button from inserting stop sign model again
    handles.stopSignInserted = true;
end

guidata(hObject, handles);


% --- Executes on button press in btn_Retreat.
function btn_Retreat_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Retreat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

%Set boolean to keep retreating even when using controller
handles.startRetreating = true;
pause(0.1);

%Update handles before starting loop
guidata(hObject, handles);

% Run this loop as long as check box is not selected
while get(handles.check_Controller, 'Value') == 0 && handles.startRetreating
    % Plot both points and targets points on image plane
    % Reset image plane every loop to prevent messy image plane
    handles.cam.hold(false);
    handles.cam.plot(handles.pStar, '*');
    handles.cam.hold(true);
    handles.uv = handles.cam.plot(handles.P);

    % Compute error between points and target points - 2 row matrix for
    % each point
    handles.e = handles.pStar-handles.uv;

    % Convert matrix to single column vector
    handles.e = handles.e(:);

    % Depth of points below camera is known, therefore calculate image
    % jacobian using points in cam plane, and known depth
    % - Jacobian returns image-plane point velocity in terms of camera
    % spacial velocity
    % - Image Jacobian also known as feature sensitivity matrix or
    % interaction matrix - not technically a Jacobian
    % - Velocity of each point is a function of its coordinate, its depth,
    % and the cameras internal parameters
    % - Each column of Jacobian indicates velocity of the point with
    % respect to the corresponding component of the velocity vector
    handles.J = handles.cam.visjac_p(handles.uv, handles.depth );
    fig2 = figure(3);
    handles.cam.flowfield([1 1 1 0 0 0]);
    
    % Compute velocity of camera in camera plane using simple linear
    % controller
    % - pStarVel = lambda(pstar - p) OR
    % - cam vel = lambda * inv(image jacobian) * (pstar - p)
    % - pstar - p = error
    % - As long as there are more than 3 points, jacobians can be stacked
    % for all features, camera motion can be solved using pseudo-inverse
    try
        % Pinv can be used to invert a matrix even if it is not square
        handles.v = handles.lambda * pinv(handles.J) * handles.e;
    catch
        return;
    end

    % Get robot jacobian and invert
    % - qdot = inv(J(q)) * v
    % - This maps desired cartesian velocity to joint velocity without
    % requireing inverse kinematics
    handles.J2 = handles.robot.model.jacobn(handles.q0);
    handles.Jinv = pinv(handles.J2);
    handles.qp = handles.Jinv*handles.v;
    % Not handling jacobian singularity here. Could be solved by replacing
    % the inverse with a damped inverse Jacobian
    % - qdot = inv(J(q) + pI) * v
    
    
     %Maximum angular velocity cannot exceed 320 degrees/s - user guide
     %says this is limit
     handles.ind=find(handles.qp>deg2rad(320));
     if ~isempty(handles.ind)
         handles.qp(handles.ind)=deg2rad(320);
     end
     handles.ind=find(handles.qp<-deg2rad(320));
     if ~isempty(handles.ind)
         handles.qp(handles.ind)=-deg2rad(320);
     end

    % Update each joint based on fps and joint velocity
    % - Decreasing fps will result in smaller movements each frame
    handles.q = handles.q0 + (1/handles.fps)*handles.qp;
    handles.robot.model.animate(handles.q');

    % Update camera position to new EE pose
    handles.Tc = handles.robot.model.fkine(handles.q);
    handles.cam.T = handles.Tc;

    drawnow;


     pause(1/handles.fps);

    %update current joint position
    handles.q0 = handles.q;
    
    guidata(hObject, handles);
end
    

guidata(hObject, handles);


% --- Executes on button press in btn_StopRetreat.
function btn_StopRetreat_Callback(hObject, eventdata, handles)
% hObject    handle to btn_StopRetreat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)S
guidata(hObject, handles);


% --- Executes on button press in btn_Exit.
function btn_Exit_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Exit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
guidata(hObject,handles);
figure1_CloseRequestFcn(handles.figure1, eventdata, handles);


% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: delete(hObject) closes the figure

if isequal(get(handles.figure1, 'waitstatus'), 'waiting')
    uiresume(hObject);
    %close all;
else
    delete(hObject);
end

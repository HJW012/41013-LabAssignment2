function varargout = Simulation(varargin)
% SIMULATION MATLAB code for Simulation.fig
%      SIMULATION, by itself, creates a new SIMULATION or raises the existing
%      singleton*.
%
%      H = SIMULATION returns the handle to a new SIMULATION or the handle to
%      the existing singleton*.
%
%      SIMULATION('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SIMULATION.M with the given input arguments.
%
%      SIMULATION('Property','Value',...) creates a new SIMULATION or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Simulation_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Simulation_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Simulation

% Last Modified by GUIDE v2.5 06-Jun-2020 18:51:22

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @Simulation_OpeningFcn, ...
                   'gui_OutputFcn',  @Simulation_OutputFcn, ...
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


% --- Executes just before Simulation is made visible.
function Simulation_OpeningFcn(hObject, eventdata, handles, varargin)

% Choose default command line output for Simulation
handles.output = hObject;

% Set up simulation

%{
handles.table = EnvironmentObject('Type', 'foundation', 'ModelPath', 'table.ply', 'Pose', transl(0, 0, 0), 'Dimensions', [2.1956 1.0097 0.8911]);
handles.blueCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'blueCrate.ply', 'Pose', transl(0.75, 0.2, 0.8911), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'b');
handles.yellowCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'yellowCrate.ply', 'Pose', transl(0.75, 0, 0.8911), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'g');
handles.redCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'redCrate.ply', 'Pose', transl(0.75, -0.2, 0.8911), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'r');
handles.cone1 = EnvironmentObject('Type', 'safety', 'ModelPath', 'cone.ply', 'Pose', transl(1.25, -0.75, 0));
handles.cone2 = EnvironmentObject('Type', 'safety', 'ModelPath', 'cone.ply', 'Pose', transl(-1.25, -0.75, 0));
handles.cone3 = EnvironmentObject('Type', 'safety', 'ModelPath', 'cone.ply', 'Pose', transl(1.25, 0.75, 0));
handles.cone4 = EnvironmentObject('Type', 'safety', 'ModelPath', 'cone.ply', 'Pose', transl(-1.25, 0.75, 0));
handles.estop = EnvironmentObject('Type', 'safety', 'ModelPath', 'EStop.ply', 'Pose', transl(0, 0.8, 0));
hold on;
%{
handles.cone1.Display();
handles.cone2.Display();
handles.cone3.Display();
handles.cone4.Display();
handles.estop.Display();
%}
handles.environment = Environment(handles.table, handles.blueCrate, handles.yellowCrate, handles.redCrate, handles.cone1, handles.cone2, handles.cone3, handles.cone4, handles.estop);
handles.robot = Dobot(); %No basepose given as the linear rail will automatically update its position
handles.robot.GenerateLinearRail([-0.45, 0, 0.8911]);
handles.environment.AddRobot(handles.robot);
handles.camera = RGBCamera('CentrePose', transl(0, 0, 2.5) * troty(pi));
handles.plc = GlobalController(handles.environment, handles.camera);
%}

% Set Up Simulation
handles.plc = GlobalController();
handles.plc.Setup();

% Initialise controls on GUI
handles.EStopPress = false;
handles.txt_TargetZ.String = sprintf("%.3f", handles.plc.environment.foundation.dimensions(3));
handles.btn_InsertObstacle.Enable = 'off';
handles.btn_RemoveObstacle.Enable = 'off';
handles.btn_InsertObstruction.Enable = 'off';
handles.btn_RemoveObstruction.Enable = 'off';
handles.btn_RemoveObstruction.Enable = 'off';
handles.btn_EStop.Enable = 'off';
axes(handles.axes_Simulation);  %Set axes in GUI as current axes to plot all simulation objects
hold on;



%handles.plc.environment.Display();
disp('Initialisation Complete');


guidata(hObject, handles);

% UIWAIT makes Simulation wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Simulation_OutputFcn(hObject, eventdata, handles) 

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in btn_EStop.
function btn_EStop_Callback(hObject, eventdata, handles)
handles.EStopPress = ~handles.EStopPress;
if ~handles.EStopPress
    handles.InsertObstacle.Enable = 'on';
    handles.RemoveObstacle.Enable = 'on';
    handles.InsertObstruction.Enable = 'on';
    handles.RemoveObstruction.Enable = 'on';
    guidata(hObject, handles);
    handles.plc.emergencyStop = 0;
else
    handles.InsertObstacle.Enable = 'off';
    handles.RemoveObstacle.Enable = 'off';
    handles.InsertObstruction.Enable = 'off';
    handles.RemoveObstruction.Enable = 'off';
    guidata(hObject, handles);
    handles.plc.emergencyStop = 1;
end

% Update GUI handles variables - MUST include this otherwise handles wont
% be updated in other functions
guidata(hObject, handles);


% --- Executes on button press in btn_InsertObstruction.
function btn_InsertObstruction_Callback(hObject, eventdata, handles)
%{
handles.plc.hand = EnvironmentObject('Type', 'misc', 'ModelPath', 'hand.ply', 'Pose', transl(-0.2, 0.45, handles.plc.environment.foundation.dimensions(1,3)+0.1), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'y');
handles.plc.hand.SetPose(handles.plc.hand.pose * trotz(-pi/2));
handles.plc.hand.Display();

handles.plc.environment.AddObject(handles.plc.hand);  
%}
handles.plc.InsertObstruction();
guidata(hObject, handles);



% --- Executes on button press in btn_RemoveObstruction.
function btn_RemoveObstruction_Callback(hObject, eventdata, handles)
handles.plc.RemoveObstruction();
guidata(hObject, handles);



function txt_ObstacleX_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function txt_ObstacleX_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


function txt_ObstacleY_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function txt_ObstacleY_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_ObstacleZ_Callback(hObject, eventdata, handles)



% --- Executes during object creation, after setting all properties.
function txt_ObstacleZ_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_RemoveObstacle.
function btn_RemoveObstacle_Callback(hObject, eventdata, handles)
%handles.plc.obstacle.Remove();
%handles.plc.environment.obstacleObjects{1}.pose = handles.plc.obstacle.pose;
handles.plc.RemoveObstacle();
guidata(hObject, handles);

% --- Executes on button press in btn_InsertObstacle.
function btn_InsertObstacle_Callback(hObject, eventdata, handles)
obstacleX = str2double(handles.txt_ObstacleX.String);
obstacleY = str2double(handles.txt_ObstacleY.String);
obstacleZ = str2double(handles.txt_ObstacleZ.String);
%handles.plc.obstacle = EnvironmentObject('Type', 'obstacle', 'ModelPath', 'obstacleBall.ply', 'Pose', transl(obstacleX, obstacleY, obstacleZ), 'Dimensions', [0.1 0.1 0.1], 'GeneralColour', 'y');
%handles.plc.obstacle.Display();
%handles.plc.environment.AddObject(handles.plc.obstacle);

obstaclePose = transl(obstacleX, obstacleY, obstacleZ);
handles.plc.InsertObstacle(obstaclePose);
guidata(hObject, handles);


function txt_TargetX_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function txt_TargetX_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_TargetY_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function txt_TargetY_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_TargetZ_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function txt_TargetZ_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_TargetRot_Callback(hObject, eventdata, handles)


% --- Executes during object creation, after setting all properties.
function txt_TargetRot_CreateFcn(hObject, eventdata, handles)
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_InsertTarget.
function btn_InsertTarget_Callback(hObject, eventdata, handles)
hold on;
targetX = str2double(get(handles.txt_TargetX, 'String'))
targetY = str2double(get(handles.txt_TargetY, 'String'))
targetRot = str2double(get(handles.txt_TargetRot, 'String'))
targetPose = transl(targetX, targetY, handles.plc.environment.foundation.dimensions(3)) * trotz(deg2rad(targetRot));
if get(handles.rad_TargetRed, 'Value') == 1
    disp('Red Pen');
    %tempObject = EnvironmentObject('Type', 'target', 'ModelPath', 'redPen.ply', 'Pose', transl(targetX, targetY, handles.plc.environment.foundation.dimensions(3)) * trotz(deg2rad(targetRot)), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'r');
    %tempObject.Display();
    %tempObject.SetPose(tempObject.pose * trotz(deg2rad(targetRot)));
    %handles.plc.environment.AddObject(tempObject);
    handles.plc.InsertTarget(targetPose, 'r');

elseif get(handles.rad_TargetBlue, 'Value') == 1
    disp('Blue Pen');
    %tempObject = EnvironmentObject('Type', 'target', 'ModelPath', 'bluePen.ply', 'Pose', transl(targetX, targetY, handles.plc.environment.foundation.dimensions(3)), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'b');
    %tempObject.Display();
    %tempObject.SetPose(tempObject.pose * trotz(deg2rad(targetRot)));
    %handles.plc.environment.AddObject(tempObject);
    handles.plc.InsertTarget(targetPose, 'b');

else
    disp('Pencil');
    %tempObject = EnvironmentObject('Type', 'target', 'ModelPath', 'pencil.ply', 'Pose', transl(targetX, targetY, handles.plc.environment.foundation.dimensions(3)), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'g');
    %tempObject.Display();
    %tempObject.SetPose(tempObject.pose * trotz(deg2rad(targetRot)));
    %handles.plc.environment.AddObject(tempObject);
    handles.plc.InsertTarget(targetPose, 'g');

end
guidata(hObject, handles);

% --- Executes on button press in btn_StartSim.
function btn_StartSim_Callback(hObject, eventdata, handles)
handles.btn_InsertObstacle.Enable = 'on';
handles.btn_RemoveObstacle.Enable = 'on';
handles.btn_InsertObstruction.Enable = 'on';
handles.btn_RemoveObstruction.Enable = 'on';
handles.btn_RemoveObstruction.Enable = 'on';
handles.btn_InsertTarget.Enable = 'off';
handles.btn_EStop.Enable = 'on';
handles.btn_StartSim.Enable = 'off';

handles.plc.Init();
handles.plc.Run();


guidata(hObject, handles);



% --- Executes when user attempts to close figure1.
function figure1_CloseRequestFcn(hObject, eventdata, handles)
% Hint: delete(hObject) closes the figure
if isequal(get(handles.figure1, 'waitstatus'), 'waiting')
    uiresume(hObject);
    %close all;
else
    delete(hObject);
end


% --- Executes on button press in btn_Exit.
function btn_Exit_Callback(hObject, eventdata, handles)
guidata(hObject,handles);
figure1_CloseRequestFcn(handles.figure1, eventdata, handles);

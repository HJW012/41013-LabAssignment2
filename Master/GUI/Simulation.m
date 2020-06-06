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
% End initialization code - DO NOT EDIT


% --- Executes just before Simulation is made visible.
function Simulation_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Simulation (see VARARGIN)

% Choose default command line output for Simulation
handles.output = hObject;

% Set up simulation
try
    stop(handles.t);
    delete(handles.t);
end

handles.EStopPress = false;
handles.table = EnvironmentObject('Type', 'foundation', 'ModelPath', 'table.ply', 'Pose', transl(0, 0, 0), 'Dimensions', [2.1956 1.0097 0.8911]);
handles.blueCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'blueCrate.ply', 'Pose', transl(0.75, 0.2, 0.8911), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'b');
handles.yellowCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'yellowCrate.ply', 'Pose', transl(0.75, 0, 0.8911), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'g');
handles.redCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'redCrate.ply', 'Pose', transl(0.75, -0.2, 0.8911), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'r');
handles.cone1 = EnvironmentObject('Type', 'misc', 'ModelPath', 'cone.ply', 'Pose', transl(1.25, -0.75, 0));
handles.cone2 = EnvironmentObject('Type', 'misc', 'ModelPath', 'cone.ply', 'Pose', transl(-1.25, -0.75, 0));
handles.cone3 = EnvironmentObject('Type', 'misc', 'ModelPath', 'cone.ply', 'Pose', transl(1.25, 0.75, 0));
handles.cone4 = EnvironmentObject('Type', 'misc', 'ModelPath', 'cone.ply', 'Pose', transl(-1.25, 0.75, 0));
handles.estop = EnvironmentObject('Type', 'misc', 'ModelPath', 'EStop.ply', 'Pose', transl(0, 0.8, 0));
hold on;
handles.cone1.Display();
handles.cone2.Display();
handles.cone3.Display();
handles.cone4.Display();
handles.estop.Display();

handles.environment = Environment(handles.table, handles.blueCrate, handles.yellowCrate, handles.redCrate, handles.cone1);
handles.robot = Dobot(); %No basepose given as the linear rail will automatically update its position
handles.robot.GenerateLinearRail([-0.45, 0, 0.8911]);
handles.environment.AddRobot(handles.robot);
handles.camera = RGBCamera('CentrePose', transl(0, 0, 2.5) * troty(pi));
handles.txt_TargetZ.String = sprintf("%.3f", handles.table.dimensions(3));
handles.plc = GlobalController(handles.environment, handles.camera);
handles.btn_InsertObstacle.Enable = 'off';
handles.btn_RemoveObstacle.Enable = 'off';
handles.btn_InsertObstruction.Enable = 'off';
handles.btn_RemoveObstruction.Enable = 'off';
handles.btn_RemoveObstruction.Enable = 'off';
handles.btn_EStop.Enable = 'off';

% Initialise the simulation
axes(handles.axes_Simulation); %Set axes in GUI as current axes to plot all simulation objects
hold on;



handles.plc.environment.Display();
disp('Initialisation Complete');


guidata(hObject, handles);

% UIWAIT makes Simulation wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = Simulation_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in btn_EStop.
function btn_EStop_Callback(hObject, eventdata, handles)
% hObject    handle to btn_EStop (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
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


guidata(hObject, handles);


% --- Executes on button press in btn_InsertObstruction.
function btn_InsertObstruction_Callback(hObject, eventdata, handles)
% hObject    handle to btn_InsertObstruction (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.plc.hand = EnvironmentObject('Type', 'misc', 'ModelPath', 'hand.ply', 'Pose', transl(-0.2, 0.45, handles.plc.environment.foundation.dimensions(1,3)+0.1), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'y');
handles.plc.hand.SetPose(handles.plc.hand.pose * trotz(-pi/2));
handles.plc.hand.Display();

handles.plc.environment.AddObject(handles.plc.hand);  

guidata(hObject, handles);



% --- Executes on button press in btn_RemoveObstruction.
function btn_RemoveObstruction_Callback(hObject, eventdata, handles)
% hObject    handle to btn_RemoveObstruction (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.plc.hand.Remove();
guidata(hObject, handles);



function txt_ObstacleX_Callback(hObject, eventdata, handles)
% hObject    handle to txt_ObstacleX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_ObstacleX as text
%        str2double(get(hObject,'String')) returns contents of txt_ObstacleX as a double


% --- Executes during object creation, after setting all properties.
function txt_ObstacleX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_ObstacleX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_ObstacleY_Callback(hObject, eventdata, handles)
% hObject    handle to txt_ObstacleY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_ObstacleY as text
%        str2double(get(hObject,'String')) returns contents of txt_ObstacleY as a double


% --- Executes during object creation, after setting all properties.
function txt_ObstacleY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_ObstacleY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_ObstacleZ_Callback(hObject, eventdata, handles)
% hObject    handle to txt_ObstacleZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_ObstacleZ as text
%        str2double(get(hObject,'String')) returns contents of txt_ObstacleZ as a double


% --- Executes during object creation, after setting all properties.
function txt_ObstacleZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_ObstacleZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_RemoveObstacle.
function btn_RemoveObstacle_Callback(hObject, eventdata, handles)
% hObject    handle to btn_RemoveObstacle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.plc.obstacle.Remove();
handles.plc.environment.obstacleObjects{1}.pose = handles.plc.obstacle.pose;
guidata(hObject, handles);

% --- Executes on button press in btn_InsertObstacle.
function btn_InsertObstacle_Callback(hObject, eventdata, handles)
% hObject    handle to btn_InsertObstacle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
obstacleX = str2double(handles.txt_ObstacleX.String);
obstacleY = str2double(handles.txt_ObstacleY.String);
obstacleZ = str2double(handles.txt_ObstacleZ.String);
handles.plc.obstacle = EnvironmentObject('Type', 'obstacle', 'ModelPath', 'obstacleBall.ply', 'Pose', transl(obstacleX, obstacleY, obstacleZ), 'Dimensions', [0.1 0.1 0.1], 'GeneralColour', 'y');
handles.plc.obstacle.Display();
handles.plc.environment.AddObject(handles.plc.obstacle);
guidata(hObject, handles);


function txt_TargetX_Callback(hObject, eventdata, handles)
% hObject    handle to txt_TargetX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_TargetX as text
%        str2double(get(hObject,'String')) returns contents of txt_TargetX as a double


% --- Executes during object creation, after setting all properties.
function txt_TargetX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_TargetX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_TargetY_Callback(hObject, eventdata, handles)
% hObject    handle to txt_TargetY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_TargetY as text
%        str2double(get(hObject,'String')) returns contents of txt_TargetY as a double


% --- Executes during object creation, after setting all properties.
function txt_TargetY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_TargetY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_TargetZ_Callback(hObject, eventdata, handles)
% hObject    handle to txt_TargetZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_TargetZ as text
%        str2double(get(hObject,'String')) returns contents of txt_TargetZ as a double


% --- Executes during object creation, after setting all properties.
function txt_TargetZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_TargetZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_TargetRot_Callback(hObject, eventdata, handles)
% hObject    handle to txt_TargetRot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_TargetRot as text
%        str2double(get(hObject,'String')) returns contents of txt_TargetRot as a double


% --- Executes during object creation, after setting all properties.
function txt_TargetRot_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_TargetRot (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_InsertTarget.
function btn_InsertTarget_Callback(hObject, eventdata, handles)
hold on;
targetX = str2double(get(handles.txt_TargetX, 'String'))
targetY = str2double(get(handles.txt_TargetY, 'String'))
targetRot = str2double(get(handles.txt_TargetRot, 'String'))

if get(handles.rad_TargetRed, 'Value') == 1
    disp('Red Pen');
    tempObject = EnvironmentObject('Type', 'target', 'ModelPath', 'redPen.ply', 'Pose', transl(targetX, targetY, handles.plc.environment.foundation.dimensions(3)), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'r');
    tempObject.Display();
    tempObject.SetPose(tempObject.pose * trotz(deg2rad(targetRot)));
    handles.plc.environment.AddObject(tempObject);
elseif get(handles.rad_TargetBlue, 'Value') == 1
    disp('Blue Pen');
    tempObject = EnvironmentObject('Type', 'target', 'ModelPath', 'bluePen.ply', 'Pose', transl(targetX, targetY, handles.plc.environment.foundation.dimensions(3)), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'b');
    tempObject.Display();
    tempObject.SetPose(tempObject.pose * trotz(deg2rad(targetRot)));
    handles.plc.environment.AddObject(tempObject);
else
    disp('Pencil');
    tempObject = EnvironmentObject('Type', 'target', 'ModelPath', 'pencil.ply', 'Pose', transl(targetX, targetY, handles.plc.environment.foundation.dimensions(3)), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'g');
    tempObject.Display();
    tempObject.SetPose(tempObject.pose * trotz(deg2rad(targetRot)));
    handles.plc.environment.AddObject(tempObject);
end
guidata(hObject, handles);


% --- Executes on button press in btn_ResetSim.
function btn_ResetSim_Callback(hObject, eventdata, handles)
% hObject    handle to btn_ResetSim (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA
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


% --- Executes on button press in btn_StopSim.
function btn_StopSim_Callback(hObject, eventdata, handles)
% hObject    handle to btn_StopSim (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

function timer_callback(~, ~, hFigure)
handles = guidata(hFigure);
set(0, 'currentfigure', hFigure);
handles.environment.robot.model.getpos
%{
orient = tr2rpy(EEPose, 'deg');
disp('test1');
set(handles.txt_EEPX, 'String', sprintf('%.3f', EEPose(1, 4)));
disp('test 2');
set(handles.txt_EEPY, 'String', sprintf('%.3f', EEPose(2, 4)));
set(handles.txt_EEPZ, 'String', sprintf('%.3f', EEPose(3, 4)));
set(handles.txt_EERR, 'String', sprintf('%.3f', orient(1)));
set(handles.txt_EERP, 'String', sprintf('%.3f', orient(2)));
set(handles.txt_EERY, 'String', sprintf('%.3f', orient(3)));
%}



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


% --- Executes on button press in btn_Exit.
function btn_Exit_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Exit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
guidata(hObject,handles);
figure1_CloseRequestFcn(handles.figure1, eventdata, handles);

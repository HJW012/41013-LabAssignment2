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

% Last Modified by GUIDE v2.5 03-Jun-2020 00:32:27

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

% Update handles structure
handles.blueCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'blueCrate.ply', 'Pose', transl(0.75, 0.2, 0.8911), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'b');
handles.yellowCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'yellowCrate.ply', 'Pose', transl(0.75, 0, 0.8911), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'y');
handles.redCrate = EnvironmentObject('Type', 'deposit', 'ModelPath', 'redCrate.ply', 'Pose', transl(0.75, -0.2, 0.8911), 'Dimensions', [0.24 0.16 0.0664], 'GeneralColour', 'r');
handles.redPen = EnvironmentObject('Type', 'target', 'ModelPath', 'redPen.ply', 'Pose', transl(0.45, 0.25, 0.8911), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'r');
handles.bluePen = EnvironmentObject('Type', 'target', 'ModelPath', 'bluePen.ply', 'Pose', transl(0, -0.2, 0.8911), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'b');
handles.pencil1 = EnvironmentObject('Type', 'target', 'ModelPath', 'pencil.ply', 'Pose', transl(0.15, 0.2, 0.8911), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'y');
handles.pencil2 = EnvironmentObject('Type', 'target', 'ModelPath', 'pencil.ply', 'Pose', transl(-0.15, -0.25, 0.8911), 'Dimensions', [0.1734 0.0123 0.0124], 'GeneralColour', 'y');
handles.table = EnvironmentObject('Type', 'foundation', 'ModelPath', 'table.ply', 'Pose', transl(0, 0, 0), 'Dimensions', [2.1956 1.0097 0.8911], 'GeneralColour', 'r');
handles.environment = Environment(handles.blueCrate, handles.yellowCrate, handles.redCrate, handles.redPen, handles.bluePen, handles.pencil1, handles.pencil2, handles.table);
handles.Dobot1 = Dobot('BasePose', eye(4)*transl(0,0,handles.table.dimensions(1,3)));
handles.Dobot1.GenerateLinearRail([-0.45,0,0.8911]);
handles.environment.AddRobot(handles.Dobot1);
handles.camera = RGBCamera('CentrePose', transl(0, 0, 2.5) * troty(pi));

% Create the Master Controller and tell it about the environment
handles.plc = GlobalController(handles.environment, handles.camera);

% Initialise the simulation
hold on;
handles.plc.Init();
disp('Initialisation Complete');
%axes(handles.axes2);
%hold on;
%handles.Dobot1.Display();
%axis equal;
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


% --- Executes on button press in btn_InsertObstruction.
function btn_InsertObstruction_Callback(hObject, eventdata, handles)
% hObject    handle to btn_InsertObstruction (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in btn_RemoveObstruction.
function btn_RemoveObstruction_Callback(hObject, eventdata, handles)
% hObject    handle to btn_RemoveObstruction (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



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


% --- Executes on button press in btn_InsertObstacle.
function btn_InsertObstacle_Callback(hObject, eventdata, handles)
% hObject    handle to btn_InsertObstacle (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)



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
% hObject    handle to btn_InsertTarget (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in btn_ResetSim.
function btn_ResetSim_Callback(hObject, eventdata, handles)
% hObject    handle to btn_ResetSim (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in btn_StartSim.
function btn_StartSim_Callback(hObject, eventdata, handles)
% hObject    handle to btn_StartSim (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.plc.Run();


% --- Executes on button press in btn_StopSim.
function btn_StopSim_Callback(hObject, eventdata, handles)
% hObject    handle to btn_StopSim (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

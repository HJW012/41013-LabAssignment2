function varargout = AdvancedTeach(varargin)
% ADVANCEDTEACH MATLAB code for AdvancedTeach.fig
%      ADVANCEDTEACH, by itself, creates a new ADVANCEDTEACH or raises the existing
%      singleton*.
%
%      H = ADVANCEDTEACH returns the handle to a new ADVANCEDTEACH or the handle to
%      the existing singleton*.
%
%      ADVANCEDTEACH('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in ADVANCEDTEACH.M with the given input arguments.
%
%      ADVANCEDTEACH('Property','Value',...) creates a new ADVANCEDTEACH or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before AdvancedTeach_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to AdvancedTeach_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help AdvancedTeach

% Last Modified by GUIDE v2.5 04-Jun-2020 23:18:48

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @AdvancedTeach_OpeningFcn, ...
                   'gui_OutputFcn',  @AdvancedTeach_OutputFcn, ...
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


% --- Executes just before AdvancedTeach is made visible.
function AdvancedTeach_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to AdvancedTeach (see VARARGIN)

% Choose default command line output for AdvancedTeach
handles.output = hObject;



id = 2; % Note: may need to be changed if multiple joysticks present
handles.joy = vrjoystick(id);

handles.robot = Dobot('BasePose', transl(0, 0, 0));
handles.robot.GenerateLinearRail([0,0,0]);
hold on;
handles.robot.Display;
handles.teachDist = 0.002;
set(handles.slider_q1, 'min', rad2deg(handles.robot.model.qlim(1, 1)));
set(handles.slider_q1, 'max', rad2deg(handles.robot.model.qlim(1, 2)));
set(handles.slider_q2, 'min', rad2deg(handles.robot.model.qlim(2, 1)));
set(handles.slider_q2, 'max', rad2deg(handles.robot.model.qlim(2, 2)));
set(handles.slider_q3, 'min', rad2deg(handles.robot.model.qlim(3, 1)));
set(handles.slider_q3, 'max', rad2deg(handles.robot.model.qlim(3, 2)));
set(handles.slider_q4, 'min', rad2deg(handles.robot.model.qlim(4, 1)));
set(handles.slider_q4, 'max', rad2deg(handles.robot.model.qlim(4, 2)));
set(handles.slider_q5, 'min', rad2deg(handles.robot.model.qlim(5, 1)));
set(handles.slider_q5, 'max', rad2deg(handles.robot.model.qlim(5, 2)));
set(handles.slider_LR, 'min', 0);
set(handles.slider_LR, 'max', handles.robot.linearRailTravelDist);

localQ0 = rad2deg(handles.robot.model.getpos);
EEPose0 = handles.robot.model.fkine(deg2rad(localQ0));
orient = tr2rpy(EEPose0, 'deg');
set(handles.txt_EEPX, 'String', sprintf("%.3f", EEPose0(1, 4)));
set(handles.txt_EEPY, 'String', sprintf("%.3f", EEPose0(2, 4)));
set(handles.txt_EEPZ, 'String', sprintf("%.3f", EEPose0(3, 4)));
set(handles.txt_EERR, 'String', sprintf("%.3f", orient(1)));
set(handles.txt_EERP, 'String', sprintf("%.3f", orient(2)));
set(handles.txt_EERY, 'String', sprintf("%.3f", orient(3)));
set(handles.slider_q1, 'Value', localQ0(1));
set(handles.txt_q1, 'String', sprintf("%.3f", localQ0(1)));
set(handles.slider_q2, 'Value', localQ0(2));
set(handles.txt_q2, 'String', sprintf("%.3f", localQ0(2)));
set(handles.slider_q3, 'Value', localQ0(3));
set(handles.txt_q3, 'String', sprintf("%.3f", localQ0(3)));
set(handles.slider_q4, 'Value', localQ0(4));
set(handles.txt_q4, 'String', sprintf("%.3f", localQ0(4)));
set(handles.slider_q5, 'Value', localQ0(5));
set(handles.txt_q5, 'String', sprintf("%.3f", localQ0(5)));
handles.startingBase = handles.robot.model.base;

%UpdateFields(hObject, eventdata, handles);
% Update handles structure
guidata(hObject, handles);

% UIWAIT makes AdvancedTeach wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = AdvancedTeach_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on slider movement.
function slider_LR_Callback(hObject, eventdata, handles)
% hObject    handle to slider_LR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
localQ0 = handles.robot.model.getpos;
LRValue = handles.slider_LR.Value;
handles.robot.model.base = handles.startingBase * transl(LRValue, 0, 0);

handles.robot.model.animate(localQ0);

localQ0 = rad2deg(handles.robot.model.getpos);
EEPose0 = handles.robot.model.fkine(deg2rad(localQ0));
orient = tr2rpy(EEPose0, 'deg');
set(handles.txt_EEPX, 'String', sprintf("%.3f", EEPose0(1, 4)));
set(handles.txt_EEPY, 'String', sprintf("%.3f", EEPose0(2, 4)));
set(handles.txt_EEPZ, 'String', sprintf("%.3f", EEPose0(3, 4)));
set(handles.txt_EERR, 'String', sprintf("%.3f", orient(1)));
set(handles.txt_EERP, 'String', sprintf("%.3f", orient(2)));
set(handles.txt_EERY, 'String', sprintf("%.3f", orient(3)));
set(handles.slider_q1, 'Value', localQ0(1));
set(handles.txt_q1, 'String', sprintf("%.3f", localQ0(1)));
set(handles.slider_q2, 'Value', localQ0(2));
set(handles.txt_q2, 'String', sprintf("%.3f", localQ0(2)));
set(handles.slider_q3, 'Value', localQ0(3));
set(handles.txt_q3, 'String', sprintf("%.3f", localQ0(3)));
set(handles.slider_q4, 'Value', localQ0(4));
set(handles.txt_q4, 'String', sprintf("%.3f", localQ0(4)));
set(handles.slider_q5, 'Value', localQ0(5));
set(handles.txt_q5, 'String', sprintf("%.3f", localQ0(5)));
set(handles.txt_LRX, 'String', sprintf("%.3f", LRValue));

guidata(hObject, handles);



% --- Executes during object creation, after setting all properties.
function slider_LR_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_LR (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function txt_LRX_Callback(hObject, eventdata, handles)
% hObject    handle to txt_LRX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_LRX as text
%        str2double(get(hObject,'String')) returns contents of txt_LRX as a double
localQ0 = handles.robot.model.getpos;
LRValue = str2double(handles.txt_LRX.String);
handles.robot.model.base = handles.startingBase * transl(LRValue, 0, 0);

handles.robot.model.animate(localQ0);

localQ0 = rad2deg(handles.robot.model.getpos);
EEPose0 = handles.robot.model.fkine(deg2rad(localQ0));
orient = tr2rpy(EEPose0, 'deg');
set(handles.txt_EEPX, 'String', sprintf("%.3f", EEPose0(1, 4)));
set(handles.txt_EEPY, 'String', sprintf("%.3f", EEPose0(2, 4)));
set(handles.txt_EEPZ, 'String', sprintf("%.3f", EEPose0(3, 4)));
set(handles.txt_EERR, 'String', sprintf("%.3f", orient(1)));
set(handles.txt_EERP, 'String', sprintf("%.3f", orient(2)));
set(handles.txt_EERY, 'String', sprintf("%.3f", orient(3)));
set(handles.slider_q1, 'Value', localQ0(1));
set(handles.txt_q1, 'String', sprintf("%.3f", localQ0(1)));
set(handles.slider_q2, 'Value', localQ0(2));
set(handles.txt_q2, 'String', sprintf("%.3f", localQ0(2)));
set(handles.slider_q3, 'Value', localQ0(3));
set(handles.txt_q3, 'String', sprintf("%.3f", localQ0(3)));
set(handles.slider_q4, 'Value', localQ0(4));
set(handles.txt_q4, 'String', sprintf("%.3f", localQ0(4)));
set(handles.slider_q5, 'Value', localQ0(5));
set(handles.txt_q5, 'String', sprintf("%.3f", localQ0(5)));
set(handles.slider_LR, 'Value', LRValue);

guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function txt_LRX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_LRX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_JogXPos.
function btn_JogXPos_Callback(hObject, eventdata, handles)
% hObject    handle to btn_JogXPos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
localQ0 = handles.robot.model.getpos;
EEPose0 = handles.robot.model.fkine(localQ0);
EEPose1 = EEPose0 * transl(handles.teachDist, 0, 0);
localQ1 = handles.robot.model.ikcon(EEPose1, localQ0);
handles.robot.model.animate(localQ1);

localQ0 = rad2deg(handles.robot.model.getpos);
EEPose0 = handles.robot.model.fkine(deg2rad(localQ0));
orient = tr2rpy(EEPose0, 'deg');
set(handles.txt_EEPX, 'String', sprintf("%.3f", EEPose0(1, 4)));
set(handles.txt_EEPY, 'String', sprintf("%.3f", EEPose0(2, 4)));
set(handles.txt_EEPZ, 'String', sprintf("%.3f", EEPose0(3, 4)));
set(handles.txt_EERR, 'String', sprintf("%.3f", orient(1)));
set(handles.txt_EERP, 'String', sprintf("%.3f", orient(2)));
set(handles.txt_EERY, 'String', sprintf("%.3f", orient(3)));
set(handles.slider_q1, 'Value', localQ0(1));
set(handles.txt_q1, 'String', sprintf("%.3f", localQ0(1)));
set(handles.slider_q2, 'Value', localQ0(2));
set(handles.txt_q2, 'String', sprintf("%.3f", localQ0(2)));
set(handles.slider_q3, 'Value', localQ0(3));
set(handles.txt_q3, 'String', sprintf("%.3f", localQ0(3)));
set(handles.slider_q4, 'Value', localQ0(4));
set(handles.txt_q4, 'String', sprintf("%.3f", localQ0(4)));
set(handles.slider_q5, 'Value', localQ0(5));
set(handles.txt_q5, 'String', sprintf("%.3f", localQ0(5)));

guidata(hObject, handles);

% --- Executes on button press in btn_JogXNeg.
function btn_JogXNeg_Callback(hObject, eventdata, handles)
% hObject    handle to btn_JogXNeg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
localQ0 = handles.robot.model.getpos;
EEPose0 = handles.robot.model.fkine(localQ0);
EEPose1 = EEPose0 * transl(-handles.teachDist, 0, 0);
localQ1 = handles.robot.model.ikcon(EEPose1, localQ0);
handles.robot.model.animate(localQ1);

localQ0 = rad2deg(handles.robot.model.getpos);
EEPose0 = handles.robot.model.fkine(deg2rad(localQ0));
orient = tr2rpy(EEPose0, 'deg');
set(handles.txt_EEPX, 'String', sprintf("%.3f", EEPose0(1, 4)));
set(handles.txt_EEPY, 'String', sprintf("%.3f", EEPose0(2, 4)));
set(handles.txt_EEPZ, 'String', sprintf("%.3f", EEPose0(3, 4)));
set(handles.txt_EERR, 'String', sprintf("%.3f", orient(1)));
set(handles.txt_EERP, 'String', sprintf("%.3f", orient(2)));
set(handles.txt_EERY, 'String', sprintf("%.3f", orient(3)));
set(handles.slider_q1, 'Value', localQ0(1));
set(handles.txt_q1, 'String', sprintf("%.3f", localQ0(1)));
set(handles.slider_q2, 'Value', localQ0(2));
set(handles.txt_q2, 'String', sprintf("%.3f", localQ0(2)));
set(handles.slider_q3, 'Value', localQ0(3));
set(handles.txt_q3, 'String', sprintf("%.3f", localQ0(3)));
set(handles.slider_q4, 'Value', localQ0(4));
set(handles.txt_q4, 'String', sprintf("%.3f", localQ0(4)));
set(handles.slider_q5, 'Value', localQ0(5));
set(handles.txt_q5, 'String', sprintf("%.3f", localQ0(5)));

guidata(hObject, handles);

% --- Executes on button press in btn_JogYPos.
function btn_JogYPos_Callback(hObject, eventdata, handles)
% hObject    handle to btn_JogYPos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
localQ0 = handles.robot.model.getpos;
EEPose0 = handles.robot.model.fkine(localQ0);
EEPose1 = EEPose0 * transl(0, handles.teachDist, 0);
localQ1 = handles.robot.model.ikcon(EEPose1, localQ0);
handles.robot.model.animate(localQ1);

localQ0 = rad2deg(handles.robot.model.getpos);
EEPose0 = handles.robot.model.fkine(deg2rad(localQ0));
orient = tr2rpy(EEPose0, 'deg');
set(handles.txt_EEPX, 'String', sprintf("%.3f", EEPose0(1, 4)));
set(handles.txt_EEPY, 'String', sprintf("%.3f", EEPose0(2, 4)));
set(handles.txt_EEPZ, 'String', sprintf("%.3f", EEPose0(3, 4)));
set(handles.txt_EERR, 'String', sprintf("%.3f", orient(1)));
set(handles.txt_EERP, 'String', sprintf("%.3f", orient(2)));
set(handles.txt_EERY, 'String', sprintf("%.3f", orient(3)));
set(handles.slider_q1, 'Value', localQ0(1));
set(handles.txt_q1, 'String', sprintf("%.3f", localQ0(1)));
set(handles.slider_q2, 'Value', localQ0(2));
set(handles.txt_q2, 'String', sprintf("%.3f", localQ0(2)));
set(handles.slider_q3, 'Value', localQ0(3));
set(handles.txt_q3, 'String', sprintf("%.3f", localQ0(3)));
set(handles.slider_q4, 'Value', localQ0(4));
set(handles.txt_q4, 'String', sprintf("%.3f", localQ0(4)));
set(handles.slider_q5, 'Value', localQ0(5));
set(handles.txt_q5, 'String', sprintf("%.3f", localQ0(5)));

guidata(hObject, handles);


% --- Executes on button press in btn_JogYNeg.
function btn_JogYNeg_Callback(hObject, eventdata, handles)
% hObject    handle to btn_JogYNeg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
localQ0 = handles.robot.model.getpos;
EEPose0 = handles.robot.model.fkine(localQ0);
EEPose1 = EEPose0 * transl(0, -handles.teachDist, 0);
localQ1 = handles.robot.model.ikcon(EEPose1, localQ0);
handles.robot.model.animate(localQ1);

localQ0 = rad2deg(handles.robot.model.getpos);
EEPose0 = handles.robot.model.fkine(deg2rad(localQ0));
orient = tr2rpy(EEPose0, 'deg');
set(handles.txt_EEPX, 'String', sprintf("%.3f", EEPose0(1, 4)));
set(handles.txt_EEPY, 'String', sprintf("%.3f", EEPose0(2, 4)));
set(handles.txt_EEPZ, 'String', sprintf("%.3f", EEPose0(3, 4)));
set(handles.txt_EERR, 'String', sprintf("%.3f", orient(1)));
set(handles.txt_EERP, 'String', sprintf("%.3f", orient(2)));
set(handles.txt_EERY, 'String', sprintf("%.3f", orient(3)));
set(handles.slider_q1, 'Value', localQ0(1));
set(handles.txt_q1, 'String', sprintf("%.3f", localQ0(1)));
set(handles.slider_q2, 'Value', localQ0(2));
set(handles.txt_q2, 'String', sprintf("%.3f", localQ0(2)));
set(handles.slider_q3, 'Value', localQ0(3));
set(handles.txt_q3, 'String', sprintf("%.3f", localQ0(3)));
set(handles.slider_q4, 'Value', localQ0(4));
set(handles.txt_q4, 'String', sprintf("%.3f", localQ0(4)));
set(handles.slider_q5, 'Value', localQ0(5));
set(handles.txt_q5, 'String', sprintf("%.3f", localQ0(5)));

guidata(hObject, handles);

% --- Executes on button press in btn_JogZPos.
function btn_JogZPos_Callback(hObject, eventdata, handles)
% hObject    handle to btn_JogZPos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
localQ0 = handles.robot.model.getpos;
EEPose0 = handles.robot.model.fkine(localQ0);
EEPose1 = EEPose0 * transl(0, 0, -handles.teachDist);
localQ1 = handles.robot.model.ikcon(EEPose1, localQ0);
handles.robot.model.animate(localQ1);

localQ0 = rad2deg(handles.robot.model.getpos);
EEPose0 = handles.robot.model.fkine(deg2rad(localQ0))
orient = tr2rpy(EEPose0, 'deg');
set(handles.txt_EEPX, 'String', sprintf("%.3f", EEPose0(1, 4)));
set(handles.txt_EEPY, 'String', sprintf("%.3f", EEPose0(2, 4)));
set(handles.txt_EEPZ, 'String', sprintf("%.3f", EEPose0(3, 4)));
set(handles.txt_EERR, 'String', sprintf("%.3f", orient(1)));
set(handles.txt_EERP, 'String', sprintf("%.3f", orient(2)));
set(handles.txt_EERY, 'String', sprintf("%.3f", orient(3)));
set(handles.slider_q1, 'Value', localQ0(1));
set(handles.txt_q1, 'String', sprintf("%.3f", localQ0(1)));
set(handles.slider_q2, 'Value', localQ0(2));
set(handles.txt_q2, 'String', sprintf("%.3f", localQ0(2)));
set(handles.slider_q3, 'Value', localQ0(3));
set(handles.txt_q3, 'String', sprintf("%.3f", localQ0(3)));
set(handles.slider_q4, 'Value', localQ0(4));
set(handles.txt_q4, 'String', sprintf("%.3f", localQ0(4)));
set(handles.slider_q5, 'Value', localQ0(5));
set(handles.txt_q5, 'String', sprintf("%.3f", localQ0(5)));

guidata(hObject, handles);

% --- Executes on button press in btn_JogZNeg.
function btn_JogZNeg_Callback(hObject, eventdata, handles)
% hObject    handle to btn_JogZNeg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
localQ0 = handles.robot.model.getpos;
EEPose0 = handles.robot.model.fkine(localQ0);
EEPose1 = EEPose0 * transl(0, 0, handles.teachDist);
localQ1 = handles.robot.model.ikcon(EEPose1, localQ0);
handles.robot.model.animate(localQ1);

localQ0 = rad2deg(handles.robot.model.getpos);
EEPose0 = handles.robot.model.fkine(deg2rad(localQ0));
orient = tr2rpy(EEPose0, 'deg');
set(handles.txt_EEPX, 'String', sprintf("%.3f", EEPose0(1, 4)));
set(handles.txt_EEPY, 'String', sprintf("%.3f", EEPose0(2, 4)));
set(handles.txt_EEPZ, 'String', sprintf("%.3f", EEPose0(3, 4)));
set(handles.txt_EERR, 'String', sprintf("%.3f", orient(1)));
set(handles.txt_EERP, 'String', sprintf("%.3f", orient(2)));
set(handles.txt_EERY, 'String', sprintf("%.3f", orient(3)));
set(handles.slider_q1, 'Value', localQ0(1));
set(handles.txt_q1, 'String', sprintf("%.3f", localQ0(1)));
set(handles.slider_q2, 'Value', localQ0(2));
set(handles.txt_q2, 'String', sprintf("%.3f", localQ0(2)));
set(handles.slider_q3, 'Value', localQ0(3));
set(handles.txt_q3, 'String', sprintf("%.3f", localQ0(3)));
set(handles.slider_q4, 'Value', localQ0(4));
set(handles.txt_q4, 'String', sprintf("%.3f", localQ0(4)));
set(handles.slider_q5, 'Value', localQ0(5));
set(handles.txt_q5, 'String', sprintf("%.3f", localQ0(5)));

guidata(hObject, handles);


function txt_invX_Callback(hObject, eventdata, handles)
% hObject    handle to txt_invX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_invX as text
%        str2double(get(hObject,'String')) returns contents of txt_invX as a double


% --- Executes during object creation, after setting all properties.
function txt_invX_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_invX (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_invY_Callback(hObject, eventdata, handles)
% hObject    handle to txt_invY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_invY as text
%        str2double(get(hObject,'String')) returns contents of txt_invY as a double


% --- Executes during object creation, after setting all properties.
function txt_invY_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_invY (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txt_invZ_Callback(hObject, eventdata, handles)
% hObject    handle to txt_invZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_invZ as text
%        str2double(get(hObject,'String')) returns contents of txt_invZ as a double


% --- Executes during object creation, after setting all properties.
function txt_invZ_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_invZ (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btn_inverse.
function btn_inverse_Callback(hObject, eventdata, handles)
% hObject    handle to btn_inverse (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
EEPose = transl(str2double(handles.txt_invX.String), str2double(handles.txt_invY.String), str2double(handles.txt_invZ.String)) * trotx(pi);
q0 = handles.robot.model.getpos;
q1 = handles.robot.model.ikcon(EEPose, q0);
handles.robot.model.animate(q1);

localQ0 = rad2deg(handles.robot.model.getpos);
EEPose0 = handles.robot.model.fkine(deg2rad(localQ0));
orient = tr2rpy(EEPose0, 'deg');
set(handles.txt_EEPX, 'String', sprintf("%.3f", EEPose0(1, 4)));
set(handles.txt_EEPY, 'String', sprintf("%.3f", EEPose0(2, 4)));
set(handles.txt_EEPZ, 'String', sprintf("%.3f", EEPose0(3, 4)));
set(handles.txt_EERR, 'String', sprintf("%.3f", orient(1)));
set(handles.txt_EERP, 'String', sprintf("%.3f", orient(2)));
set(handles.txt_EERY, 'String', sprintf("%.3f", orient(3)));
set(handles.slider_q1, 'Value', localQ0(1));
set(handles.txt_q1, 'String', sprintf("%.3f", localQ0(1)));
set(handles.slider_q2, 'Value', localQ0(2));
set(handles.txt_q2, 'String', sprintf("%.3f", localQ0(2)));
set(handles.slider_q3, 'Value', localQ0(3));
set(handles.txt_q3, 'String', sprintf("%.3f", localQ0(3)));
set(handles.slider_q4, 'Value', localQ0(4));
set(handles.txt_q4, 'String', sprintf("%.3f", localQ0(4)));
set(handles.slider_q5, 'Value', localQ0(5));
set(handles.txt_q5, 'String', sprintf("%.3f", localQ0(5)));

guidata(hObject, handles);


% --- Executes on slider movement.
function slider_q1_Callback(hObject, eventdata, handles)
% hObject    handle to slider_q1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

localQ0 = rad2deg(handles.robot.model.getpos);
localQ0(1) = handles.slider_q1.Value;
handles.robot.model.animate(deg2rad(localQ0));

EEPose0 = handles.robot.model.fkine(deg2rad(localQ0));
orient = tr2rpy(EEPose0, 'deg');
set(handles.txt_EEPX, 'String', sprintf("%.3f", EEPose0(1, 4)));
set(handles.txt_EEPY, 'String', sprintf("%.3f", EEPose0(2, 4)));
set(handles.txt_EEPZ, 'String', sprintf("%.3f", EEPose0(3, 4)));
set(handles.txt_EERR, 'String', sprintf("%.3f", orient(1)));
set(handles.txt_EERP, 'String', sprintf("%.3f", orient(2)));
set(handles.txt_EERY, 'String', sprintf("%.3f", orient(3)));
set(handles.slider_q1, 'Value', localQ0(1));
set(handles.txt_q1, 'String', sprintf("%.3f", localQ0(1)));
set(handles.slider_q2, 'Value', localQ0(2));
set(handles.txt_q2, 'String', sprintf("%.3f", localQ0(2)));
set(handles.slider_q3, 'Value', localQ0(3));
set(handles.txt_q3, 'String', sprintf("%.3f", localQ0(3)));
set(handles.slider_q4, 'Value', localQ0(4));
set(handles.txt_q4, 'String', sprintf("%.3f", localQ0(4)));
set(handles.slider_q5, 'Value', localQ0(5));
set(handles.txt_q5, 'String', sprintf("%.3f", localQ0(5)));

guidata(hObject, handles);



function txt_q1_Callback(hObject, eventdata, handles)
% hObject    handle to txt_q1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_q1 as text
%        str2double(get(hObject,'String')) returns contents of txt_q1 as a double
localQ0 = rad2deg(handles.robot.model.getpos);
localQ0(1) = str2double(handles.txt_q1.String);
handles.robot.model.animate(deg2rad(localQ0));

EEPose0 = handles.robot.model.fkine(deg2rad(localQ0));
orient = tr2rpy(EEPose0, 'deg');
set(handles.txt_EEPX, 'String', sprintf("%.3f", EEPose0(1, 4)));
set(handles.txt_EEPY, 'String', sprintf("%.3f", EEPose0(2, 4)));
set(handles.txt_EEPZ, 'String', sprintf("%.3f", EEPose0(3, 4)));
set(handles.txt_EERR, 'String', sprintf("%.3f", orient(1)));
set(handles.txt_EERP, 'String', sprintf("%.3f", orient(2)));
set(handles.txt_EERY, 'String', sprintf("%.3f", orient(3)));
set(handles.slider_q1, 'Value', localQ0(1));
set(handles.txt_q1, 'String', sprintf("%.3f", localQ0(1)));
set(handles.slider_q2, 'Value', localQ0(2));
set(handles.txt_q2, 'String', sprintf("%.3f", localQ0(2)));
set(handles.slider_q3, 'Value', localQ0(3));
set(handles.txt_q3, 'String', sprintf("%.3f", localQ0(3)));
set(handles.slider_q4, 'Value', localQ0(4));
set(handles.txt_q4, 'String', sprintf("%.3f", localQ0(4)));
set(handles.slider_q5, 'Value', localQ0(5));
set(handles.txt_q5, 'String', sprintf("%.3f", localQ0(5)));

guidata(hObject, handles);



% --- Executes during object creation, after setting all properties.
function txt_q1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_q1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider_q2_Callback(hObject, eventdata, handles)
% hObject    handle to slider_q2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
localQ0 = rad2deg(handles.robot.model.getpos);
localQ0(2) = handles.slider_q2.Value;
handles.robot.model.animate(deg2rad(localQ0));

EEPose0 = handles.robot.model.fkine(deg2rad(localQ0));
orient = tr2rpy(EEPose0, 'deg');
set(handles.txt_EEPX, 'String', sprintf("%.3f", EEPose0(1, 4)));
set(handles.txt_EEPY, 'String', sprintf("%.3f", EEPose0(2, 4)));
set(handles.txt_EEPZ, 'String', sprintf("%.3f", EEPose0(3, 4)));
set(handles.txt_EERR, 'String', sprintf("%.3f", orient(1)));
set(handles.txt_EERP, 'String', sprintf("%.3f", orient(2)));
set(handles.txt_EERY, 'String', sprintf("%.3f", orient(3)));
set(handles.slider_q1, 'Value', localQ0(1));
set(handles.txt_q1, 'String', sprintf("%.3f", localQ0(1)));
set(handles.slider_q2, 'Value', localQ0(2));
set(handles.txt_q2, 'String', sprintf("%.3f", localQ0(2)));
set(handles.slider_q3, 'Value', localQ0(3));
set(handles.txt_q3, 'String', sprintf("%.3f", localQ0(3)));
set(handles.slider_q4, 'Value', localQ0(4));
set(handles.txt_q4, 'String', sprintf("%.3f", localQ0(4)));
set(handles.slider_q5, 'Value', localQ0(5));
set(handles.txt_q5, 'String', sprintf("%.3f", localQ0(5)));

guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function slider_q2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_q2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function txt_q2_Callback(hObject, eventdata, handles)
% hObject    handle to txt_q2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_q2 as text
%        str2double(get(hObject,'String')) returns contents of txt_q2 as a double
localQ0 = rad2deg(handles.robot.model.getpos);
localQ0(2) = str2double(handles.txt_q2.String);
handles.robot.model.animate(deg2rad(localQ0));

EEPose0 = handles.robot.model.fkine(deg2rad(localQ0));
orient = tr2rpy(EEPose0, 'deg');
set(handles.txt_EEPX, 'String', sprintf("%.3f", EEPose0(1, 4)));
set(handles.txt_EEPY, 'String', sprintf("%.3f", EEPose0(2, 4)));
set(handles.txt_EEPZ, 'String', sprintf("%.3f", EEPose0(3, 4)));
set(handles.txt_EERR, 'String', sprintf("%.3f", orient(1)));
set(handles.txt_EERP, 'String', sprintf("%.3f", orient(2)));
set(handles.txt_EERY, 'String', sprintf("%.3f", orient(3)));
set(handles.slider_q1, 'Value', localQ0(1));
set(handles.txt_q1, 'String', sprintf("%.3f", localQ0(1)));
set(handles.slider_q2, 'Value', localQ0(2));
set(handles.txt_q2, 'String', sprintf("%.3f", localQ0(2)));
set(handles.slider_q3, 'Value', localQ0(3));
set(handles.txt_q3, 'String', sprintf("%.3f", localQ0(3)));
set(handles.slider_q4, 'Value', localQ0(4));
set(handles.txt_q4, 'String', sprintf("%.3f", localQ0(4)));
set(handles.slider_q5, 'Value', localQ0(5));
set(handles.txt_q5, 'String', sprintf("%.3f", localQ0(5)));

guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function txt_q2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_q2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider_q3_Callback(hObject, eventdata, handles)
% hObject    handle to slider_q3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
localQ0 = rad2deg(handles.robot.model.getpos);
localQ0(3) = handles.slider_q3.Value;
handles.robot.model.animate(deg2rad(localQ0));

EEPose0 = handles.robot.model.fkine(deg2rad(localQ0));
orient = tr2rpy(EEPose0, 'deg');
set(handles.txt_EEPX, 'String', sprintf("%.3f", EEPose0(1, 4)));
set(handles.txt_EEPY, 'String', sprintf("%.3f", EEPose0(2, 4)));
set(handles.txt_EEPZ, 'String', sprintf("%.3f", EEPose0(3, 4)));
set(handles.txt_EERR, 'String', sprintf("%.3f", orient(1)));
set(handles.txt_EERP, 'String', sprintf("%.3f", orient(2)));
set(handles.txt_EERY, 'String', sprintf("%.3f", orient(3)));
set(handles.slider_q1, 'Value', localQ0(1));
set(handles.txt_q1, 'String', sprintf("%.3f", localQ0(1)));
set(handles.slider_q2, 'Value', localQ0(2));
set(handles.txt_q2, 'String', sprintf("%.3f", localQ0(2)));
set(handles.slider_q3, 'Value', localQ0(3));
set(handles.txt_q3, 'String', sprintf("%.3f", localQ0(3)));
set(handles.slider_q4, 'Value', localQ0(4));
set(handles.txt_q4, 'String', sprintf("%.3f", localQ0(4)));
set(handles.slider_q5, 'Value', localQ0(5));
set(handles.txt_q5, 'String', sprintf("%.3f", localQ0(5)));

guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function slider_q3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_q3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function txt_q3_Callback(hObject, eventdata, handles)
% hObject    handle to txt_q3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_q3 as text
%        str2double(get(hObject,'String')) returns contents of txt_q3 as a double
localQ0 = rad2deg(handles.robot.model.getpos);
localQ0(3) = str2double(handles.txt_q3.String);
handles.robot.model.animate(deg2rad(localQ0));

EEPose0 = handles.robot.model.fkine(deg2rad(localQ0));
orient = tr2rpy(EEPose0, 'deg');
set(handles.txt_EEPX, 'String', sprintf("%.3f", EEPose0(1, 4)));
set(handles.txt_EEPY, 'String', sprintf("%.3f", EEPose0(2, 4)));
set(handles.txt_EEPZ, 'String', sprintf("%.3f", EEPose0(3, 4)));
set(handles.txt_EERR, 'String', sprintf("%.3f", orient(1)));
set(handles.txt_EERP, 'String', sprintf("%.3f", orient(2)));
set(handles.txt_EERY, 'String', sprintf("%.3f", orient(3)));
set(handles.slider_q1, 'Value', localQ0(1));
set(handles.txt_q1, 'String', sprintf("%.3f", localQ0(1)));
set(handles.slider_q2, 'Value', localQ0(2));
set(handles.txt_q2, 'String', sprintf("%.3f", localQ0(2)));
set(handles.slider_q3, 'Value', localQ0(3));
set(handles.txt_q3, 'String', sprintf("%.3f", localQ0(3)));
set(handles.slider_q4, 'Value', localQ0(4));
set(handles.txt_q4, 'String', sprintf("%.3f", localQ0(4)));
set(handles.slider_q5, 'Value', localQ0(5));
set(handles.txt_q5, 'String', sprintf("%.3f", localQ0(5)));

guidata(hObject, handles);



% --- Executes during object creation, after setting all properties.
function txt_q3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_q3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider_q4_Callback(hObject, eventdata, handles)
% hObject    handle to slider_q4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
localQ0 = rad2deg(handles.robot.model.getpos);
localQ0(4) = handles.slider_q4.Value;
handles.robot.model.animate(deg2rad(localQ0));

EEPose0 = handles.robot.model.fkine(deg2rad(localQ0));
orient = tr2rpy(EEPose0, 'deg');
set(handles.txt_EEPX, 'String', sprintf("%.3f", EEPose0(1, 4)));
set(handles.txt_EEPY, 'String', sprintf("%.3f", EEPose0(2, 4)));
set(handles.txt_EEPZ, 'String', sprintf("%.3f", EEPose0(3, 4)));
set(handles.txt_EERR, 'String', sprintf("%.3f", orient(1)));
set(handles.txt_EERP, 'String', sprintf("%.3f", orient(2)));
set(handles.txt_EERY, 'String', sprintf("%.3f", orient(3)));
set(handles.slider_q1, 'Value', localQ0(1));
set(handles.txt_q1, 'String', sprintf("%.3f", localQ0(1)));
set(handles.slider_q2, 'Value', localQ0(2));
set(handles.txt_q2, 'String', sprintf("%.3f", localQ0(2)));
set(handles.slider_q3, 'Value', localQ0(3));
set(handles.txt_q3, 'String', sprintf("%.3f", localQ0(3)));
set(handles.slider_q4, 'Value', localQ0(4));
set(handles.txt_q4, 'String', sprintf("%.3f", localQ0(4)));
set(handles.slider_q5, 'Value', localQ0(5));
set(handles.txt_q5, 'String', sprintf("%.3f", localQ0(5)));

guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function slider_q4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_q4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function txt_q4_Callback(hObject, eventdata, handles)
% hObject    handle to txt_q4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_q4 as text
%        str2double(get(hObject,'String')) returns contents of txt_q4 as a double
localQ0 = rad2deg(handles.robot.model.getpos);
localQ0(4) = str2double(handles.txt_q4.String);
handles.robot.model.animate(deg2rad(localQ0));

EEPose0 = handles.robot.model.fkine(deg2rad(localQ0));
orient = tr2rpy(EEPose0, 'deg');
set(handles.txt_EEPX, 'String', sprintf("%.3f", EEPose0(1, 4)));
set(handles.txt_EEPY, 'String', sprintf("%.3f", EEPose0(2, 4)));
set(handles.txt_EEPZ, 'String', sprintf("%.3f", EEPose0(3, 4)));
set(handles.txt_EERR, 'String', sprintf("%.3f", orient(1)));
set(handles.txt_EERP, 'String', sprintf("%.3f", orient(2)));
set(handles.txt_EERY, 'String', sprintf("%.3f", orient(3)));
set(handles.slider_q1, 'Value', localQ0(1));
set(handles.txt_q1, 'String', sprintf("%.3f", localQ0(1)));
set(handles.slider_q2, 'Value', localQ0(2));
set(handles.txt_q2, 'String', sprintf("%.3f", localQ0(2)));
set(handles.slider_q3, 'Value', localQ0(3));
set(handles.txt_q3, 'String', sprintf("%.3f", localQ0(3)));
set(handles.slider_q4, 'Value', localQ0(4));
set(handles.txt_q4, 'String', sprintf("%.3f", localQ0(4)));
set(handles.slider_q5, 'Value', localQ0(5));
set(handles.txt_q5, 'String', sprintf("%.3f", localQ0(5)));

guidata(hObject, handles);



% --- Executes during object creation, after setting all properties.
function txt_q4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_q4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider_q5_Callback(hObject, eventdata, handles)
% hObject    handle to slider_q5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
localQ0 = rad2deg(handles.robot.model.getpos);
localQ0(5) = handles.slider_q5.Value;
handles.robot.model.animate(deg2rad(localQ0));

EEPose0 = handles.robot.model.fkine(deg2rad(localQ0));
orient = tr2rpy(EEPose0, 'deg');
set(handles.txt_EEPX, 'String', sprintf("%.3f", EEPose0(1, 4)));
set(handles.txt_EEPY, 'String', sprintf("%.3f", EEPose0(2, 4)));
set(handles.txt_EEPZ, 'String', sprintf("%.3f", EEPose0(3, 4)));
set(handles.txt_EERR, 'String', sprintf("%.3f", orient(1)));
set(handles.txt_EERP, 'String', sprintf("%.3f", orient(2)));
set(handles.txt_EERY, 'String', sprintf("%.3f", orient(3)));
set(handles.slider_q1, 'Value', localQ0(1));
set(handles.txt_q1, 'String', sprintf("%.3f", localQ0(1)));
set(handles.slider_q2, 'Value', localQ0(2));
set(handles.txt_q2, 'String', sprintf("%.3f", localQ0(2)));
set(handles.slider_q3, 'Value', localQ0(3));
set(handles.txt_q3, 'String', sprintf("%.3f", localQ0(3)));
set(handles.slider_q4, 'Value', localQ0(4));
set(handles.txt_q4, 'String', sprintf("%.3f", localQ0(4)));
set(handles.slider_q5, 'Value', localQ0(5));
set(handles.txt_q5, 'String', sprintf("%.3f", localQ0(5)));

guidata(hObject, handles);


% --- Executes during object creation, after setting all properties.
function slider_q5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider_q5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function txt_q5_Callback(hObject, eventdata, handles)
% hObject    handle to txt_q5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txt_q5 as text
%        str2double(get(hObject,'String')) returns contents of txt_q5 as a double
localQ0 = rad2deg(handles.robot.model.getpos);
localQ0(5) = str2double(handles.txt_q5.String);
handles.robot.model.animate(deg2rad(localQ0));

EEPose0 = handles.robot.model.fkine(deg2rad(localQ0));
orient = tr2rpy(EEPose0, 'deg');
set(handles.txt_EEPX, 'String', sprintf("%.3f", EEPose0(1, 4)));
set(handles.txt_EEPY, 'String', sprintf("%.3f", EEPose0(2, 4)));
set(handles.txt_EEPZ, 'String', sprintf("%.3f", EEPose0(3, 4)));
set(handles.txt_EERR, 'String', sprintf("%.3f", orient(1)));
set(handles.txt_EERP, 'String', sprintf("%.3f", orient(2)));
set(handles.txt_EERY, 'String', sprintf("%.3f", orient(3)));
set(handles.slider_q1, 'Value', localQ0(1));
set(handles.txt_q1, 'String', sprintf("%.3f", localQ0(1)));
set(handles.slider_q2, 'Value', localQ0(2));
set(handles.txt_q2, 'String', sprintf("%.3f", localQ0(2)));
set(handles.slider_q3, 'Value', localQ0(3));
set(handles.txt_q3, 'String', sprintf("%.3f", localQ0(3)));
set(handles.slider_q4, 'Value', localQ0(4));
set(handles.txt_q4, 'String', sprintf("%.3f", localQ0(4)));
set(handles.slider_q5, 'Value', localQ0(5));
set(handles.txt_q5, 'String', sprintf("%.3f", localQ0(5)));

guidata(hObject, handles);



% --- Executes during object creation, after setting all properties.
function txt_q5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txt_q5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

function UpdateFields(hObject, eventdata, handles)
handles = guidata(hObject);
localQ = rad2deg(handles.robot.model.getpos);EEPose0 = handles.robot.model.fkine(deg2rad(localQ0));
disp(1);
EEPose = handles.robot.model.fkine(localQ)
disp(2);
set(handles.slider_q1, 'Value', localQ(1));
set(handles.txt_q1, 'String', sprintf('%.3f', localQ(1)));
set(handles.slider_q2, 'Value', localQ(2));
set(handles.txt_q2, 'String', sprintf('%.3f', localQ(2)));
set(handles.slider_q3, 'Value', localQ(3));
set(handles.txt_q3, 'String', sprintf('%.3f', localQ(3)));
set(handles.slider_q4, 'Value', localQ(4));
set(handles.txt_q4, 'String', sprintf('%.3f', localQ(4)));
set(handles.slider_q5, 'Value', localQ(5));
set(handles.txt_q5, 'String', sprintf('%.3f', localQ(5)));

set(handles.txt_EEPX, 'String', sprintf('%.3f', EEPose(1, 4)));
set(handles.txt_EEPY, 'String', sprintf('%.3f', EEPose(2, 4)));
set(handles.txt_EEPZ, 'String', sprintf('%.3f', EEPose(3, 4)));
guidata(hObject, handles);


% --- Executes on button press in check_Controller.
function check_Controller_Callback(hObject, eventdata, handles)
% hObject    handle to check_Controller (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of check_Controller
%value = get(handles.check_Controller, 'Value'); for outside this function
if get(hObject, 'Value') == 1
    disp("Check box on");
    dt = 0.15;
    n = 0;
    q = handles.robot.model.getpos;
    tic;
    linearMultiplier = 0.01;
    while (get(hObject, 'Value') == 1)
        n = n + 1;
        if n >= 10
            n = 0;
        [axes, buttons, povs] = read(handles.joy);
        
        %non rmrc teach pendant to keep joint 4 parallel
        xMov = -axes(2) * linearMultiplier;
        yMov = axes(1) * linearMultiplier;
        zMov = axes(3) * linearMultiplier;
        LRMov = -axes(5) * linearMultiplier;
        %{
        if handles.robot.model.base(1, 4) >= handles.startingBase(1, 4) + handles.robot.linearRailTravelDist
            disp(1);
            handles.robot.model.base(1, 4) = handles.startingBase(1, 4) + handles.robot.linearRailTravelDist;
            if LRMov < 0
                handles.robot.model.base = handles.robot.model.base * transl(LRMov, 0, 0);
                q = handles.robot.model.getpos;
                handles.robot.model.animate(q);
                drawnow();
            end
        end
        if handles.robot.model.base(1, 4) <= handles.startingBase(1, 4)
            disp(2);
            handles.robot.model.base(1, 4) = handles.startingBase(1, 4);
            if LRMov > 0
                handles.robot.model.base = handles.robot.model.base * transl(LRMov, 0, 0);
                q = handles.robot.model.getpos;
                handles.robot.model.animate(q);
                drawnow();
            end
        end
        %}
        handles.robot.model.base = handles.robot.model.base * transl(LRMov, 0, 0);
        q = handles.robot.model.getpos;
        
        if handles.robot.model.base(1, 4) >= handles.startingBase(1, 4) + handles.robot.linearRailTravelDist
            handles.robot.model.base(1, 4) = handles.startingBase(1, 4) + handles.robot.linearRailTravelDist;
        elseif handles.robot.model.base(1, 4) <= handles.startingBase(1, 4)
            handles.robot.model.base(1, 4) = handles.startingBase(1, 4);
        end
        handles.robot.model.animate(q);
        
        q0 = handles.robot.model.getpos;
        EEPose0 = handles.robot.model.fkine(q0);
        EEPose1 = EEPose0 * transl(xMov, yMov, zMov);
        q1 = handles.robot.model.ikcon(EEPose1, q0);
        
        handles.robot.model.animate(q1);
        drawnow(); 
        
        localQ0 = rad2deg(handles.robot.model.getpos);
        EEPose0 = handles.robot.model.fkine(deg2rad(localQ0));
        
        orient = tr2rpy(EEPose0, 'deg');
        set(handles.txt_EEPX, 'String', sprintf("%.3f", EEPose0(1, 4)));
        set(handles.txt_EEPY, 'String', sprintf("%.3f", EEPose0(2, 4)));
        set(handles.txt_EEPZ, 'String', sprintf("%.3f", EEPose0(3, 4)));
        set(handles.txt_EERR, 'String', sprintf("%.3f", orient(1)));
        set(handles.txt_EERP, 'String', sprintf("%.3f", orient(2)));
        set(handles.txt_EERY, 'String', sprintf("%.3f", orient(3)));
        set(handles.slider_q1, 'Value', localQ0(1));
        set(handles.txt_q1, 'String', sprintf("%.3f", localQ0(1)));
        set(handles.slider_q2, 'Value', localQ0(2));
        set(handles.txt_q2, 'String', sprintf("%.3f", localQ0(2)));
        set(handles.slider_q3, 'Value', localQ0(3));
        set(handles.txt_q3, 'String', sprintf("%.3f", localQ0(3)));
        set(handles.slider_q4, 'Value', localQ0(4));
        set(handles.txt_q4, 'String', sprintf("%.3f", localQ0(4)));
        set(handles.slider_q5, 'Value', localQ0(5));
        set(handles.txt_q5, 'String', sprintf("%.3f", localQ0(5)));
        LRValue = handles.robot.model.base(1, 4) - handles.startingBase(1, 4);
        set(handles.slider_LR, 'Value', LRValue);
        set(handles.txt_LRX, 'String', sprintf("%.3f", LRValue));

        end
    end
    
    %{
    while (get(hObject, 'Value') == 1)
        n=n+1; % increment step count

        % read joystick
        [axes, buttons, povs] = read(handles.joy);

        % -------------------------------------------------------------
        % YOUR CODE GOES HERE
        % 1 - turn joystick input into an end-effector velocity command
        Kv = 0.3; % linear velocity gain
        Kw = 0.8; % angular velocity gain

        vx = Kv*axes(1);
        vy = Kv*-axes(2);
        vz = Kv*axes(3);

        wx = Kw*axes(4);
        wy = Kw*axes(3);
        wz = Kw*(buttons(6)-buttons(8));

        dx = [vx;vy;vz;wx;wy;wz]; % combined velocity vector

        % 2 - use DLS J inverse to calculate joint velocity
        lambda = 0.5;
        J = handles.robot.model.jacob0(q);
        Jinv_dls = inv((J'*J)+lambda^2*eye(5))*J';
        dq = Jinv_dls*dx;

        % 3 - apply joint velocity to step robot joint angles 
        q = q + dq'*dt;

        % -------------------------------------------------------------

        % Update plot
        handles.robot.model.animate(q);  
        drawnow();

        % wait until loop time elapsed
        if (toc > dt*n)
            warning('Loop %i took too much time - consider increating dt',n);
        end
        while (toc < dt*n); % wait until loop time (dt) has elapsed 
        end
    
    end
%}
elseif get(hObject, 'Value') == 0
    disp("Check box off");
end
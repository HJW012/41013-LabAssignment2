function varargout = MainMenu(varargin)
% MAINMENU MATLAB code for MainMenu.fig
%      MAINMENU, by itself, creates a new MAINMENU or raises the existing
%      singleton*.
%
%      H = MAINMENU returns the handle to a new MAINMENU or the handle to
%      the existing singleton*.
%
%      MAINMENU('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in MAINMENU.M with the given input arguments.
%
%      MAINMENU('Property','Value',...) creates a new MAINMENU or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before MainMenu_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to MainMenu_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help MainMenu

% Last Modified by GUIDE v2.5 03-Jun-2020 00:16:21

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @MainMenu_OpeningFcn, ...
                   'gui_OutputFcn',  @MainMenu_OutputFcn, ...
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


% --- Executes just before MainMenu is made visible.
function MainMenu_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to MainMenu (see VARARGIN)

% Choose default command line output for MainMenu
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes MainMenu wait for user response (see UIRESUME)
 uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = MainMenu_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
temp = [];
try
    temp(1) = handles.mode; %Operation mode: 1 is Start Simulation, 2 is Advanced Teach, 3 is Safety Demo
catch
    
end
varargout{1} = temp;
delete(handles.figure1);


% --- Executes on button press in btn_StartSim.
function btn_StartSim_Callback(hObject, eventdata, handles)
% hObject    handle to btn_StartSim (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.mode = 1;
guidata(hObject,handles);
figure1_CloseRequestFcn(handles.figure1, eventdata, handles);

% --- Executes on button press in btn_AdvancedTeach.
function btn_AdvancedTeach_Callback(hObject, eventdata, handles)
% hObject    handle to btn_AdvancedTeach (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.mode = 2;
guidata(hObject,handles);
figure1_CloseRequestFcn(handles.figure1, eventdata, handles);

% --- Executes on button press in btn_SafetyDemo.
function btn_SafetyDemo_Callback(hObject, eventdata, handles)
% hObject    handle to btn_SafetyDemo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.mode = 3;
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
else
    delete(hObject);
end


% --- Executes on button press in btn_Exit.
function btn_Exit_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Exit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.mode = 0;
guidata(hObject,handles);
figure1_CloseRequestFcn(handles.figure1, eventdata, handles);
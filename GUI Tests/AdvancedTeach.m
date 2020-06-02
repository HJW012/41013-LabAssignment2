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

% Last Modified by GUIDE v2.5 02-Jun-2020 12:37:28

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

end

% --- Executes just before AdvancedTeach is made visible.
function AdvancedTeach_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to AdvancedTeach (see VARARGIN)

% Choose default command line output for AdvancedTeach
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes AdvancedTeach wait for user response (see UIRESUME)
% uiwait(handles.figure1);
end

% --- Outputs from this function are returned to the command line.
function varargout = AdvancedTeach_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;
end

% --- Executes during object creation, after setting all properties.
function figure1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called
end

% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end

% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
end

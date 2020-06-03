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

% Last Modified by GUIDE v2.5 04-Jun-2020 01:08:01

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

% Update handles structure
id = 2; % Note: may need to be changed if multiple joysticks present
handles.joy = vrjoystick(id);
axes(handles.axes_Safety)
hold on;
handles.robot = Dobot();
handles.robot.Display();
EEPose1 = transl(0.23, 0, 0.1) * trotx(pi);
q = handles.robot.model.ikcon(EEPose1, handles.robot.model.getpos);
handles.robot.model.animate(q);

guidata(hObject, handles);

% UIWAIT makes SafetyDemo wait for user response (see UIRESUME)
% uiwait(handles.figure1);


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

% Hint: get(hObject,'Value') returns toggle state of check_Controller


% --- Executes on button press in btn_StopSign.
function btn_StopSign_Callback(hObject, eventdata, handles)
% hObject    handle to btn_StopSign (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.symbolPresent = true;
guidata(hObject, handles);


% --- Executes on button press in btn_Retreat.
function btn_Retreat_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Retreat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


pStar = [662 362 362 662; 362 362 662 662];
P = [0.25, 0.2, 0.2, 0.25;
     0.025, 0.025, -0.025, -0.025;
     -0.1, -0.1, -0.1, -0.1];
cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
'resolution', [1024 1024], 'centre', [512 512],'name', 'UR10camera');

q0 = handles.robot.model.getpos';
fps = 25;
lambda = 0.6;
depth = mean(P(1, :));
Tc0 = handles.robot.model.fkine(handles.robot.model.getpos);
drawnow;

cam.T = Tc0;
cam.plot_camera('Tcam',Tc0, 'label','scale',0.015);
spheres = plot_sphere(P, 0.005, 'b');

p = cam.plot(P, 'Tcam', Tc0);
cam.clf()
cam.plot(pStar, '*'); % create the camera view
cam.hold(true);
cam.plot(P, 'Tcam', Tc0, 'o'); % create the camera view
pause(2)
cam.hold(true);
cam.plot(P); 

ksteps = 0;
pressed = false;
while (true)
    [axes, buttons, povs] = read(handles.joy);
    if buttons(1) && ~pressed
       P(3, 1) = P(3, 1) + 0.02;
       P(3, 2) = P(3, 2) + 0.02;
       P(3, 3) = P(3, 3) + 0.02;
       P(3, 4) = P(3, 4) + 0.02;
       delete(spheres);
       spheres = plot_sphere(P, 0.005, 'b');
       pressed = true;
    elseif ~buttons(1)
        pressed = false;
    end
    uv = cam.plot(P)
        
        % compute image plane error as a column
        e = pStar-uv   % feature error
        e = e(:);
        Zest = []
        % compute the Jacobian
        if isempty(depth)
            % exact depth from simulation (not possible in practice)
            pt = homtrans(inv(Tcam), P)
            J = cam.visjac_p(uv, pt(3,:) );
        elseif ~isempty(Zest)
            J = cam.visjac_p(uv, Zest);
        else
            disp('here');
            J = cam.visjac_p(uv, depth )
        end

        % compute the velocity of camera in camera frame
        try
            v = lambda * pinv(J) * e;
        catch
            status = -1;
            return
        end
        fprintf('v: %.3f %.3f %.3f %.3f %.3f %.3f\n', v);

        %compute robot's Jacobian and inverse
        J2 = handles.robot.model.jacobn(q0);
        Jinv = pinv(J2);
        % get joint velocities
        qp = Jinv*v;

         
         %Maximum angular velocity cannot exceed 180 degrees/s
         ind=find(qp>pi);
         if ~isempty(ind)
             qp(ind)=pi;
         end
         ind=find(qp<-pi);
         if ~isempty(ind)
             qp(ind)=-pi;
         end

        %Update joints 
        q = q0 + (1/fps)*qp;
        handles.robot.model.animate(q');

        %Get camera location
        Tc = handles.robot.model.fkine(q);
        cam.T = Tc;

        drawnow


         pause(1/fps)

        if ~isempty(200) && (ksteps > 200)
            break;
        end
        
        %update current joint position
        q0 = q;
end
 
guidata(hObject, handles);

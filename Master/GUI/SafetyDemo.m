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

% Last Modified by GUIDE v2.5 05-Jun-2020 17:51:32

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
handles.startRetreating = false;
handles.useController = false;
handles.btn_Retreat.Enable = 'off';
handles.btn_XPos.Enable = 'off';
handles.btn_XNeg.Enable = 'off';
handles.btn_YPos.Enable = 'off';
handles.btn_YNeg.Enable = 'off';
handles.btn_ZPos.Enable = 'off';
handles.btn_ZNeg.Enable = 'off';
handles.check_Controller.Enable = 'off';
handles.linearMultiplier = 0.01;
handles.robot = Dobot();
handles.robot.Display();
EEPose1 = transl(0.23, 0, 0.1) * trotx(pi);
q = handles.robot.model.ikcon(EEPose1, handles.robot.model.getpos);
handles.robot.model.animate(q);
handles.P = [0.175, 0.25, 0.2, 0.2, 0.25, 0.275;
             0, 0.025, 0.025, -0.025, -0.025, 0;
            -0.1, -0.1, -0.1, -0.1, -0.1, -0.1];
handles.pStar = [212 662 362 362 662 812; 512 362 362 662 662 512];
handles.cam = CentralCamera('focal', 0.08, 'pixel', 10e-5, ...
            'resolution', [1024 1024], 'centre', [512 512],'name', 'Dobot Camera');
handles.fps = 25;
handles.lambda = 0.6;
handles.depth = mean(handles.P(1, :));

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
ksteps = 0;
pressed = false;
delete(handles.spheres);
disp('Check box change');
guidata(hObject, handles);
if handles.startRetreating
    while get(hObject, 'Value') == 1.0 && handles.startRetreating
        handles = guidata(hObject);
        [axes, buttons, povs] = read(handles.joy);
        xChange = axes(2) * handles.linearMultiplier;
        yChange = axes(1) * handles.linearMultiplier;
        zChange = axes(3) * handles.linearMultiplier;
        
        for i = 1:6
            handles.P(1, i) = handles.P(1, i) + xChange;
            handles.P(2, i) = handles.P(2, i) + yChange;
            handles.P(3, i) = handles.P(3, i) + zChange;
        end
        delete(handles.spheres);
        for i = 1:6
            handles.spheres(:, i) = plot_sphere(handles.P(:, i), 0.005, 'b');
        end
           

        handles.uv = handles.cam.plot(handles.P)
        
        % compute image plane error as a column
        handles.e = handles.pStar-handles.uv   % feature error
        handles.e = handles.e(:);
        handles.Zest = []
        % compute the Jacobian
        if isempty(handles.depth)
            % exact depth from simulation (not possible in practice)
            handles.pt = homtrans(inv(handles.Tcam), handles.P)
            handles.J = handles.cam.visjac_p(handles.uv, handles.pt(3,:) );
        elseif ~isempty(handles.Zest)
            handles.J = handles.cam.visjac_p(handles.uv, handles.Zest);
        else
            disp('here');
            handles.J = handles.cam.visjac_p(handles.uv, handles.depth )
        end

        % compute the velocity of camera in camera frame
        try
            handles.v = handles.lambda * pinv(handles.J) * handles.e;
        catch
            status = -1;
            return
        end

        %compute robot's Jacobian and inverse
        handles.J2 = handles.robot.model.jacobn(handles.q0);
        handles.Jinv = pinv(handles.J2);
        % get joint velocities
        handles.qp = handles.Jinv*handles.v;

         
         %Maximum angular velocity cannot exceed 180 degrees/s
         handles.ind=find(handles.qp>pi);
         if ~isempty(handles.ind)
             handles.qp(handles.ind)=pi;
         end
         handles.ind=find(handles.qp<-pi);
         if ~isempty(handles.ind)
             handles.qp(handles.ind)=-pi;
         end

        %Update joints 
        handles.q = handles.q0 + (1/handles.fps)*handles.qp;
        handles.robot.model.animate(handles.q');

        %Get camera location
        handles.Tc = handles.robot.model.fkine(handles.q);
        handles.cam.T = handles.Tc;

        drawnow


         pause(1/handles.fps)

        if ~isempty(200) && (ksteps > 200)
            break;
        end
        
        %update current joint position
        handles.q0 = handles.q;
        guidata(hObject, handles);
    end
    
    while get(handles.check_Controller, 'Value') == 0 && handles.startRetreating
        handles = guidata(hObject);
        delete(handles.spheres);
        for i = 1:6
            handles.spheres(:, i) = plot_sphere(handles.P(:, i), 0.005, 'b');
        end
           

        handles.uv = handles.cam.plot(handles.P)
        
        % compute image plane error as a column
        handles.e = handles.pStar-handles.uv   % feature error
        handles.e = handles.e(:);
        handles.Zest = []
        % compute the Jacobian
        if isempty(handles.depth)
            % exact depth from simulation (not possible in practice)
            handles.pt = homtrans(inv(handles.Tcam), handles.P)
            handles.J = handles.cam.visjac_p(handles.uv, handles.pt(3,:) );
        elseif ~isempty(handles.Zest)
            handles.J = handles.cam.visjac_p(handles.uv, handles.Zest);
        else
            disp('here');
            handles.J = handles.cam.visjac_p(handles.uv, handles.depth )
        end

        % compute the velocity of camera in camera frame
        try
            handles.v = handles.lambda * pinv(handles.J) * handles.e;
        catch
            status = -1;
            return
        end

        %compute robot's Jacobian and inverse
        handles.J2 = handles.robot.model.jacobn(handles.q0);
        handles.Jinv = pinv(handles.J2);
        % get joint velocities
        handles.qp = handles.Jinv*handles.v;

         
         %Maximum angular velocity cannot exceed 180 degrees/s
         handles.ind=find(handles.qp>pi);
         if ~isempty(handles.ind)
             handles.qp(handles.ind)=pi;
         end
         handles.ind=find(handles.qp<-pi);
         if ~isempty(handles.ind)
             handles.qp(handles.ind)=-pi;
         end

        %Update joints 
        handles.q = handles.q0 + (1/handles.fps)*handles.qp;
        handles.robot.model.animate(handles.q');

        %Get camera location
        handles.Tc = handles.robot.model.fkine(handles.q);
        handles.cam.T = handles.Tc;

        drawnow


         pause(1/handles.fps)

        if ~isempty(200) && (ksteps > 200)
            break;
        end
        
        %update current joint position
        handles.q0 = handles.q;
        guidata(hObject, handles);
    end
else
    
end

guidata(hObject, handles);

% --- Executes on button press in btn_StopSign.
function btn_StopSign_Callback(hObject, eventdata, handles)
% hObject    handle to btn_StopSign (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.q0 = handles.robot.model.getpos';

handles.Tc0 = handles.robot.model.fkine(handles.robot.model.getpos);
drawnow;

handles.cam.T = handles.Tc0;
handles.cam.plot_camera('Tcam',handles.Tc0, 'label','scale',0.015);
for i = 1:6
    handles.spheres(:, i) = plot_sphere(handles.P(:, i), 0.005, 'b');
end

handles.p = handles.cam.plot(handles.P, 'Tcam', handles.Tc0);
handles.cam.clf()
handles.cam.plot(handles.pStar, '*'); % create the camera view
handles.cam.hold(true);
handles.cam.plot(handles.P, 'Tcam', handles.Tc0, 'o'); % create the camera view
handles.cam.hold(true);
handles.cam.plot(handles.P); 

handles.symbolPresent = true;
handles.btn_Retreat.Enable = 'on';
handles.btn_XPos.Enable = 'on';
handles.btn_XNeg.Enable = 'on';
handles.btn_YPos.Enable = 'on';
handles.btn_YNeg.Enable = 'on';
handles.btn_ZPos.Enable = 'on';
handles.btn_ZNeg.Enable = 'on';
handles.check_Controller.Enable = 'on';



guidata(hObject, handles);


% --- Executes on button press in btn_Retreat.
function btn_Retreat_Callback(hObject, eventdata, handles)
% hObject    handle to btn_Retreat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

handles.startRetreating = true;
pause(0.1);
ksteps = 0;
guidata(hObject, handles);
while get(handles.check_Controller, 'Value') == 0 && handles.startRetreating
    handles = guidata(hObject);
    delete(handles.spheres);
    for i = 1:6
        handles.spheres(:, i) = plot_sphere(handles.P(:, i), 0.005, 'b');
    end


    handles.uv = handles.cam.plot(handles.P)

    % compute image plane error as a column
    handles.e = handles.pStar-handles.uv   % feature error
    handles.e = handles.e(:);
    handles.Zest = []
    % compute the Jacobian
    if isempty(handles.depth)
        % exact depth from simulation (not possible in practice)
        handles.pt = homtrans(inv(handles.Tcam), handles.P)
        handles.J = handles.cam.visjac_p(handles.uv, handles.pt(3,:) );
    elseif ~isempty(handles.Zest)
        handles.J = handles.cam.visjac_p(handles.uv, handles.Zest);
    else
        disp('here');
        handles.J = handles.cam.visjac_p(handles.uv, handles.depth )
    end

    % compute the velocity of camera in camera frame
    try
        handles.v = handles.lambda * pinv(handles.J) * handles.e;
    catch
        status = -1;
        return
    end

    %compute robot's Jacobian and inverse
    handles.J2 = handles.robot.model.jacobn(handles.q0);
    handles.Jinv = pinv(handles.J2);
    % get joint velocities
    handles.qp = handles.Jinv*handles.v;


     %Maximum angular velocity cannot exceed 180 degrees/s
     handles.ind=find(handles.qp>pi);
     if ~isempty(handles.ind)
         handles.qp(handles.ind)=pi;
     end
     handles.ind=find(handles.qp<-pi);
     if ~isempty(handles.ind)
         handles.qp(handles.ind)=-pi;
     end

    %Update joints 
    handles.q = handles.q0 + (1/handles.fps)*handles.qp;
    handles.robot.model.animate(handles.q');

    %Get camera location
    handles.Tc = handles.robot.model.fkine(handles.q);
    handles.cam.T = handles.Tc;

    drawnow


     pause(1/handles.fps)

    if ~isempty(200) && (ksteps > 200)
        break;
    end

    %update current joint position
    handles.q0 = handles.q;
    guidata(hObject, handles);
end
    

guidata(hObject, handles);


% --- Executes on button press in btn_XPos.
function btn_XPos_Callback(hObject, eventdata, handles)
% hObject    handle to btn_XPos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.spheres

% --- Executes on button press in btn_XNeg.
function btn_XNeg_Callback(hObject, eventdata, handles)
% hObject    handle to btn_XNeg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in btn_YPos.
function btn_YPos_Callback(hObject, eventdata, handles)
% hObject    handle to btn_YPos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in btn_YNeg.
function btn_YNeg_Callback(hObject, eventdata, handles)
% hObject    handle to btn_YNeg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in btn_ZPos.
function btn_ZPos_Callback(hObject, eventdata, handles)
% hObject    handle to btn_ZPos (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in btn_ZNeg.
function btn_ZNeg_Callback(hObject, eventdata, handles)
% hObject    handle to btn_ZNeg (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in btn_StopRetreat.
function btn_StopRetreat_Callback(hObject, eventdata, handles)
% hObject    handle to btn_StopRetreat (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
handles.startRetreating = false;
guidata(hObject, handles);
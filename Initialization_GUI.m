function varargout = Initialization_GUI(varargin)
% INITIALIZATION_GUI MATLAB code for Initialization_GUI.fig
%      INITIALIZATION_GUI, by itself, creates a new INITIALIZATION_GUI or raises the existing
%      singleton*.
%
%      H = INITIALIZATION_GUI returns the handle to a new INITIALIZATION_GUI or the handle to
%      the existing singleton*.
%
%      INITIALIZATION_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in INITIALIZATION_GUI.M with the given input arguments.
%
%      INITIALIZATION_GUI('Property','Value',...) creates a new INITIALIZATION_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before Initialization_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to Initialization_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help Initialization_GUI

% Last Modified by GUIDE v2.5 30-Jan-2018 15:15:34

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
    'gui_Singleton',  gui_Singleton, ...
    'gui_OpeningFcn', @Initialization_GUI_OpeningFcn, ...
    'gui_OutputFcn',  @Initialization_GUI_OutputFcn, ...
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


% --- Executes just before Initialization_GUI is made visible.
function Initialization_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to Initialization_GUI (see VARARGIN)

% Choose default command line output for Initialization_GUI
handles.output = hObject;

set(handles.slider1,'Min',0, 'Max', 180);
set(handles.slider2,'Min', 0, 'Max', 180);
set(handles.slider3,'Min', 0.0, 'Max', 180);
set(handles.slider4,'Min', 0, 'Max', 270);
set(handles.slider5,'Min', 0, 'Max', 270);
set(handles.slider6,'Min', 0, 'Max', 180);
set(handles.slider7,'Min', 0, 'Max', 180);
set(handles.slider8,'Min', -180, 'Max', 180);
set(handles.slider9,'Min',0, 'Max', 180);
set(handles.slider10,'Min',-180, 'Max', 180);
set(handles.slider11,'Min',0, 'Max', 180);

% This is to load the pre-computed data structure
load('Pre_Load_Structure.mat');
P = Environmental_Features_Initialization(P);

handles.P = P;

% This is the robot state
rIx = 0;
rIy = 0.5;
theta = 90;                   % Checked
q1 = 90;                      % Checked
q2 = 120;                     % Checked
q3 = 235;                     % Checked
q4 = 150;                     % Checked
q5 = 150;                     % Checked
q6 = 90;                      % Checked
q7 = 60;                      % Checked
q8 = 45;                      % Checked
q9 = 60;                      % Checked
q10 = 30;                     % Checked

q_tot = [rIx;rIy;theta;q1;q2;q3;q4;q5;q6;q7;q8;q9;q10];

handles.rIx   = q_tot(1);
handles.rIy   = q_tot(2);
handles.theta = q_tot(3);
handles.q1    = q_tot(4);
handles.q2    = q_tot(5);
handles.q3    = q_tot(6);
handles.q4    = q_tot(7);
handles.q5    = q_tot(8);
handles.q6    = q_tot(9);
handles.q7    = q_tot(10);
handles.q8    = q_tot(11);
handles.q9    = q_tot(12);
handles.q10   = q_tot(13);

% Pre-set the configuration values
set(handles.edit28, 'String',   q_tot(1));
set(handles.edit27, 'String',   q_tot(2));
set(handles.edit2,  'String',   q_tot(3));
set(handles.edit3,  'String',   q_tot(4));
set(handles.edit4,  'String',   q_tot(5));
set(handles.edit5,  'String',   q_tot(6));
set(handles.edit6,  'String',   q_tot(7));
set(handles.edit7,  'String',   q_tot(8));
set(handles.edit8,  'String',   q_tot(9));
set(handles.edit9,  'String',   q_tot(10));
set(handles.edit10, 'String',   q_tot(11));
set(handles.edit11, 'String',   q_tot(12));
set(handles.edit12, 'String',   q_tot(13));

set(handles.slider1, 'value', q_tot(3));
set(handles.slider2, 'value', q_tot(4));
set(handles.slider3, 'value', q_tot(5));
set(handles.slider4, 'value', q_tot(6));
set(handles.slider5, 'value', q_tot(7));
set(handles.slider6, 'value', q_tot(8));
set(handles.slider7, 'value', q_tot(9));
set(handles.slider8, 'value', q_tot(10));
set(handles.slider9, 'value', q_tot(11));
set(handles.slider10, 'value', q_tot(12));
set(handles.slider11, 'value', q_tot(13));

qdot_tot = [1; -1; -1; -1; -1; 1; -1; -1; -1; 1; -1; 1; 1];
handles.rIxdot   = qdot_tot(1);
handles.rIydot   = qdot_tot(2);
handles.thetadot = qdot_tot(3);
handles.q1dot    = qdot_tot(4);
handles.q2dot    = qdot_tot(5);
handles.q3dot    = qdot_tot(6);
handles.q4dot    = qdot_tot(7);
handles.q5dot    = qdot_tot(8);
handles.q6dot    = qdot_tot(9);
handles.q7dot    = qdot_tot(10);
handles.q8dot    = qdot_tot(11);
handles.q9dot    = qdot_tot(12);
handles.q10dot   = qdot_tot(13);

set(handles.edit13, 'String', qdot_tot(1));
set(handles.edit14, 'String', qdot_tot(2));
set(handles.edit15, 'String', qdot_tot(3));
set(handles.edit16, 'String', qdot_tot(4));
set(handles.edit17, 'String', qdot_tot(5));
set(handles.edit18, 'String', qdot_tot(6));
set(handles.edit19, 'String', qdot_tot(7));
set(handles.edit20, 'String', qdot_tot(8));
set(handles.edit21, 'String', qdot_tot(9));
set(handles.edit22, 'String', qdot_tot(10));
set(handles.edit23, 'String', qdot_tot(11));
set(handles.edit24, 'String', qdot_tot(12));
set(handles.edit25, 'String', qdot_tot(13));

Robot_Configuration_Plot(handles);

image1 = imread('robotplot.jpg');
axes(handles.axes2);
imshow(image1);

% Initialize the pop-up menu
% The default contact status is that point A is sticking on the ground
set(handles.popupmenu1,'value',1);
set(handles.popupmenu2,'value',1);

set(handles.popupmenu3,'value',3);
set(handles.popupmenu4,'value',3);

set(handles.popupmenu5,'value',3);
set(handles.popupmenu6,'value',3);

set(handles.popupmenu7,'value',3);
set(handles.popupmenu8,'value',3);

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes Initialization_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);

function Robot_Configuration_Plot(handles)
% This function is used to update the position of the robot configuration
% with the help of P
rIx = handles.rIx;
rIy = handles.rIy;
theta = handles.theta * pi/180;
q1 = handles.q1 * pi/180;
q2 = handles.q2 * pi/180;
q3 = handles.q3 * pi/180;
q4 = handles.q4 * pi/180;
q5 = handles.q5 * pi/180;
q6 = handles.q6 * pi/180;
q7 = handles.q7 * pi/180;
q8 = handles.q8 * pi/180;
q9 = handles.q9 * pi/180;
q10 = handles.q10 * pi/180;

rA = handles.P.rA_fn(q4,q5,q6,rIx,rIy,theta);
rB = handles.P.rB_fn(q4,q5,q6,rIx,rIy,theta);
rC = handles.P.rC_fn(q1,q2,q3,rIx,rIy,theta);
rD = handles.P.rD_fn(q1,q2,q3,rIx,rIy,theta);
rE = handles.P.rE_fn(q4,q5,rIx,rIy,theta);
rF = handles.P.rF_fn(q4,rIx,rIy,theta);
rG = handles.P.rG_fn(q2,q3,rIx,rIy,theta);
rH = handles.P.rH_fn(q3,rIx,rIy,theta);
rI = handles.P.rI_fn(rIx,rIy);
rJ = handles.P.rJ_fn(rIx,rIy,theta);
rK = handles.P.rK_fn(rIx,rIy,theta);
rL = handles.P.rL_fn(q9,rIx,rIy,theta);
rM = handles.P.rM_fn(q9,q10,rIx,rIy,theta);
rN = handles.P.rN_fn(q7,rIx,rIy,theta);
rO = handles.P.rO_fn(q7,q8,rIx,rIy,theta);

% AngxIK = theta;
% AngxIH = -(2 * pi - theta - q3);
% AngxIF = -(2 * pi - theta - q4);
% AngIFx = pi + AngxIF;
% AngIHx = pi + AngxIH;
% AngxFE = -(2 * pi - AngIFx - q5);
% AngFEx = pi + AngxFE;
% AngxBA = -(q6 - AngFEx);
% AngxHG = -(2 * pi - AngIHx  -q2);
% AngHGx = pi + AngxHG;
% AngxDC = -(q1 - AngHGx);
% AngxJL = -(pi - AngxIK - q9);
% AngJLx = pi + AngxJL;
% AngMLx = q10 - (pi - AngJLx);
% AngxJN = -(q7 + q9 -AngxJL);
% AngJNx = pi + AngxJN;
% AngxNO = -(pi - q8 - AngJNx);

% p = Humanoid_Physical_Parameter();

% linklength = p.l;

% rI = [rIx, rIy]';
% rIF = linklength(3) * lamdadirection(AngxIF);
% rF = rI + rIF;
% rE = linklength(2) * lamdadirection(AngxFE) + rF;
% rA = 1/2 * linklength(1) * lamdadirection(AngxBA) + rE;
% rB = 1/2 * linklength(1) * lamdadirection(pi + AngxBA) + rE;
% rH = rI + linklength(3) * lamdadirection(AngxIH);
% rG = rH + linklength(2) * lamdadirection(AngxHG);
% rC = rG + 1/2 * linklength(1) * lamdadirection(AngxDC);
% rD = rG - 1/2 * linklength(1) * lamdadirection(AngxDC);
% rJ = rI + linklength(4) * lamdadirection(AngxIK);
% rK = rJ + linklength(5) * lamdadirection(AngxIK);
% rL = rJ + linklength(6) * lamdadirection(AngxJL);
% rM = rL + linklength(7) * lamdadirection(AngMLx);
% rN = rJ + linklength(6) * lamdadirection(AngxJN);
% rO = rN + linklength(7) * lamdadirection(AngxNO);

% rI = [rIx, rIy]';
% rIF = linklength(3) * lamdadirection(AngxIF);
% rF = rI + rIF;
% rE = linklength(2) * lamdadirection(AngxFE) + rF;
% rA = 1/2 * linklength(1) * lamdadirection(AngxBA) + rE;
% rB = 1/2 * linklength(1) * lamdadirection(pi + AngxBA) + rE;
% rH = rI + linklength(3) * lamdadirection(AngxIH);
% rG = rH + linklength(2) * lamdadirection(AngxHG);
% rC = rG + 1/2 * linklength(1) * lamdadirection(AngxDC);
% rD = rG - 1/2 * linklength(1) * lamdadirection(AngxDC);
% rJ = rI + linklength(4) * lamdadirection(AngxIK);
% rK = rJ + linklength(5) * lamdadirection(AngxIK);
% rL = rJ + linklength(6) * lamdadirection(AngxJL);
% rM = rL + linklength(7) * lamdadirection(AngMLx);
% rN = rJ + linklength(6) * lamdadirection(AngxJN);
% rO = rN + linklength(7) * lamdadirection(AngxNO);

link_color = [8 128 224] ./ 255;
Linewidth_Coef = 2.5;
% Stance foot plot
plot(handles.axes1, [rA(1),rB(1)]',[rA(2),rB(2)]','LineWidth',Linewidth_Coef, 'color', link_color);
hold(handles.axes1,'on');
% Stance shank plot
plot(handles.axes1, [rE(1),rF(1)]',[rE(2),rF(2)]','LineWidth',Linewidth_Coef, 'color', link_color);
hold(handles.axes1,'on');
% Stance thigh plot
plot(handles.axes1, [rF(1),rI(1)]',[rF(2),rI(2)]','LineWidth',Linewidth_Coef, 'color', link_color);
hold(handles.axes1,'on');
% Body plot
plot(handles.axes1, [rI(1),rK(1)]',[rI(2),rK(2)]','LineWidth',Linewidth_Coef, 'color', link_color);
hold(handles.axes1,'on');
% Stance side forearm plot
plot(handles.axes1, [rJ(1),rL(1)]',[rJ(2),rL(2)]','LineWidth',Linewidth_Coef, 'color', link_color);
hold(handles.axes1,'on');
% Stance side arm plot
plot(handles.axes1, [rL(1),rM(1)]',[rL(2),rM(2)]','LineWidth',Linewidth_Coef, 'color', link_color);
hold(handles.axes1,'on');

% Swing foot plot
plot(handles.axes1, [rC(1),rD(1)]',[rC(2),rD(2)]','LineWidth',2.5, 'color', link_color);
hold(handles.axes1,'on');
% Swing shank plot
plot(handles.axes1, [rG(1),rH(1)]',[rG(2),rH(2)]','LineWidth',2.5, 'color', link_color);
hold(handles.axes1,'on');
% Swing thigh plot
plot(handles.axes1, [rH(1),rI(1)]',[rH(2),rI(2)]','LineWidth',2.5, 'color', link_color);
hold(handles.axes1,'on');

% Swing side forearm plot
plot(handles.axes1, [rJ(1),rN(1)]',[rJ(2),rN(2)]','LineWidth',2.5, 'color', link_color);
hold(handles.axes1,'on');
% Swing side arm plot
plot(handles.axes1, [rN(1),rO(1)]',[rN(2),rO(2)]','LineWidth',2.5, 'color', link_color);
hold(handles.axes1,'on');

MarkSize_Coef = 5;
MarkerEdgeColor_Coef = [28 148 149] ./ 255;
MarkerFaceColor_Coef = [225 9 92] ./ 255;
% Edge point plots
plot(handles.axes1, rA(1),rA(2),'o','MarkerSize',MarkSize_Coef,...
    'MarkerEdgeColor',MarkerEdgeColor_Coef,...
    'MarkerFaceColor',MarkerFaceColor_Coef);  hold(handles.axes1,'on');

plot(handles.axes1, rB(1),rB(2),'o','MarkerSize',MarkSize_Coef,...
    'MarkerEdgeColor',MarkerEdgeColor_Coef,...
    'MarkerFaceColor',MarkerFaceColor_Coef);  hold(handles.axes1,'on');

for i = 1:15
    r_Point = strcat('r',char(i + 64));
    r_Point_Var = genvarname(r_Point);
    evalc(['r_Point_Plot', ' = ', r_Point_Var]);
    plot(handles.axes1, r_Point_Plot(1),r_Point_Plot(2),'o','MarkerSize',MarkSize_Coef,...
        'MarkerEdgeColor',MarkerEdgeColor_Coef,...
        'MarkerFaceColor',MarkerFaceColor_Coef);  hold(handles.axes1,'on');
end
axis(handles.axes1, 'equal');
hold(handles.axes1,'off');

function slide_val = slider_scale_fn(slider_handle, unscaled_val)
% This function is used to scale the given value to a proper value
slider_min = get(slider_handle, 'min');
slider_max = get(slider_handle, 'max');
slider_range = slider_max - slider_min;
if unscaled_val <=slider_min
    unscaled_val = slider_min;
else
    if unscaled_val>=slider_max
        unscaled_val = slider_max;
    end
end
slide_val = (unscaled_val - slider_min)/slider_range;


% --- Outputs from this function are returned to the command line.
function varargout = Initialization_GUI_OutputFcn(hObject, eventdata, handles)
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in popupmenu1.
function popupmenu1_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu1


% --- Executes during object creation, after setting all properties.
function popupmenu1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu11.
function popupmenu11_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu11 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu11


% --- Executes during object creation, after setting all properties.
function popupmenu11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu12.
function popupmenu12_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu12 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu12


% --- Executes during object creation, after setting all properties.
function popupmenu12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu9.
function popupmenu9_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu9 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu9


% --- Executes during object creation, after setting all properties.
function popupmenu9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu10.
function popupmenu10_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu10 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu10


% --- Executes during object creation, after setting all properties.
function popupmenu10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu7.
function popupmenu7_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu7 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu7


% --- Executes during object creation, after setting all properties.
function popupmenu7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu8.
function popupmenu8_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu8 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu8


% --- Executes during object creation, after setting all properties.
function popupmenu8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu3.
function popupmenu3_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu3 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu3


% --- Executes during object creation, after setting all properties.
function popupmenu3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu4.
function popupmenu4_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu4 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu4


% --- Executes during object creation, after setting all properties.
function popupmenu4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu5.
function popupmenu5_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu5 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu5


% --- Executes during object creation, after setting all properties.
function popupmenu5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu6.
function popupmenu6_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu6 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu6


% --- Executes during object creation, after setting all properties.
function popupmenu6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on selection change in popupmenu2.
function popupmenu2_Callback(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns popupmenu2 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from popupmenu2


% --- Executes during object creation, after setting all properties.
function popupmenu2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to popupmenu2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.edit2,'String', num2str(get(hObject, 'Value')));
addlistener(hObject, 'Value','PostSet',@slider1_listener_Callback);
handles.theta = get(hObject, 'Value');

if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end


function slider1_listener_Callback(hObject, eventdata, handles)
handle = guidata(eventdata.AffectedObject);
set(handle.edit2,'String', num2str(get(handle.slider1, 'Value')));

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

function edit2_Callback(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit2 as text
%        str2double(get(hObject,'String')) returns contents of edit2 as a double

set(handles.slider1,'value',str2double(get(hObject, 'String')));
handles.theta = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end


% --- Executes during object creation, after setting all properties.
function edit2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.edit3,'String', num2str(get(hObject, 'Value')));
addlistener(hObject, 'Value','PostSet',@slider2_listener_Callback);
handles.q1 = get(hObject, 'Value');
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end

function slider2_listener_Callback(hObject, eventdata, handles)
handle = guidata(eventdata.AffectedObject);
set(handle.edit3,'String', num2str(get(handle.slider2, 'Value')));

% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit3_Callback(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit3 as text
%        str2double(get(hObject,'String')) returns contents of edit3 as a double
set(handles.slider2,'value',str2double(get(hObject, 'String')));
handles.q1 = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end


% --- Executes during object creation, after setting all properties.
function edit3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

set(handles.edit4,'String', num2str(get(hObject, 'Value')));
addlistener(hObject, 'Value','PostSet',@slider3_listener_Callback);
handles.q2 = get(hObject, 'Value');
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end

function slider3_listener_Callback(hObject, eventdata, handles)
handle = guidata(eventdata.AffectedObject);
set(handle.edit4,'String', num2str(get(handle.slider3, 'Value')));

% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit4_Callback(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit4 as text
%        str2double(get(hObject,'String')) returns contents of edit4 as a double
set(handles.slider3,'value',str2double(get(hObject, 'String')));
handles.q2 = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end

% --- Executes during object creation, after setting all properties.
function edit4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider4_Callback(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

set(handles.edit5,'String', num2str(get(hObject, 'Value')));
addlistener(hObject, 'Value','PostSet',@slider4_listener_Callback);
handles.q3 = get(hObject, 'Value');
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end

function slider4_listener_Callback(hObject, eventdata, handles)
handle = guidata(eventdata.AffectedObject);
set(handle.edit5,'String', num2str(get(handle.slider4, 'Value')));

% --- Executes during object creation, after setting all properties.
function slider4_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider4 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit5_Callback(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit5 as text
%        str2double(get(hObject,'String')) returns contents of edit5 as a double
set(handles.slider4,'value',str2double(get(hObject, 'String')));
handles.q3 = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end

% --- Executes during object creation, after setting all properties.
function edit5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider5_Callback(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.edit6,'String', num2str(get(hObject, 'Value')));
addlistener(hObject, 'Value','PostSet',@slider5_listener_Callback);
handles.q4 = get(hObject, 'Value');
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end

function slider5_listener_Callback(hObject, eventdata, handles)
handle = guidata(eventdata.AffectedObject);
set(handle.edit6,'String', num2str(get(handle.slider5, 'Value')));

% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit6_Callback(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit6 as text
%        str2double(get(hObject,'String')) returns contents of edit6 as a double
set(handles.slider5,'value',str2double(get(hObject, 'String')));
handles.q4 = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end

% --- Executes during object creation, after setting all properties.
function edit6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider6_Callback(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

set(handles.edit7,'String', num2str(get(hObject, 'Value')));
addlistener(hObject, 'Value','PostSet',@slider6_listener_Callback);
handles.q5 = get(hObject, 'Value');
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end

function slider6_listener_Callback(hObject, eventdata, handles)
handle = guidata(eventdata.AffectedObject);
set(handle.edit7,'String', num2str(get(handle.slider6, 'Value')));

% --- Executes during object creation, after setting all properties.
function slider6_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider6 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit7_Callback(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit7 as text
%        str2double(get(hObject,'String')) returns contents of edit7 as a double
set(handles.slider6,'value',str2double(get(hObject, 'String')));
handles.q5 = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end

% --- Executes during object creation, after setting all properties.
function edit7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider7_Callback(hObject, eventdata, handles)
% hObject    handle to slider7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

set(handles.edit8,'String', num2str(get(hObject, 'Value')));
addlistener(hObject, 'Value','PostSet',@slider7_listener_Callback);
handles.q6 = get(hObject, 'Value');
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end

function slider7_listener_Callback(hObject, eventdata, handles)
handle = guidata(eventdata.AffectedObject);
set(handle.edit8,'String', num2str(get(handle.slider7, 'Value')));

% --- Executes during object creation, after setting all properties.
function slider7_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider7 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit8_Callback(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit8 as text
%        str2double(get(hObject,'String')) returns contents of edit8 as a double
set(handles.slider7,'value',str2double(get(hObject, 'String')));
handles.q6 = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end

% --- Executes during object creation, after setting all properties.
function edit8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider8_Callback(hObject, eventdata, handles)
% hObject    handle to slider8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.edit9,'String', num2str(get(hObject, 'Value')));
addlistener(hObject, 'Value','PostSet',@slider8_listener_Callback);
handles.q7 = get(hObject, 'Value');
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end

function slider8_listener_Callback(hObject, eventdata, handles)
handle = guidata(eventdata.AffectedObject);
set(handle.edit9,'String', num2str(get(handle.slider8, 'Value')));

% --- Executes during object creation, after setting all properties.
function slider8_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider8 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit9_Callback(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit9 as text
%        str2double(get(hObject,'String')) returns contents of edit9 as a double
set(handles.slider8,'value',str2double(get(hObject, 'String')));
handles.q7 = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end

% --- Executes during object creation, after setting all properties.
function edit9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider9_Callback(hObject, eventdata, handles)
% hObject    handle to slider9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
set(handles.edit10,'String', num2str(get(hObject, 'Value')));
addlistener(hObject, 'Value','PostSet',@slider9_listener_Callback);
handles.q8 = get(hObject, 'Value');
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end

function slider9_listener_Callback(hObject, eventdata, handles)
handle = guidata(eventdata.AffectedObject);
set(handle.edit10,'String', num2str(get(handle.slider9, 'Value')));

% --- Executes during object creation, after setting all properties.
function slider9_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider9 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit10_Callback(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit10 as text
%        str2double(get(hObject,'String')) returns contents of edit10 as a double
set(handles.slider9,'value',str2double(get(hObject, 'String')));
handles.q8 = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end

% --- Executes during object creation, after setting all properties.
function edit10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider10_Callback(hObject, eventdata, handles)
% hObject    handle to slider10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

set(handles.edit11,'String', num2str(get(hObject, 'Value')));
addlistener(hObject, 'Value','PostSet',@slider10_listener_Callback);
handles.q9 = get(hObject, 'Value');
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end

function slider10_listener_Callback(hObject, eventdata, handles)
handle = guidata(eventdata.AffectedObject);
set(handle.edit11,'String', num2str(get(handle.slider10, 'Value')));

% --- Executes during object creation, after setting all properties.
function slider10_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider10 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit11_Callback(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit11 as text
%        str2double(get(hObject,'String')) returns contents of edit11 as a double
set(handles.slider10,'value',str2double(get(hObject, 'String')));
handles.q9 = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end

% --- Executes during object creation, after setting all properties.
function edit11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function slider11_Callback(hObject, eventdata, handles)
% hObject    handle to slider11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

set(handles.edit12,'String', num2str(get(hObject, 'Value')));
addlistener(hObject, 'Value','PostSet',@slider11_listener_Callback);
handles.q10 = get(hObject, 'Value');
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end

function slider11_listener_Callback(hObject, eventdata, handles)
handle = guidata(eventdata.AffectedObject);
set(handle.edit12,'String', num2str(get(handle.slider11, 'Value')));

% --- Executes during object creation, after setting all properties.
function slider11_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider11 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function edit12_Callback(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit12 as text
%        str2double(get(hObject,'String')) returns contents of edit12 as a double
set(handles.slider11,'value',str2double(get(hObject, 'String')));
handles.q10 = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end
% --- Executes during object creation, after setting all properties.
function edit12_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit12 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit13_Callback(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit13 as text
%        str2double(get(hObject,'String')) returns contents of edit13 as a double
handles.rIxdot = str2double(get(hObject, 'String'));
% guidata(hObject, handles);
if get(handles.checkbox1,'value')==0
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end

% --- Executes during object creation, after setting all properties.
function edit13_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit13 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit14_Callback(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit14 as text
%        str2double(get(hObject,'String')) returns contents of edit14 as a double
handles.rIydot = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end

% --- Executes during object creation, after setting all properties.
function edit14_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit14 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit15_Callback(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit15 as text
%        str2double(get(hObject,'String')) returns contents of edit15 as a double
handles.thetadot = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end
% --- Executes during object creation, after setting all properties.
function edit15_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit15 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit16_Callback(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit16 as text
%        str2double(get(hObject,'String')) returns contents of edit16 as a double
handles.q1dot = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end
% --- Executes during object creation, after setting all properties.
function edit16_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit16 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit17_Callback(hObject, eventdata, handles)
% hObject    handle to edit17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit17 as text
%        str2double(get(hObject,'String')) returns contents of edit17 as a double
handles.q2dot = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end
% --- Executes during object creation, after setting all properties.
function edit17_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit17 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit18_Callback(hObject, eventdata, handles)
% hObject    handle to edit18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit18 as text
%        str2double(get(hObject,'String')) returns contents of edit18 as a double
handles.q3dot = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end
% --- Executes during object creation, after setting all properties.
function edit18_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit18 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit19_Callback(hObject, eventdata, handles)
% hObject    handle to edit19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit19 as text
%        str2double(get(hObject,'String')) returns contents of edit19 as a double
handles.q4dot = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end
% --- Executes during object creation, after setting all properties.
function edit19_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit19 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit20_Callback(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit20 as text
%        str2double(get(hObject,'String')) returns contents of edit20 as a double
handles.q5dot = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end
% --- Executes during object creation, after setting all properties.
function edit20_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit20 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit21_Callback(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit21 as text
%        str2double(get(hObject,'String')) returns contents of edit21 as a double
handles.q6dot = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end
% --- Executes during object creation, after setting all properties.
function edit21_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit21 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit22_Callback(hObject, eventdata, handles)
% hObject    handle to edit22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit22 as text
%        str2double(get(hObject,'String')) returns contents of edit22 as a double
handles.q7dot = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end
% --- Executes during object creation, after setting all properties.
function edit22_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit22 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit23_Callback(hObject, eventdata, handles)
% hObject    handle to edit23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit23 as text
%        str2double(get(hObject,'String')) returns contents of edit23 as a double
handles.q8dot = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end
% --- Executes during object creation, after setting all properties.
function edit23_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit23 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit24_Callback(hObject, eventdata, handles)
% hObject    handle to edit24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit24 as text
%        str2double(get(hObject,'String')) returns contents of edit24 as a double
handles.q9dot = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end
% --- Executes during object creation, after setting all properties.
function edit24_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit24 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit25_Callback(hObject, eventdata, handles)
% hObject    handle to edit25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit25 as text
%        str2double(get(hObject,'String')) returns contents of edit25 as a double
handles.q10dot = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end
% --- Executes during object creation, after setting all properties.
function edit25_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit25 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit27_Callback(hObject, eventdata, handles)
% hObject    handle to edit27 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit27 as text
%        str2double(get(hObject,'String')) returns contents of edit27 as a double
handles.rIy = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end

% --- Executes during object creation, after setting all properties.
function edit27_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit27 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function edit28_Callback(hObject, eventdata, handles)
% hObject    handle to edit28 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit28 as text
%        str2double(get(hObject,'String')) returns contents of edit28 as a double

handles.rIx = str2double(get(hObject, 'String'));
if get(handles.checkbox1,'value')==0
    Robot_Configuration_Plot(handles);
    guidata(hObject, handles);    
else
    handles = Constraint_Optimization(handles);
    guidata(hObject, handles);
    Display_Update(handles);
    Robot_Configuration_Plot(handles);
end

% --- Executes during object creation, after setting all properties.
function edit28_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit28 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in pushbutton1.
function pushbutton1_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% This function is used to optimize the current contact condition such that
% the given contact requirement is satisfied.

% There are four candidate contacts and each contact can have three
% different contact condition.

handles = Constraint_Optimization(handles);
guidata(hObject, handles);
Display_Update(handles);
Robot_Configuration_Plot(handles);

function handles = Constraint_Optimization(handles)
x0 = [handles.rIx;...
      handles.rIy;...
      handles.theta;
      handles.q1;
      handles.q2;
      handles.q3;
      handles.q4;
      handles.q5;
      handles.q6;
      handles.q7;
      handles.q8;
      handles.q9;
      handles.q10;
      handles.rIxdot;
      handles.rIydot;
      handles.thetadot;
      handles.q1dot;
      handles.q2dot;
      handles.q3dot;
      handles.q4dot;
      handles.q5dot;
      handles.q6dot;
      handles.q7dot;
      handles.q8dot;
      handles.q9dot;
      handles.q10dot];
  
value_rAx = get(handles.popupmenu1,'value');
value_rAy = get(handles.popupmenu2,'value');
value_rBx = get(handles.popupmenu3,'value');
value_rBy = get(handles.popupmenu4,'value');
value_rCx = get(handles.popupmenu5,'value');
value_rCy = get(handles.popupmenu6,'value');
value_rDx = get(handles.popupmenu7,'value');
value_rDy = get(handles.popupmenu8,'value');

% set(handles.slider1,'Min',0, 'Max', 180);
% set(handles.slider2,'Min', 0, 'Max', 180);
% set(handles.slider3,'Min', 0.0, 'Max', 180);
% set(handles.slider4,'Min', 0, 'Max', 270);
% set(handles.slider5,'Min', 0, 'Max', 270);
% set(handles.slider6,'Min', 0, 'Max', 180);
% set(handles.slider7,'Min', 0, 'Max', 180);
% set(handles.slider8,'Min', -180, 'Max', 180);
% set(handles.slider9,'Min',0, 'Max', 180);
% set(handles.slider10,'Min',-180, 'Max', 180);
% set(handles.slider11,'Min',0, 'Max', 180);

options.lb = [-inf,  0, -90,   0,   0,   0,   0,   0,   0, -180,   0, -180,   0];
options.ub = [inf, inf,  180, 180, 180, 270, 270, 180, 180,  180, 180,  180, 180];
options.cl = [0 0 0 0 0 0 0 0];
options.cu = [0 0 0 0 0 0 0 0];
options.auxdata.value_rAx = value_rAx;
options.auxdata.value_rAy = value_rAy;
options.auxdata.value_rBx = value_rBx;
options.auxdata.value_rBy = value_rBy;
options.auxdata.value_rCx = value_rCx;
options.auxdata.value_rCy = value_rCy;
options.auxdata.value_rDx = value_rDx;
options.auxdata.value_rDy = value_rDy;
options.auxdata.x_start = x0;
options.auxdata.P = handles.P;

options.ipopt.max_iter = 100; % max. number of iterations
options.ipopt.tol = 1e-8; % accuracy to which the NLP is solved
options.ipopt.print_level = 5; % {6}, [5] is a nice compact form
options.ipopt.hessian_approximation = 'limited-memory';
options.ipopt.limited_memory_update_type = 'bfgs'; % {bfgs}, sr1
options.ipopt.limited_memory_max_history = 6; % {6}

funcs.objective         = @objective;
funcs.constraints       = @constraints;
funcs.gradient          = @gradient;

fmincon_opt = optimoptions('fmincon','Algorithm','sqp');

x = fmincon(funcs.objective,x0,[],[],[],[],options.lb,options.ub,@fmincon_constraint,fmincon_opt, options.auxdata);

handles.rIx = x(1);
handles.rIy = x(2);
handles.theta = x(3);
handles.q1 = x(4);
handles.q2 = x(5);
handles.q3 = x(6);
handles.q4 = x(7);
handles.q5 = x(8);
handles.q6 = x(9);
handles.q7 = x(10);
handles.q8 = x(11);
handles.q9 = x(12);
handles.q10 = x(13);
handles.rIxdot = x(14);
handles.rIydot = x(15);
handles.thetadot = x(16);
handles.q1dot = x(17);
handles.q2dot = x(18);
handles.q3dot = x(19);
handles.q4dot = x(20);
handles.q5dot = x(21);
handles.q6dot = x(22);
handles.q7dot = x(23);
handles.q8dot = x(24);
handles.q9dot = x(25);
handles.q10dot = x(26);

function Display_Update(handles)
% This function is used to update all the display values given an optimized
% configuration set
set(handles.edit28,'String',num2str(handles.rIx));
set(handles.edit27,'String',num2str(handles.rIy));

set(handles.slider1,'value',handles.theta);
set(handles.edit2,'String',num2str(handles.theta));

set(handles.slider2,'value',handles.q1);
set(handles.edit3,'String',num2str(handles.q1));

set(handles.slider3,'value',handles.q2);
set(handles.edit4,'String',num2str(handles.q2));

set(handles.slider4,'value',handles.q3);
set(handles.edit5,'String',num2str(handles.q3));

set(handles.slider5,'value',handles.q4);
set(handles.edit6,'String',num2str(handles.q4));

set(handles.slider6,'value',handles.q5);
set(handles.edit7,'String',num2str(handles.q5));

set(handles.slider7,'value',handles.q6);
set(handles.edit8,'String',num2str(handles.q6));

set(handles.slider8,'value',handles.q7);
set(handles.edit9,'String',num2str(handles.q7));

set(handles.slider9,'value',handles.q8);
set(handles.edit10,'String',num2str(handles.q8));

set(handles.slider10,'value',handles.q9);
set(handles.edit11,'String',num2str(handles.q9));

set(handles.slider11,'value',handles.q10);
set(handles.edit12,'String',num2str(handles.q10));

set(handles.edit13,'String',num2str(handles.rIxdot));
set(handles.edit14,'String',num2str(handles.rIydot));
set(handles.edit15,'String',num2str(handles.thetadot));
set(handles.edit16,'String',num2str(handles.q1dot));
set(handles.edit17,'String',num2str(handles.q2dot));
set(handles.edit18,'String',num2str(handles.q3dot));
set(handles.edit19,'String',num2str(handles.q4dot));
set(handles.edit20,'String',num2str(handles.q5dot));
set(handles.edit21,'String',num2str(handles.q6dot));
set(handles.edit22,'String',num2str(handles.q7dot));
set(handles.edit23,'String',num2str(handles.q8dot));
set(handles.edit24,'String',num2str(handles.q9dot));
set(handles.edit25,'String',num2str(handles.q10dot));


function obj_val = objective(x, auxdata)
% This is the value function for the optimization

x_start = x;

x_off = 90 - 0*x_start(3);

obj_val = dot(x_off, x_off);

function [c, ceq] = fmincon_constraint(x, auxdata)
rIx        = x(1);
rIy        = x(2);
theta      = x(3)  * pi/180;
q1         = x(4) * pi/180;
q2         = x(5) * pi/180;
q3         = x(6) * pi/180;
q4         = x(7) * pi/180;
q5         = x(8) * pi/180;
q6         = x(9) * pi/180;
q7         = x(10) * pi/180;
q8         = x(11) * pi/180;
q9         = x(12) * pi/180;
q10        = x(13) * pi/180;

rIxdot     = x(14);
rIydot     = x(15);
thetadot   = x(16);
q1dot      = x(17);
q2dot      = x(18);
q3dot      = x(19);
q4dot      = x(20);
q5dot      = x(21);
q6dot      = x(22);
% q7dot      = x(23);
% q8dot      = x(24);
% q9dot      = x(25);
% q10dot     = x(26);

value_rAx = auxdata.value_rAx;
value_rAy = auxdata.value_rAy;
value_rBx = auxdata.value_rBx;
value_rBy = auxdata.value_rBy;
value_rCx = auxdata.value_rCx;
value_rCy = auxdata.value_rCy;
value_rDx = auxdata.value_rDx;
value_rDy = auxdata.value_rDy;

rA_fn = auxdata.P.rA_fn;
rB_fn = auxdata.P.rB_fn;
rC_fn = auxdata.P.rC_fn;
rD_fn = auxdata.P.rD_fn;
rE_fn = auxdata.P.rE_fn;
rF_fn = auxdata.P.rF_fn;
rG_fn = auxdata.P.rG_fn;
rH_fn = auxdata.P.rH_fn;
rI_fn = auxdata.P.rI_fn;
rJ_fn = auxdata.P.rJ_fn;
rK_fn = auxdata.P.rK_fn;
rL_fn = auxdata.P.rL_fn;
rM_fn = auxdata.P.rM_fn;
rN_fn = auxdata.P.rN_fn;
rO_fn = auxdata.P.rO_fn;


vA_fn = auxdata.P.vA_fn;
vB_fn = auxdata.P.vB_fn;
vC_fn = auxdata.P.vC_fn;
vD_fn = auxdata.P.vD_fn;

rA = rA_fn(q4,q5,q6,rIx,rIy,theta);
rB = rB_fn(q4,q5,q6,rIx,rIy,theta);
rC = rC_fn(q1,q2,q3,rIx,rIy,theta);
rD = rD_fn(q1,q2,q3,rIx,rIy,theta);

rE = rE_fn(q4,q5,rIx,rIy,theta);
rF = rF_fn(q4,rIx,rIy,theta);
rG = rG_fn(q2,q3,rIx,rIy,theta);
rH = rH_fn(q3,rIx,rIy,theta);
rI = rI_fn(rIx,rIy);
rJ = rJ_fn(rIx,rIy,theta);
rK = rK_fn(rIx,rIy,theta);
rL = rL_fn(q9,rIx,rIy,theta);
rM = rM_fn(q9,q10,rIx,rIy,theta);
rN = rN_fn(q7,rIx,rIy,theta);
rO = rO_fn(q7,q8,rIx,rIy,theta);

vA = vA_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
vB = vB_fn(q4,q5,q6,q4dot,q5dot,q6dot,rIxdot,rIydot,thetadot,theta);
vC = vC_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);
vD = vD_fn(q1,q2,q3,q1dot,q2dot,q3dot,rIxdot,rIydot,thetadot,theta);

[M_rAx_Pos, M_rAy_Vel] = Selection_Matrix_Cal(value_rAx,value_rAy);
[M_rBx_Pos, M_rBy_Vel] = Selection_Matrix_Cal(value_rBx,value_rBy);
[M_rCx_Pos, M_rCy_Vel] = Selection_Matrix_Cal(value_rCx,value_rCy);
[M_rDx_Pos, M_rDy_Vel] = Selection_Matrix_Cal(value_rDx,value_rDy);

ceq = [M_rAx_Pos * rA;...
       M_rBx_Pos * rB;...
       M_rCx_Pos * rC;...
       M_rDx_Pos * rD;...
       M_rAy_Vel * vA;...
       M_rBy_Vel * vB;...
       M_rCy_Vel * vC;...
       M_rDy_Vel * vD];

eps = 0.1;
eps_rAy = eps; 
eps_rBy = eps;
eps_rCy = eps;
eps_rDy = eps;

if (value_rAy == 1) 
    eps_rAy = 0; 
end
if (value_rBy == 1) 
    eps_rBy = 0; 
end
if (value_rCy == 1) 
    eps_rCy = 0; 
end
if (value_rDy == 1) 
    eps_rDy = 0; 
end

c =   -[rA(2) - eps_rAy;...
      rB(2) - eps_rBy;...
      rC(2) - eps_rCy;...
      rD(2) - eps_rDy;...
      rE(2);...
      rF(2);...
      rG(2);...
      rH(2);...
      rI(2);...
      rJ(2);...
      rK(2);...
      rL(2);...
      rM(2);...
      rN(2);...
      rO(2)];


% --- Executes on button press in pushbutton2.
function pushbutton2_Callback(hObject, eventdata, handles)
% hObject    handle to pushbutton2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% This function is used to output the current robot state for the further
% simulation.


value_rAx = get(handles.popupmenu1,'value');
value_rAy = get(handles.popupmenu2,'value');
value_rBx = get(handles.popupmenu3,'value');
value_rBy = get(handles.popupmenu4,'value');
value_rCx = get(handles.popupmenu5,'value');
value_rCy = get(handles.popupmenu6,'value');
value_rDx = get(handles.popupmenu7,'value');
value_rDy = get(handles.popupmenu8,'value');

contact_status_ref = [value_rAx, value_rAy;...
                      value_rBx, value_rBy;...
                      value_rCx, value_rCy;...
                      value_rDx, value_rDy];
                  
rIx        = handles.rIx;
rIy        = handles.rIy;
theta      = handles.theta * pi/180;
q1         = handles.q1 * pi/180;
q2         = handles.q2 * pi/180;
q3         = handles.q3 * pi/180;
q4         = handles.q4 * pi/180;
q5         = handles.q5 * pi/180;
q6         = handles.q6 * pi/180;
q7         = handles.q7 * pi/180;
q8         = handles.q8 * pi/180;
q9         = handles.q9 * pi/180;
q10        = handles.q10 * pi/180;

rIxdot     = handles.rIxdot;
rIydot     = handles.rIydot;
thetadot   = handles.thetadot;
q1dot      = handles.q1dot;
q2dot      = handles.q2dot;
q3dot      = handles.q3dot;
q4dot      = handles.q4dot;
q5dot      = handles.q5dot;
q6dot      = handles.q6dot;
q7dot      = handles.q7dot;
q8dot      = handles.q8dot;
q9dot      = handles.q9dot;
q10dot     = handles.q10dot;

q_ref = [rIx, rIy, theta,...
        q1,q2,q3,q4,q5,q6,q7,q8,q9,q10,...
        rIxdot, rIydot, thetadot,...
       q1dot,q2dot,q3dot,q4dot,q5dot,q6dot,q7dot,q8dot,q9dot,q10dot]';

[Contact_Sequence, x0] = Eleven_Link_Optimization(contact_status_ref, q_ref, handles);

Snopt_Constraint_Formulation(handles,Contact_Sequence);

% figure
% handles.axes3 = axes;
% set(handles.axes3, 'Visible','off');                                   
% Eleven_Link_Sim(contact_status_ref, q_ref, handles);


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over slider1.
function slider1_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes during object creation, after setting all properties.
function axes2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to axes2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: place code in OpeningFcn to populate axes2


% --- Executes on button press in checkbox1.
function checkbox1_Callback(hObject, eventdata, handles)
% hObject    handle to checkbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of checkbox1

function varargout = DPGUI_beta(varargin)
% DPGUI_BETA MATLAB code for DPGUI_beta.fig
%      DPGUI_BETA, by itself, creates a new DPGUI_BETA or raises the existing
%      singleton*.
%
%      H = DPGUI_BETA returns the handle to a new DPGUI_BETA or the handle to
%      the existing singleton*.
%
%      DPGUI_BETA('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in DPGUI_BETA.M with the given input arguments.
%
%      DPGUI_BETA('Property','Value',...) creates a new DPGUI_BETA or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before DPGUI_beta_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to DPGUI_beta_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help DPGUI_beta

% Last Modified by GUIDE v2.5 30-Jun-2012 22:32:29

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @DPGUI_beta_OpeningFcn, ...
                   'gui_OutputFcn',  @DPGUI_beta_OutputFcn, ...
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


% --- Executes just before DPGUI_beta is made visible.
function DPGUI_beta_OpeningFcn(hObject, eventdata, handles, varargin)%#ok
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to DPGUI_beta (see VARARGIN)

% Choose default command line output for DPGUI_beta
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes DPGUI_beta wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = DPGUI_beta_OutputFcn(hObject, eventdata, handles) %#ok
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function Pedt_Callback(hObject, eventdata, handles)%#ok
% hObject    handle to Pedt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Pedt as text
%        str2double(get(hObject,'String')) returns contents of Pedt as a double
set_param('test_of_DP_1Hz/DP controller/Kp','Gain',get(hObject,'String'));

% --- Executes during object creation, after setting all properties.
function Pedt_CreateFcn(hObject, eventdata, handles)%#ok
% hObject    handle to Pedt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Iedt_Callback(hObject, eventdata, handles)%#ok
% hObject    handle to Iedt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Iedt as text
%        str2double(get(hObject,'String')) returns contents of Iedt as a double
set_param('test_of_DP_1Hz/DP controller/Ki','Gain',get(hObject,'String'));

% --- Executes during object creation, after setting all properties.
function Iedt_CreateFcn(hObject, eventdata, handles)%#ok
% hObject    handle to Iedt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function Dedt_Callback(hObject, eventdata, handles)%#ok
% hObject    handle to Dedt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of Dedt as text
%        str2double(get(hObject,'String')) returns contents of Dedt as a double
set_param('test_of_DP_1Hz/DP controller/Kd','Gain',get(hObject,'String'));

% --- Executes during object creation, after setting all properties.
function Dedt_CreateFcn(hObject, eventdata, handles)%#ok
% hObject    handle to Dedt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function ypedt_Callback(hObject, eventdata, handles)%#ok
% hObject    handle to ypedt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of ypedt as text
%        str2double(get(hObject,'String')) returns contents of ypedt as a double
set_param('test_of_DP_1Hz/yd','Value',get(hObject,'String'));

% --- Executes during object creation, after setting all properties.
function ypedt_CreateFcn(hObject, eventdata, handles)%#ok
% hObject    handle to ypedt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function yawedt_Callback(hObject, eventdata, handles)%#ok
% hObject    handle to yawedt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of yawedt as text
%        str2double(get(hObject,'String')) returns contents of yawedt as a double
set_param('test_of_DP_1Hz/psid','Value',get(hObject,'String'));

% --- Executes during object creation, after setting all properties.
function yawedt_CreateFcn(hObject, eventdata, handles)%#ok
% hObject    handle to yawedt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function xpedt_Callback(hObject, eventdata, handles)%#ok
% hObject    handle to xpedt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of xpedt as text
%        str2double(get(hObject,'String')) returns contents of xpedt as a double
set_param('test_of_DP_1Hz/xd','Value',get(hObject,'String'));

% --- Executes during object creation, after setting all properties.
function xpedt_CreateFcn(hObject, eventdata, handles)%#ok
% hObject    handle to xpedt (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in startpb.
function startpb_Callback(hObject, eventdata, handles)%#ok
% hObject    handle to startpb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
hw = waitbar(0.5,'Please Wait...');
load_system('test_of_DP_1Hz');
set_param('test_of_DP_1Hz','Simulationcommand','start');
delete(hw);
ad = guihandles(hObject);
set(ad.startpb,'Enable','off');
set(ad.pausepb,'Enable','on');
set(ad.continuepb,'Enable','on');
set(ad.stoppb,'Enable','on');

% --- Executes on button press in pausepb.
function pausepb_Callback(hObject, eventdata, handles)%#ok
% hObject    handle to pausepb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set_param('test_of_DP_1Hz','Simulationcommand','pause');

% --- Executes on button press in continuepb.
function continuepb_Callback(hObject, eventdata, handles)%#ok
% hObject    handle to continuepb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set_param('test_of_DP_1Hz','Simulationcommand','continue');

% --- Executes on button press in stoppb.
function stoppb_Callback(hObject, eventdata, handles)%#ok
% hObject    handle to stoppb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set_param('test_of_DP_1Hz','Simulationcommand','stop');
ad = guihandles(hObject);
set(ad.pausepb,'Enable','off');
set(ad.continuepb,'Enable','off');
set(ad.stoppb,'Enable','off');
set(ad.closepb,'Enable','on');

% --- Executes on button press in closepb.
function closepb_Callback(hObject, eventdata, handles)%#ok
% hObject    handle to closepb (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
bdclose('test_of_DP_1Hz');
close(01);
close(02);
ad = guihandles(hObject);
set(ad.closepb,'Enable','off');
set(ad.startpb,'Enable','on');
set(ad.pausepb,'Enable','off');
set(ad.continuepb,'Enable','off');
set(ad.stoppb,'Enable','off');

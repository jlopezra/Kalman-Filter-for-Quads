function varargout = UAV_UGV_CNS(varargin)
% UAV_UGV_CNS M-file for UAV_UGV_CNS.fig
%      UAV_UGV_CNS, by itself, creates a new UAV_UGV_CNS or raises the existing
%      singleton*.
%
%      H = UAV_UGV_CNS returns the handle to a new UAV_UGV_CNS or the handle to
%      the existing singleton*.
%
%      UAV_UGV_CNS('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in UAV_UGV_CNS.M with the given input arguments.
%
%      UAV_UGV_CNS('Property','Value',...) creates a new UAV_UGV_CNS or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before UAV_UGV_CNS_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to UAV_UGV_CNS_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help UAV_UGV_CNS

% Last Modified by GUIDE v2.5 22-Jun-2010 14:19:40

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @UAV_UGV_CNS_OpeningFcn, ...
                   'gui_OutputFcn',  @UAV_UGV_CNS_OutputFcn, ...
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


% --- Executes just before UAV_UGV_CNS is made visible.
function UAV_UGV_CNS_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to UAV_UGV_CNS (see VARARGIN)

% Choose default command line output for UAV_UGV_CNS
handles.output = hObject;

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes UAV_UGV_CNS wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = UAV_UGV_CNS_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;



function edit1_Callback(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of edit1 as text
%        str2double(get(hObject,'String')) returns contents of edit1 as a double


% --- Executes during object creation, after setting all properties.
function edit1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to edit1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end

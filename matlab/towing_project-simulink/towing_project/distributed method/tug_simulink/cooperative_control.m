function [pd1, pd2] = cooperative_control(p1, p2, v1, v2, p0)
%MSFUNTA is used to setup a cooperative guidance controller block

% INPUT:
%  p1, p2, v1, v2   编队中跟随者的位置和速度，角度单位为弧度
%  p0               编队中领航者的位置

% OUTPUT:
% pd1, pd2          发布给跟随者的目标定位点

setup(block);

end


function setup(block)

% Register number of ports
block.NumInputPorts  = 5;
% block.NumOutputPorts = 7;
block.NumOutputPorts = 2;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
% N = 3;  %quantity of the thrusters
%% 编队的相对位置
deg_to_rad = pi / 180;
deta10 = [397, 237, 31 * deg_to_rad, 0, 0, 0]';
deta20 = [397, -237, 31 * deg_to_rad, 0, 0, 0]';
deta12 = [0, 474, 62 * deg_to_rad, 0, 0, 0]';
deta21 = deta12;



%% p1
block.InputPort(1).Dimensions        = 3;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = false;
block.InputPort(1).SamplingMode = 'Sample';
%% v1
block.InputPort(2).Dimensions        = 3;
block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = false;
block.InputPort(2).SamplingMode = 'Sample';
%% p2
block.InputPort(3).Dimensions        = 3;
block.InputPort(3).DatatypeID  = 0;  % double
block.InputPort(3).Complexity  = 'Real';
block.InputPort(3).DirectFeedthrough = false;
block.InputPort(3).SamplingMode = 'Sample';
%% v2
block.InputPort(4).Dimensions        = 3;
block.InputPort(4).DatatypeID  = 0;  % double
block.InputPort(4).Complexity  = 'Real';
block.InputPort(4).DirectFeedthrough = false;
block.InputPort(4).SamplingMode = 'Sample';
%% p0
block.InputPort(5).Dimensions        = 3;
block.InputPort(5).DatatypeID  = 0;  % double
block.InputPort(5).Complexity  = 'Real';
block.InputPort(5).DirectFeedthrough = false;
block.InputPort(5).SamplingMode = 'Sample';

%%
% Override output port properties
%% pd1
block.InputPort(1).Dimensions        = 3;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = false;
block.InputPort(1).SamplingMode = 'Sample';
%% pd1
block.InputPort(2).Dimensions        = 3;
block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = false;
block.InputPort(2).SamplingMode = 'Sample';


%%
% Register parameters
block.NumDialogPrms     = 0;

% Register sample times
%  [0 offset]            : Continuous sample time
%  [positive_num offset] : Discrete sample time
%
%  [-1, 0]               : Inherited sample time
%  [-2, 0]               : Variable sample time
block.SampleTimes = [1 0];

% Specify the block simStateCompliance. The allowed values are:
%    'UnknownSimState', < The default setting; warn and assume DefaultSimState
%    'DefaultSimState', < Same sim state as a built-in block
%    'HasNoSimState',   < No sim state
%    'CustomSimState',  < Has GetSimState and SetSimState methods
%    'DisallowSimState' < Error out when saving or restoring the model sim state
block.SimStateCompliance = 'DefaultSimState';

%% -----------------------------------------------------------------
%% The MATLAB S-function uses an internal registry for all
%% block methods. You should register all relevant methods
%% (optional and required) as illustrated below. You may choose
%% any suitable name for the methods and implement these methods
%% as local functions within the same file. See comments
%% provided for each function for more information.
%% -----------------------------------------------------------------

block.RegBlockMethod('PostPropagationSetup',    @DoPostPropSetup);
block.RegBlockMethod('InitializeConditions', @InitializeConditions);
% block.RegBlockMethod('Start', @Start);
block.RegBlockMethod('Outputs', @Outputs);     % Required
block.RegBlockMethod('Update', @Update);
% block.RegBlockMethod('Derivatives', @Derivatives);
% block.RegBlockMethod('Terminate', @Terminate); % Required
block.RegBlockMethod('SetInputPortSamplingMode',@SetInputPortSamplingMode);
block.RegBlockMethod('SetInputPortDimensions', @SetInpPortDims);

end %setup

%%
function SetInputPortSamplingMode(block, idx, fd)
block.InputPort(idx).SamplingMode = fd;
block.OutputPort(idx).SamplingMode = fd;

end
%%
function SetInpPortDims(block, idx, di)
block.InputPort(idx).Dimensions = di;
end

%% 工作区设置（可设置一些中间变量）
function DoPostPropSetup(block)
N = 3;  %quantity of the thrusters
% block.NumDworks = 11;
block.NumDworks = 10;
  
  block.Dwork(1).Name            = 'f';
  block.Dwork(1).Dimensions      = N;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;

end

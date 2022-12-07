function [tau, s] = BacksteppingController(block)
%MSFUNTA is used to setup a thrust-allocation block
% INPUT:
%  
%
% OUTPUT:
%  

setup(block);

end

%% Function: setup ===================================================
%% Abstract:
%%   Set up the basic characteristics of the S-function block such as:
%%   - Input ports
%%   - Output ports
%%   - Dialog parameters
%%   - Options
%%
%%   Required         : Yes
%%   C-Mex counterpart: mdlInitializeSizes
%%
%%

function setup(block)

% Register number of ports
block.NumInputPorts  = 4;
% block.NumOutputPorts = 7;
block.NumOutputPorts = 4;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
N = 4;  %quantity of the thrusters
%% pd
block.InputPort(1).Dimensions        = 3;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = false;
block.InputPort(1).SamplingMode = 'Sample';
%%
% Override output port properties
%% dot_pd
block.OutputPort(1).Dimensions       = 3;
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';
%% p
block.OutputPort(2).Dimensions       = 3;
block.OutputPort(2).DatatypeID  = 0; % double
block.OutputPort(2).Complexity  = 'Real';
block.OutputPort(2).SamplingMode = 'Sample';
%% v
block.OutputPort(3).Dimensions       = 3;
block.OutputPort(3).DatatypeID  = 0; % double
block.OutputPort(3).Complexity  = 'Real';
block.OutputPort(3).SamplingMode = 'Sample';

%%
% Register parameters
block.NumDialogPrms     = 3;

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
%%
%% PostPropagationSetup:
%%   Functionality    : Setup work areas and state variables. Can
%%                      also register run-time methods here
%%   Required         : No
%%   C-Mex counterpart: mdlSetWorkWidths
%%
function DoPostPropSetup(block)
N = 3;  %quantity of the thrusters
% block.NumDworks = 11;
block.NumDworks = 3;
  
  block.Dwork(1).Name            = 'int_s';
  block.Dwork(1).Dimensions      = 3;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'tau';
  block.Dwork(2).Dimensions      = 3;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name            = 's';
  block.Dwork(3).Dimensions      = 3;
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;



end
%%
%% InitializeConditions:
%%   Functionality    : Called at the start of simulation and if it is 
%%                      present in an enabled subsystem configured to reset 
%%                      states, it will be called when the enabled subsystem
%%                      restarts execution to reset the states.
%%   Required         : No
%%   C-MEX counterpart: mdlInitializeConditions
%%
function InitializeConditions(block)

  %% Initialize Dwork
  block.Dwork(1).Data = zeros(3, 1);
%   block.Dwork(1).Data = block.DialogPrm(1).Data;
%   block.Dwork(3).Data = block.DialogPrm(2).Data;
%   block.Dwork(5).Data = thrusters_configuration(block.DialogPrm(2).Data...
%       *pi/180)*block.DialogPrm(1).Data;
  %%
    

end %InitializeConditions

%%
%% Start:
%%   Functionality    : Called once at start of model execution. If you
%%                      have states that should be initialized once, this 
%%                      is the place to do it.
%%   Required         : No
%%   C-MEX counterpart: mdlStart
%%
% function Start(block)
% 
% block.Dwork(1).Data = 0;

%endfunction

%%
%% Outputs:
%%   Functionality    : Called to generate block outputs in
%%                      simulation step
%%   Required         : Yes
%%   C-MEX counterpart: mdlOutputs
%%
function Outputs(block)

block.OutputPort(1).Data = block.Dwork(2).Data;
block.OutputPort(2).Data = block.Dwork(3).Data;

% block.OutputPort(7).Data = block.Dwork(11).Data;
end %Outputs

%%
%% Update:
%%   Functionality    : Called to update discrete states
%%                      during simulation step
%%   Required         : No
%%   C-MEX counterpart: mdlUpdate
%%
function Update(block)

%% Thrust allocation
% [block.Dwork(1).Data,block.Dwork(2).Data,block.Dwork(3).Data,block.Dwork(4).Data,...
%  block.Dwork(5).Data,block.Dwork(6).Data,block.Dwork(11).Data] = Thrust_Allocation(...
%  block.Dwork(1).Data,block.Dwork(3).Data,block.InputPort(1).Data);
    p = block.InputPort(3);
    v = block.InputPort(4);
    pd = block.InputPort(1);
    dot_pd = block.InputPort(2);
    fai = p(3);
    R = RotationMatrix(fai);
    MM = M1 + Ma1;
    C = [0, 0, -MM(2, 2) * v(2) - MM(2, 3);
        0, 0, MM(1, 1) * v(1);
        MM(2) * v(2) + MM(2, 3) * v(3), -MM(1, 1) * v(1), 0];
    S = [0, -v(3), 0; v(3), 0, 0; 0, 0, 0];
    Mp = R * MM * R';
    Cp = R * (C - MM * R' * R * S) * R';
    Dp = R * D1 * R';
    ep = p - pd;
    dot_pr = dot_pd - gama * ep;
    
    
    
    
[block.Dwork(1).Data,block.Dwork(2).Data,block.Dwork(3).Data,block.Dwork(4).Data,...
 block.Dwork(5).Data,block.Dwork(6).Data] = Thrust_Allocation(...
 block.Dwork(1).Data, block.Dwork(3).Data, block.InputPort(1).Data);




end

%%
%% Derivatives:
%%   Functionality    : Called to update derivatives of
%%                      continuous states during simulation step
%%   Required         : No
%%   C-MEX counterpart: mdlDerivatives
%%
% function Derivatives(block)

%end Derivatives

%%
%% Terminate:
%%   Functionality    : Called at the end of simulation for cleanup
%%   Required         : Yes
%%   C-MEX counterpart: mdlTerminate
%%
% function Terminate(block)
% 
% end Terminate

%% 旋转矩阵
function R = RotationMatrix(fai)
    R = [cos(fai), -sin(fai), 0;
        sin(fai), cos(fai),0;
        0, 0, 1];
end




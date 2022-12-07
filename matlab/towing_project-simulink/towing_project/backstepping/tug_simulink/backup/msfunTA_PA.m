function msfunTA_PA(block)
%MSFUNTA is used to setup a thrust-allocation block
% INPUT:
%  tau              本步所要求的推力向量 (KN;KN;KN*m)
%
% OUTPUT:
%  f                分配后的各推进器推力值向量 (KN)
%  df               相对于上一步的推力值变化 (KN)
%  a                分配后的各推进器转角向量 (degree)
%  da               相对于上一步的转角变化 (degree)
%  tau_r            分配后实际推力向量 (KN;KN;KN*m)
%  dtau             实际推力与要求推力的差值

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
block.NumInputPorts  = 1;
% block.NumOutputPorts = 7;
block.NumOutputPorts = 6;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
N = 3;  %quantity of the thrusters
%% tau
block.InputPort(1).Dimensions        = 3;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = false;
block.InputPort(1).SamplingMode = 'Sample';
%%
% Override output port properties
%% f
block.OutputPort(1).Dimensions       = N;
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';
%% df
block.OutputPort(2).Dimensions       = N;
block.OutputPort(2).DatatypeID  = 0; % double
block.OutputPort(2).Complexity  = 'Real';
block.OutputPort(2).SamplingMode = 'Sample';
%% a
block.OutputPort(3).Dimensions       = N;
block.OutputPort(3).DatatypeID  = 0; % double
block.OutputPort(3).Complexity  = 'Real';
block.OutputPort(3).SamplingMode = 'Sample';
%% da
block.OutputPort(4).Dimensions       = N;
block.OutputPort(4).DatatypeID  = 0; % double
block.OutputPort(4).Complexity  = 'Real';
block.OutputPort(4).SamplingMode = 'Sample';
%% tau_r
block.OutputPort(5).Dimensions       = 3;
block.OutputPort(5).DatatypeID  = 0; % double
block.OutputPort(5).Complexity  = 'Real';
block.OutputPort(5).SamplingMode = 'Sample';
%% dtau
block.OutputPort(6).Dimensions       = 3;
block.OutputPort(6).DatatypeID  = 0; % double
block.OutputPort(6).Complexity  = 'Real';
block.OutputPort(6).SamplingMode = 'Sample';
%% tau_r6
% block.OutputPort(7).Dimensions       = 6;
% block.OutputPort(7).DatatypeID  = 0; % double
% block.OutputPort(7).Complexity  = 'Real';
% block.OutputPort(7).SamplingMode = 'Sample';
%%
% Register parameters
block.NumDialogPrms     = 4;

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
block.NumDworks = 10;
  
  block.Dwork(1).Name            = 'f';
  block.Dwork(1).Dimensions      = N;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;

  block.Dwork(2).Name            = 'df';
  block.Dwork(2).Dimensions      = N;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name            = 'a';
  block.Dwork(3).Dimensions      = N;
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;
  
  block.Dwork(4).Name            = 'da';
  block.Dwork(4).Dimensions      = N;
  block.Dwork(4).DatatypeID      = 0;      % double
  block.Dwork(4).Complexity      = 'Real'; % real
  block.Dwork(4).UsedAsDiscState = true;
  
  block.Dwork(5).Name            = 'tau_r';
  block.Dwork(5).Dimensions      = 3;
  block.Dwork(5).DatatypeID      = 0;      % double
  block.Dwork(5).Complexity      = 'Real'; % real
  block.Dwork(5).UsedAsDiscState = true;
  
  block.Dwork(6).Name            = 'dtau';
  block.Dwork(6).Dimensions      = 3;
  block.Dwork(6).DatatypeID      = 0;      % double
  block.Dwork(6).Complexity      = 'Real'; % real
  block.Dwork(6).UsedAsDiscState = true;
  
  block.Dwork(7).Name            = 'azimuth_line';
  block.Dwork(7).Dimensions      = N;
  block.Dwork(7).DatatypeID      = 0;      % double
  block.Dwork(7).Complexity      = 'Real'; % real
  block.Dwork(7).UsedAsDiscState = true;
  
  block.Dwork(8).Name            = 't_thrust_line';
  block.Dwork(8).Dimensions      = 1;
  block.Dwork(8).DatatypeID      = 0;      % double
  block.Dwork(8).Complexity      = 'Real'; % real
  block.Dwork(8).UsedAsDiscState = true;
  
  block.Dwork(9).Name            = 't_moment_line';
  block.Dwork(9).Dimensions      = 1;
  block.Dwork(9).DatatypeID      = 0;      % double
  block.Dwork(9).Complexity      = 'Real'; % real
  block.Dwork(9).UsedAsDiscState = true;
  
  block.Dwork(10).Name            = 'time';
  block.Dwork(10).Dimensions      = 1;
  block.Dwork(10).DatatypeID      = 0;      % double
  block.Dwork(10).Complexity      = 'Real'; % real
  block.Dwork(10).UsedAsDiscState = true;
  
%   block.Dwork(11).Name            = 'tau_r6';
%   block.Dwork(11).Dimensions      = 6;
%   block.Dwork(11).DatatypeID      = 0;      % double
%   block.Dwork(11).Complexity      = 'Real'; % real
%   block.Dwork(11).UsedAsDiscState = true;

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
  block.Dwork(1).Data = block.DialogPrm(1).Data.*block.DialogPrm(4).Data;
  block.Dwork(3).Data = block.DialogPrm(2).Data;
  block.Dwork(5).Data = thrusters_configuration(block.DialogPrm(2).Data...
      *pi/180)*block.DialogPrm(1).Data;
  %%
    function T = thrusters_configuration(a)
        % 推进的坐标值
        Nt = length(a);
        L = [5.02 0; 11.956 2.7; 11.956 -2.7];     % 需按实际设置
        a1 = a(1); a2 = a(2); a3 = a(3);
        T = zeros(Nt, 3);
        for i = 1 : Nt
            T(1, i) = cos(a(i));
            T(2, i) = sin(a(i));
            T(1, i) = L(i, 1) * sin(a(i)) - L(i, 2) * cos(a(i));
        end
        
    end
 

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

block.OutputPort(1).Data = block.Dwork(1).Data;
block.OutputPort(2).Data = block.Dwork(2).Data;
block.OutputPort(3).Data = block.Dwork(3).Data;
block.OutputPort(4).Data = block.Dwork(4).Data;
block.OutputPort(5).Data = block.Dwork(5).Data;
block.OutputPort(6).Data = block.Dwork(6).Data;
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
[block.Dwork(1).Data,block.Dwork(2).Data,block.Dwork(3).Data,block.Dwork(4).Data,...
 block.Dwork(5).Data,block.Dwork(6).Data] = Thrust_Allocation(...
 block.Dwork(1).Data,block.Dwork(3).Data,block.InputPort(1).Data,...
 block.DialogPrm(4).Data);


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

%%
    function T = thrusters_configuration(a)
        % 推进的坐标值
        Nt = length(a);
        L = [-5.02 0; 11.956 2.7; 11.956 -2.7];     % 需按实际设置
        a1 = a(1); a2 = a(2); a3 = a(3);
        T = zeros(Nt, 3);
        for i = 1 : Nt
            T(1, i) = cos(a(i));
            T(2, i) = sin(a(i));
            T(1, i) = L(i, 1) * sin(a(i)) - L(i, 2) * cos(a(i));
        end
        
    end
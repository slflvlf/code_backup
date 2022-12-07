function sfun_Controller(block)
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



function setup(block)

% Register number of ports
block.NumInputPorts  = 4;
% block.NumOutputPorts = 7;
block.NumOutputPorts = 1;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
%% Un
block.InputPort(1).Dimensions        = 1;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = false;
block.InputPort(1).SamplingMode = 'Sample';
%% Xn
block.InputPort(2).Dimensions        = 1;
block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = false;
block.InputPort(2).SamplingMode = 'Sample';
%% pose
block.InputPort(3).Dimensions        = 3;
block.InputPort(3).DatatypeID  = 0;  % double
block.InputPort(3).Complexity  = 'Real';
block.InputPort(3).DirectFeedthrough = false;
block.InputPort(3).SamplingMode = 'Sample';
%% vel
block.InputPort(4).Dimensions        = 3;
block.InputPort(4).DatatypeID  = 0;  % double
block.InputPort(4).Complexity  = 'Real';
block.InputPort(4).DirectFeedthrough = false;
block.InputPort(4).SamplingMode = 'Sample';

%%
% Override output port properties
%% tau
block.OutputPort(1).Dimensions       = 3;
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';


%%
% Register parameters
block.NumDialogPrms     = 4;
%离散步长
block.SampleTimes = [1 0];
% 默认就好
block.SimStateCompliance = 'DefaultSimState';


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

block.NumDworks = 12;
% block.NumDworks = 2;
  
  block.Dwork(1).Name            = 'Un_hat';
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;

  block.Dwork(2).Name            = 'dot_Un_hat';
  block.Dwork(2).Dimensions      = 1;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;

  block.Dwork(3).Name            = 'Xn_hat';
  block.Dwork(3).Dimensions      = 1;
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;

  block.Dwork(4).Name            = 'dot_Xn_hat';
  block.Dwork(4).Dimensions      = 1;
  block.Dwork(4).DatatypeID      = 0;      % double
  block.Dwork(4).Complexity      = 'Real'; % real
  block.Dwork(4).UsedAsDiscState = true;
  

end

%% InitializeConditions:

function InitializeConditions(block)

  %% Initialize Dwork
  block.Dwork(1).Data = block.DialogPrm(1).Data;
  block.Dwork(2).Data = block.DialogPrm(2).Data;


end %InitializeConditions

%% 输出
function Outputs(block)
    f0 = block.Dwork(1).Data;
    a0 = block.Dwork(2).Data;
    tau = block.InputPort(1).Data;
    [f,df,a,da,tau_r,dtau] = Thrust_Allocation(f0,a0,tau);
    block.Dwork(1).Data = f;
    block.Dwork(2).Data = a;
    
    
    
    block.OutputPort(1).Data = f;
    block.OutputPort(2).Data = df;
    block.OutputPort(3).Data = a;
    block.OutputPort(4).Data = da;
    block.OutputPort(5).Data = tau_r;
    block.OutputPort(6).Data = dtau;
% block.OutputPort(7).Data = block.Dwork(11).Data;
end %Outputs


%% 更新
function Update(block)
    
    

end




function sfun_tau_env_tug(block)


setup(block);

end



function setup(block)

% Register number of ports
block.NumInputPorts  = 2;
% block.NumOutputPorts = 7;
block.NumOutputPorts = 4;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
%% p
block.InputPort(1).Dimensions        = 3;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = false;
block.InputPort(1).SamplingMode = 'Sample';
%% time
block.InputPort(2).Dimensions        = 1;
block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = false;
block.InputPort(2).SamplingMode = 'Sample';
%%
% Override output port properties
%% tau_wind
block.OutputPort(1).Dimensions       = 3;
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';
%% tau_current
block.OutputPort(2).Dimensions       = 3;
block.OutputPort(2).DatatypeID  = 0; % double
block.OutputPort(2).Complexity  = 'Real';
block.OutputPort(2).SamplingMode = 'Sample';
%% tau_wave1
block.OutputPort(3).Dimensions       = 3;
block.OutputPort(3).DatatypeID  = 0; % double
block.OutputPort(3).Complexity  = 'Real';
block.OutputPort(3).SamplingMode = 'Sample';
%% tau_wave2
block.OutputPort(4).Dimensions       = 3;
block.OutputPort(4).DatatypeID  = 0; % double
block.OutputPort(4).Complexity  = 'Real';
block.OutputPort(4).SamplingMode = 'Sample';


%%
% Register parameters
block.NumDialogPrms     = 7;
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

%     % block.NumDworks = 11;
%     block.NumDworks = 2;
%   
%   block.Dwork(1).Name            = 'f';
%   block.Dwork(1).Dimensions      = N;
%   block.Dwork(1).DatatypeID      = 0;      % double
%   block.Dwork(1).Complexity      = 'Real'; % real
%   block.Dwork(1).UsedAsDiscState = true;
% 
%   block.Dwork(2).Name            = 'a';
%   block.Dwork(2).Dimensions      = N;
%   block.Dwork(2).DatatypeID      = 0;      % double
%   block.Dwork(2).Complexity      = 'Real'; % real
%   block.Dwork(2).UsedAsDiscState = true;
  

end

%% InitializeConditions:

function InitializeConditions(block)
    
    


%   %% Initialize Dwork
%   block.Dwork(1).Data = block.DialogPrm(1).Data;
%   block.Dwork(2).Data = block.DialogPrm(2).Data;


end %InitializeConditions

%% 输出
function Outputs(block)

       % 环境力参数设置 
    wind_param = struct('wind_speed', block.DialogPrm(1).Data, 'wind_angle',  block.DialogPrm(2).Data/180*pi);
    current_param = struct('current_speed', block.DialogPrm(3).Data, 'current_angle', block.DialogPrm(4).Data/180*pi);
    wave_param = struct('Hs', block.DialogPrm(5).Data, 'Tp', block.DialogPrm(6).Data, 'gamma', 3.3, 'env_dir', block.DialogPrm(7).Data/180*pi, 'nfr', 100);


    p = block.InputPort(1).Data;
    current_time = block.InputPort(2).Data;
    
    [tau_wind_tug, tau_current_tug, tau_wave1_tug, tau_wave2_tug] = tau_env_tug_interface(p, current_time, wind_param,... 
            current_param, wave_param);
    
    block.OutputPort(1).Data = tau_wind_tug;
    block.OutputPort(2).Data = tau_current_tug;
    block.OutputPort(3).Data = tau_wave1_tug;
    block.OutputPort(4).Data = tau_wave2_tug;

end %Outputs


%% 更新
function Update(block)
    
    

end


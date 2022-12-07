function msfun_backstepping_controller(block)
%MSFUNTA is used to setup a backstepping controller

% INPUT:
%  pd               本步所要求的轨迹
%  dot_pd           本步所要求的轨迹速度
%  ddot_pd          本步所要求的轨迹速度
%  p                当前位置
%  v                当前速度

% OUTPUT:
%  f                控制力
%  s                输出的参数（协同的时候用）



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
block.NumInputPorts  = 5;
block.NumOutputPorts = 2;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;


%% pd
block.InputPort(1).Dimensions        = 3;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = false;
block.InputPort(1).SamplingMode = 'Sample';

%% dotpd
block.InputPort(2).Dimensions        = 3;
block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = false;
block.InputPort(2).SamplingMode = 'Sample';

%% ddpd
block.InputPort(3).Dimensions        = 3;
block.InputPort(3).DatatypeID  = 0;  % double
block.InputPort(3).Complexity  = 'Real';
block.InputPort(3).DirectFeedthrough = false;
block.InputPort(3).SamplingMode = 'Sample';

%% p
block.InputPort(4).Dimensions        = 3;
block.InputPort(4).DatatypeID  = 0;  % double
block.InputPort(4).Complexity  = 'Real';
block.InputPort(4).DirectFeedthrough = false;
block.InputPort(4).SamplingMode = 'Sample';

%% v
block.InputPort(5).Dimensions        = 3;
block.InputPort(5).DatatypeID  = 0;  % double
block.InputPort(5).Complexity  = 'Real';
block.InputPort(5).DirectFeedthrough = false;
block.InputPort(5).SamplingMode = 'Sample';

%%
% Override output port properties
%% f
block.OutputPort(1).Dimensions       = 3;
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';
%% s
block.OutputPort(2).Dimensions       = 3;
block.OutputPort(2).DatatypeID  = 0; % double
block.OutputPort(2).Complexity  = 'Real';
block.OutputPort(2).SamplingMode = 'Sample';

%%
% Register parameters
block.NumDialogPrms  = 3;
block.SampleTimes = [1 0];
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

%% 工作区
function DoPostPropSetup(block)
N = 3;  %quantity of the thrusters
% block.NumDworks = 11;
block.NumDworks = 3;
  
  block.Dwork(1).Name            = 'f';
  block.Dwork(1).Dimensions      = 3;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 's';
  block.Dwork(2).Dimensions      = 3;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name            = 'ei';
  block.Dwork(3).Dimensions      = 3;
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;
  
  

end

%% 初始换函数
function InitializeConditions(block)

  %% Initialize Dwork
  block.Dwork(3).Data = [0, 0, 0]';
 

end %InitializeConditions


%% output function
function Outputs(block)

block.OutputPort(1).Data = block.Dwork(1).Data;
block.OutputPort(2).Data = block.Dwork(2).Data;

end %Outputs


%% 更新（计算区域）
function Update(block)

    K = block.DialogPrm(1).Data;
    Ki = block.DialogPrm(2).Data;
    Gama = block.DialogPrm(3).Data;

    pd = block.InputPort(1).Data;
    dot_pd = block.InputPort(2).Data;
    ddot_pd = block.InputPort(3).Data;
    p =  block.InputPort(4).Data;
    v =  block.InputPort(5).Data;
    
    R = rotate_matrix(p);
    dot_R = dot_rotate_matrix(p, v);
    
    p_error = p - pd;
    dot_pr = dot_pd - Gama * p_error;
    ddot_pr = ddot_pd - Gama * (R * v - dot_pd);
    s = R * v -dot_pr;
    
    M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
    Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
    MM1 = M1 + Ma1;
    D1 = [MM1(1,1)/100, 0, 0; 0, MM1(2,2)/40, 0; 0, 0, MM1(3,3)/20];
    
    block.Dwork(3).data = block.Dwork(3).data + s * 1; %(sample time: 1)
    ei = block.Dwork(3).data;
    C1 = C_matrix(MM1, v);
    Mp1 = R * MM1 * R';
    Dp1 = R * D1 * R';
    Cp1 = R * (C1 - M1 * R' * dot_R) * R';
    
    tau = R' * (Mp1 * ddot_pr + Cp1 * dot_pr + Dp1 * dot_pr - K * s - Ki * ei);
    
    block.Dwork(1).data = tau;
    block.Dwork(2).data = s;

end



function R = rotate_matrix(p)
    fai = p(3);
    R = [cos(fai),  -sin(fai),  0;
         sin(fai),  cos(fai),   0;
         0,          0,         1];
     
end

function dotR = dot_rotate_matrix(p, v)
    fai = p(3);
    r = v(3);
    dotR = [-sin(fai)*r, -cos(fai)*r, 0;
           cos(fai)*r, -sin(fai)*r, 0;
           0, 0, 0];
       
end
    
function C = C_matrix(M, v)
    C = [0, 0, -M(2, 2) * v(2) - M(2, 3) * v(3);
        0, 0, M(1, 1) * v(1);
        M(2, 2) * v(2) + M(3, 2) * v(3), -M(1, 1) * v(1), 0];
end

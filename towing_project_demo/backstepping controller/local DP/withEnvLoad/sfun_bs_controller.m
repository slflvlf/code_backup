function sfun_bs_controller(block)
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
block.NumInputPorts  = 5;
% block.NumOutputPorts = 7;
block.NumOutputPorts = 2;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties
N = 3;  %quantity of the thrusters
%% p
block.InputPort(1).Dimensions        = 3;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = false;
block.InputPort(1).SamplingMode = 'Sample';
%% v
block.InputPort(2).Dimensions        = 3;
block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = false;
block.InputPort(2).SamplingMode = 'Sample';
%% pd
block.InputPort(3).Dimensions        = 3;
block.InputPort(3).DatatypeID  = 0;  % double
block.InputPort(3).Complexity  = 'Real';
block.InputPort(3).DirectFeedthrough = false;
block.InputPort(3).SamplingMode = 'Sample';
%% dot_pd
block.InputPort(4).Dimensions        = 3;
block.InputPort(4).DatatypeID  = 0;  % double
block.InputPort(4).Complexity  = 'Real';
block.InputPort(4).DirectFeedthrough = false;
block.InputPort(4).SamplingMode = 'Sample';
%% dot_pd
block.InputPort(5).Dimensions        = 3;
block.InputPort(5).DatatypeID  = 0;  % double
block.InputPort(5).Complexity  = 'Real';
block.InputPort(5).DirectFeedthrough = false;
block.InputPort(5).SamplingMode = 'Sample';
%%
% Override output port properties
%% tau
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
block.NumDialogPrms     = 3;
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
N = 3;  %quantity of the thrusters
% block.NumDworks = 11;
block.NumDworks = 1;
  %% ei
  block.Dwork(1).Name            = 'ei';
  block.Dwork(1).Dimensions      = 3;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;


  

end

%% InitializeConditions:

function InitializeConditions(block)

  %% Initialize Dwork
  block.Dwork(1).Data = zeros(3, 1);

end %InitializeConditions

%% 输出
function Outputs(block)
    ei = block.Dwork(1).Data;
    
    p = block.InputPort(1).Data;
    v = block.InputPort(2).Data;
    pd = block.InputPort(3).Data;
    dot_pd = block.InputPort(4).Data;
    ddot_pd = block.InputPort(5).Data;
    
    Kd = diag(block.DialogPrm(1).Data);
    Ki = diag(block.DialogPrm(2).Data);
    Gama = diag(block.DialogPrm(3).Data);
    
    R = rotate_matrix(p);
    dot_R = dot_rotate_matrix(p, v);
    
    p_error = p - pd;
    dot_pr = dot_pd - Gama * p_error;
    ddot_pr = ddot_pd - Gama * (R * v - dot_pd);
    s = R * v -dot_pr;

    ei = ei + p_error * 1; %误差累计，用于积分用, 1是步长
    
    imax = [100, 100, 30];
    for i = 1 : 3
        if ei(i) > imax(i)
            ei(i) = imax(i);
        end
        if ei(i) < -imax(i)
            ei(i) = -imax(i);
        end
    end
        
    M1 = diag([6.25e5, 6.25e5, 6.25e5*7.5^2]);
    Ma1 = [7.58e4, 0, 0; 0, 3.69e5 -1.4e5; 0, -1.4e5, 8.77e6];
    MM1 = M1 + Ma1;
    D1 = [MM1(1,1)/100, 0, 0; 0, MM1(2,2)/40, 0; 0, 0, MM1(3,3)/20];
    
    C1 = C_matrix(MM1, v);
    Mp1 = R * MM1 * R';
    Dp1 = R * D1 * R';
    Cp1 = R * (C1 - MM1 * R' * dot_R) * R';
    
    tau = R' * (Mp1 * ddot_pr + Cp1 * dot_pr + Dp1 * dot_pr - Kd * s - Ki * ei);
    
    tau = tau/1e3; %输出为KN

    
    block.OutputPort(1).Data = tau;
    block.OutputPort(2).Data = s;

	block.Dwork(1).Data = ei;
end %Outputs


%% 更新
function Update(block)
    
    

end

%% 小函数
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

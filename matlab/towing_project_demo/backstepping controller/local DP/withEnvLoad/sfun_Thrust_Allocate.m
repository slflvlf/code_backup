function sfun_Thrust_Allocate(block)
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

%%
% Register parameters
block.NumDialogPrms     = 2;
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
block.NumDworks = 2;
  
  block.Dwork(1).Name            = 'f';
  block.Dwork(1).Dimensions      = N;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;

  block.Dwork(2).Name            = 'a';
  block.Dwork(2).Dimensions      = N;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  

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



function[f,df,a,da,tau_r,dtau] = Thrust_Allocation(f0,a0,tau)
% function[f,df,a,da,tau_r,dtau,tau_r6] = Thrust_Allocation(f0,a0,tau)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Thrust_Allcation is used to allocate commanded thrust among the azimuth
% thrusters (only)
%
% INPUT:
%  f0               上一步的推力值向量 (KN)
%  a0               上一步的推进器转角向量 (degree)
%  tau              本步所要求的推力向量 (KN;KN;KN*m)
%
% OUTPUT:
%  f                分配后的各推进器推力值向量 (KN)
%  df               相对于上一步的推力值变化 (KN)
%  a                分配后的各推进器转角向量 (degree)
%  da               相对于上一步的转角变化 (degree)
%  tau_r            分配后实际推力向量 (KN;KN;KN*m)
%  dtau             实际推力与要求推力的差值
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%% 常量 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 将角度转化为弧度
d2r = pi/180;
% 上一步推进器转角化为弧度
a0 = a0 * d2r;
% 全回转推进器个数
N = 3;
% 松弛因子的权矩阵
Q = 1e12*diag([30 30 1]);
% 全回转速度的权矩阵
Omega = 10*eye(N);
% 推进器全回转速度区间(rad/sample_time)
da = [-25 25]*d2r;                                % 需按实际设置
% 单个推进器推力的变化区间(KN)
Frange = [0 800];                              % 需按实际设置
% 推力器使用情况
% delta_u = [0; 1; 1; 1; 0; 1; 1; 1;];            % 设置推力器失效情况
% 推进器推力的变化速度区间(KN/sample_time)
dF = [-150 150];                                  % 需按实际设置
% 推进的坐标值
L = [-5.02 0; 11.956 2.7; 11.956 -2.7];
% config = [1.4 190; 0.87 205; 0.57 275; 1.7 17; 0.87 125; 0.57 94];
% L =zeros(N, 2);
% for i = 1 : N
%     L(i, 1) = config(i, 1) * cos(config(i, 2)/180*pi);
%     L(i, 2) = config(i, 1) * sin(config(i, 2)/180*pi);
% end

% L = [76 0; 51 26; -51 26; -76 0; -51 -26; 51 -26];     % 需按实际设置
% Semi 708
% L = [15.7 35.5 -21.5; 47.02 24.58 -21.5; 47.02 -24.58 -21.5; 15.7 -35.5 -21.5; ...
%      -15.7 -35.5 -21.5;-47.02 -24.58 -21.5; -47.02 24.58 -21.5; -15.7 35.5 -21.5;];
% Semi 807
% L = [15.7 24.58; 47.02 35.5; 47.02 -35.5; 15.7 -24.58; -15.7 -24.58; -47.02 -35.5; -47.02 35.5; -15.7 24.58;];     % 需按实际设置 Semi807
% Semi 981
% L = [47.02 35.5; 47.02 24.58; 47.02 -24.58; 47.02 -35.5; -47.02 -35.5; -47.02 -24.58; -47.02 24.58; -47.02 35.5;];     % 需按实际设置

% % 目标函数奇异位置惩罚项分子常数
% pp = 1000;
% % 目标函数奇异位置惩罚项分母中的常数
% ee = 1;

% 禁止角设置后的可行区间，1、4推进器具有两个区间
% angle_sector1o = [-15 15;75 285]*d2r;
% angle_sector2o = [165 425]*d2r;
% angle_sector3o = [115 375]*d2r;
% angle_sector4o = [165 195;-105 105]*d2r;
% angle_sector5o = [-15 245]*d2r;
% angle_sector6o = [-65 195]*d2r;
angle_sector1o = [-180 180]*d2r;
angle_sector2o = [-180 180]*d2r;
angle_sector3o = [-180 180]*d2r;
% angle_sector4o = [-180 180]*d2r;
% angle_sector5o = [-180 180]*d2r;   
% angle_sector6o = [-180 180]*d2r;
% angle_sector7o = [-180 180]*d2r;   
% angle_sector8o = [-180 180]*d2r;
% 本步可行区间，考虑多区间推进器的区间选择，未来可能加入动态禁止角
angle_sector = 1000*ones(N,2);
angle_sector(1,:) = angle_sector1o;
angle_sector(2,:) = angle_sector2o;
angle_sector(3,:) = angle_sector3o;
% angle_sector(4,:) = angle_sector4o;
% angle_sector(5,:) = angle_sector5o;
% angle_sector(6,:) = angle_sector6o;
% angle_sector(7,:) = angle_sector7o;
% angle_sector(8,:) = angle_sector8o;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 推进器全回转速度约束区间
dda = 1000*ones(N,2);     % 初始化
% 首先考虑可行区间与上步角度的限制,并将dda转入[-2pi 2pi],以便与da比较
for i = 1 : N
    dda(i,:) = angle_sector(i,:) - a0(i);
    while(dda(i,2) + 100*eps < 0)
        dda(i,:) = dda(i,:) + 2*pi;
    end
    while(dda(i,1) - 100*eps > 0)
        dda(i,:) = dda(i,:) - 2*pi;
    end
end
% 与全回转推进器每步的最大转速da比较，选择最小的变化区间
for i = 1 : N
    if da(1) > dda(i,1)
        dda(i,1) = da(1);
    end
    if da(2) < dda(i,2)
        dda(i,2) = da(2);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 推进器推力变化区间
F = [ Frange(1) Frange(2)]; % 考虑失效螺旋桨的推力器变化区间
ff = 1e10*ones(N,2);
for i = 1 : N
    ff(i,1) = f0(i) + dF(1);
    if ff(i,1) <F(1)
        ff(i,1) = F(1);
    end
    
    ff(i,2) = f0(i) + dF(2);
    if ff(i,2) > F(2)
        ff(i,2) = F(2);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 其他系数
T =  thrusters_configuration(a0,L);
dTf = Get_Coefficients(f0,a0);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 为二次规划问题准备系数
H = blkdiag(2*eye(N),2*Omega,2*Q);
I = [];
A = [];
b = [];
Aeq = [T,dTf,eye(3)];
beq = tau ;
lb = [ff(:,1);dda(:,1);-1e10*ones(3,1)];
ub = [ff(:,2);dda(:,2); 1e10*ones(3,1)];
x0 = [f0;zeros(N,1);zeros(3,1)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 求解二次规划问题
options = optimset('Algorithm','interior-point-convex','Display','off');
x = quadprog(H,I,A,b,Aeq,beq,lb,ub,x0,options);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 函数输出
f = x(1:N);
df = f - f0;
da = x(N+1:2*N);
a = a0 + da;
da = da / d2r;
a = a / d2r;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 将a转入[-pi pi]区间，以方便画图
for i = 1 : N
    while a(i) > 200
        a(i) = a(i) - 360;
    end
    while a(i) <-200
        a(i) = a(i) + 360;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 实际推力
T =  thrusters_configuration(a*d2r, L);
% T1 = thrusters_configuration6(a*d2r);
tau_r = T*f;
% tau_r6 = T1*f;
dtau = tau_r - tau;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end


%% Bf微分
function dTf = Get_Coefficients(f0,a0)
%
f1 = f0(1); f2 = f0(2); f3 = f0(3); 

a1 = a0(1); a2 = a0(2); a3 = a0(3); 
%
N = length(f0);
% config = [1.4 190; 0.87 205; 0.57 275; 1.7 17; 0.87 125; 0.57 94];
L =zeros(N, 2);
L = [-5.02 0; 11.956 2.7; 11.956 -2.7];
% for i = 1 : N
%     L(i, 1) = config(i, 1) * cos(config(i, 2)/180*pi);
%     L(i, 2) = config(i, 1) * sin(config(i, 2)/180*pi);
% end

dTf = zeros(3, N);
for i = 1 : N
    dTf(1, i) = -f0(i) * sin(a0(i));
    dTf(2, i) = f0(i) * cos(a0(i));
    dTf(3, i) = f0(i) * L(i, 2) * sin(a0(i)) + f0(i) * L(i, 1) * cos(a0(i));
end

end


%% 配置矩阵
function T = thrusters_configuration(a,L)
% function thruster_configuration is used to obtain the configuration
% matrix
N_Col = length(a);

if N_Col ~= length(L)
    error('the length of a and L do not match');
end

T = zeros(3,N_Col);   % initialization

for i = 1 : N_Col
    T(1,i) = cos(a(i));
    T(2,i) = sin(a(i));
    T(3,i) = L(i,1)*sin(a(i)) - L(i,2)*cos(a(i));
end

end




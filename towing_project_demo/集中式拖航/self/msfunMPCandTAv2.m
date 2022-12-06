function msfunMPCandTAv2(block)
%msfunMPCandTAv2 is used to setup a thrust-allocation block
%日志
%9月24日可能找到了之前QP由于上下界相遇而终止的求解的原因 858行upperbound
%9月25日发现block.DialogPrm(7)控制前馈力开关的对话框参数未使用，即之前的模拟都完全没加入前馈力
%9月25日 原程序中da前后代表过两种不同变量 先是单位时间转角变化极限 后是相对于上一步的转角变化 现在将两种变量使用不同变量名做了区分
%v2取消了原来的位置松弛变量s 
% 加入控制器内部参考轨迹(用于定点定位)但没有进行对参考轨迹的可行性分析
%10月8日 新增输出QP求解的完整的输入序列 Dwork(14) 以后修改NumC时务必调整相关代码

% INPUT:
%  eta_ref               
%  eta_est
%  Nv_est
%  feedForwardForces
%  obsSlowVaringEnvForces
%  windCurrentForces
% OUTPUT:
%  f                分配后的各推进器推力值向量 (N)
%  df               相对于上一步的推力值变化 (N)
%  a                分配后的各推进器转角向量 (degree)
%  dalpha           相对于上一步的转角变化 (degree)
%  tau_r            分配后实际推力向量 (N;N;N*m)
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
%%   Required(必需性)         : Yes
%%   C-Mex counterpart: mdlInitializeSizes
%%
function setup(block)

% Register number of ports 登记输入、输出port模块个数
block.NumInputPorts  = 6;
block.NumOutputPorts = 10;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties

%% eta_ref (m;m;rad)
block.InputPort(1).Dimensions        = 3;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = false; %设置该输入是否直接馈入输出Outputs block
block.InputPort(1).SamplingMode = 'Sample';
%% eta_est (m;m;rad)
block.InputPort(2).Dimensions        = 3;
block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = false; %设置该输入是否直接馈入输出Outputs block
block.InputPort(2).SamplingMode = 'Sample';
%% Nv_est (m/s;m/s;rad/s)
block.InputPort(3).Dimensions        = 3;
block.InputPort(3).DatatypeID  = 0;  % double
block.InputPort(3).Complexity  = 'Real';
block.InputPort(3).DirectFeedthrough = false; %设置该输入是否直接馈入输出Outputs block
block.InputPort(3).SamplingMode = 'Sample';
%% feedForwardForces (N;N;N*m)
block.InputPort(4).Dimensions        = 3;
block.InputPort(4).DatatypeID  = 0;  % double
block.InputPort(4).Complexity  = 'Real';
block.InputPort(4).DirectFeedthrough = false; %设置该输入是否直接馈入输出Outputs block
block.InputPort(4).SamplingMode = 'Sample';
%% obsSlowVaringEnvForces (N;N;N*m)
block.InputPort(5).Dimensions        = 3;
block.InputPort(5).DatatypeID  = 0;  % double
block.InputPort(5).Complexity  = 'Real';
block.InputPort(5).DirectFeedthrough = false; %设置该输入是否直接馈入输出Outputs block
block.InputPort(5).SamplingMode = 'Sample';
%% windCurrentForces (N;N;N*m)
block.InputPort(6).Dimensions        = 3;
block.InputPort(6).DatatypeID  = 0;  % double
block.InputPort(6).Complexity  = 'Real';
block.InputPort(6).DirectFeedthrough = false; %设置该输入是否直接馈入输出Outputs block
block.InputPort(6).SamplingMode = 'Sample';

%%
% Override output port properties
N = 8;  %推进器数量
%% f 推力分配后产生的各推进器推力值向量(维度与推进器个数相同)(N)
block.OutputPort(1).Dimensions       = N;
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';
%% df 相对于上一步的推力值变化 (维度与推进器个数相同)(N)
block.OutputPort(2).Dimensions       = N;
block.OutputPort(2).DatatypeID  = 0; % double
block.OutputPort(2).Complexity  = 'Real';
block.OutputPort(2).SamplingMode = 'Sample';
%% a 分配后的各推进器转角向量 (维度与推进器个数相同)(degree)
block.OutputPort(3).Dimensions       = N;
block.OutputPort(3).DatatypeID  = 0; % double
block.OutputPort(3).Complexity  = 'Real';
block.OutputPort(3).SamplingMode = 'Sample';
%% dalpha 相对于上一步的转角变化 (维度与推进器个数相同)(degree)
block.OutputPort(4).Dimensions       = N;
block.OutputPort(4).DatatypeID  = 0; % double
block.OutputPort(4).Complexity  = 'Real';
block.OutputPort(4).SamplingMode = 'Sample';
%% tau_r 分配后实际总推力向量 (N;N;N*m)
block.OutputPort(5).Dimensions       = 3;
block.OutputPort(5).DatatypeID  = 0; % double
block.OutputPort(5).Complexity  = 'Real';
block.OutputPort(5).SamplingMode = 'Sample';
%% dtau 实际推力与要求推力(控制器所生成)的差值(N;N;N*m)
block.OutputPort(6).Dimensions       = 3;
block.OutputPort(6).DatatypeID  = 0; % double
block.OutputPort(6).Complexity  = 'Real';
block.OutputPort(6).SamplingMode = 'Sample';
%% s 在最终预测步距离参考位置的误差的松弛量(m;m;rad)
block.OutputPort(7).Dimensions       = 3;
block.OutputPort(7).DatatypeID  = 0; % double
block.OutputPort(7).Complexity  = 'Real';
block.OutputPort(7).SamplingMode = 'Sample';
%% xFinal_prd 控制器中最后一个预测步Np的系统状态 x(k+Np|k) = [u v r x y psi]
block.OutputPort(8).Dimensions       = 6;
block.OutputPort(8).DatatypeID  = 0; % double
block.OutputPort(8).Complexity  = 'Real';
block.OutputPort(8).SamplingMode = 'Sample';
%% abnormalUtimes QP病态产生的不正常推力次数
block.OutputPort(9).Dimensions       = 1;
block.OutputPort(9).DatatypeID  = 0; % double
block.OutputPort(9).Complexity  = 'Real';
block.OutputPort(9).SamplingMode = 'Sample';
%% ForeceMomentK QP求解的完整的NumC长的控制序列（已转换成Fx Fy Moment）
block.OutputPort(10).Dimensions       = 3*5; %维度为3*NumC
block.OutputPort(10).DatatypeID  = 0; % double
block.OutputPort(10).Complexity  = 'Real';
block.OutputPort(10).SamplingMode = 'Sample';

%%
% Register parameters 在封装界面输入的参数变量个数
block.NumDialogPrms = 12;
% 参数分别是M,Ma,D,Q,R,F,FeedForwardSwitch,f0,a0,u0,wa
%1.M 船舶的质量矩阵
%2.Ma 船舶的附加质量矩阵
%3.D 船舶的阻尼矩阵
%4.Q Position Error Weight Matrix Q 
%5.R Input Weight Matrix R 
%6.F Final Position Error Weight Matrix F 
%7.FeedForwardSwitch
%8.f0 各推进器初始推力组成的列向量(KN) 
%9.a0 各推进器转角组成的向量 (degree)(随船坐标系下)
%10.u0 各推进器的启用情况组成的列向量
%11.wa 是否显示推力和推进器转角的动画
%12.Omega 全回转推进器转角的权矩阵

% Register sample times 登记采样时间，为0.05s
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
%在此处setup函数内注册别的子函数功能才能在后续引入别的子函数功能，这里就没有引入开始和求导等子功能
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
% function SetInpPortDims(block, idx, di)
% block.InputPort(idx).Dimensions = di;
% end
%%
%% PostPropagationSetup:
%%   Functionality    : Setup work areas and state variables. Can
%%                      also register run-time methods here
%%   Required         : No
%%   C-Mex counterpart: mdlSetWorkWidths
%%
function DoPostPropSetup(block)
%% 设置存储在该函数专用工作区的变量，也就是函数内的状态变量，Dwork向量是Simulink分配给模型中每个S函数实例的存储空间块。
%当不同S函数块之间需要通过全局变量或者静态变量进行数据交互时，必须在S函数中使用Dwork向量来进行变量存储。（比如在InitializeConditions函数中定义的变量，在Update函数中是没有的）
%Dwork(1)至Dwork(8)与OutputPort(1)至OutputPort(8)一一对应

N = 8;  %推进器数量
block.NumDworks = 14;
%% f 推力分配后产生的各推进器推力值向量(维度与推进器个数相同)(N)
  block.Dwork(1).Name            = 'f';
  block.Dwork(1).Dimensions      = N;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
%% df 相对于上一步的推力值变化 (维度与推进器个数相同)(N)
  block.Dwork(2).Name            = 'df';
  block.Dwork(2).Dimensions      = N;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
%% a 分配后的各推进器转角向量 (维度与推进器个数相同)(degree)
  block.Dwork(3).Name            = 'a';
  block.Dwork(3).Dimensions      = N;
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;
%% dalpha 相对于上一步的转角变化 (维度与推进器个数相同)(degree) 
  block.Dwork(4).Name            = 'dalpha';
  block.Dwork(4).Dimensions      = N;
  block.Dwork(4).DatatypeID      = 0;      % double
  block.Dwork(4).Complexity      = 'Real'; % real
  block.Dwork(4).UsedAsDiscState = true;
%% tau_r 分配后实际总推力向量 (N;N;N*m)  
  block.Dwork(5).Name            = 'tau_r';
  block.Dwork(5).Dimensions      = 3;
  block.Dwork(5).DatatypeID      = 0;      % double
  block.Dwork(5).Complexity      = 'Real'; % real
  block.Dwork(5).UsedAsDiscState = true;
%% sTau QP中的推力松弛变量相当于实际推力与要求推力(控制器所生成)的差值(N;N;N*m) %当设置了实际的推力松弛变量 才有此行
  block.Dwork(6).Name            = 'dtau';
  block.Dwork(6).Dimensions      = 3;
  block.Dwork(6).DatatypeID      = 0;      % double
  block.Dwork(6).Complexity      = 'Real'; % real
  block.Dwork(6).UsedAsDiscState = true;
%% s  在最终预测步距离参考位置的误差的松弛量(m;m;rad)
  block.Dwork(7).Name            = 's';
  block.Dwork(7).Dimensions      = 3;
  block.Dwork(7).DatatypeID      = 0;      % double
  block.Dwork(7).Complexity      = 'Real'; % real
  block.Dwork(7).UsedAsDiscState = true;
%% azimuth_line 各个推进器推力的线段（维度为何为N，因为存储了N个推进器line函数的对象）
  block.Dwork(8).Name            = 'azimuth_line';
  block.Dwork(8).Dimensions      = N;
  block.Dwork(8).DatatypeID      = 0;      % double
  block.Dwork(8).Complexity      = 'Real'; % real
  block.Dwork(8).UsedAsDiscState = true;
%% t_thrust_line 总推力以及转矩的线段（维度为何为2，因为推力和转矩各需要1个）
  block.Dwork(9).Name            = 't_thrust_line';
  block.Dwork(9).Dimensions      = 2;
  block.Dwork(9).DatatypeID      = 0;      % double
  block.Dwork(9).Complexity      = 'Real'; % real
  block.Dwork(9).UsedAsDiscState = true;
%% time 模拟的时间
  block.Dwork(10).Name            = 'time';
  block.Dwork(10).Dimensions      = 1;
  block.Dwork(10).DatatypeID      = 0;      % double
  block.Dwork(10).Complexity      = 'Real'; % real
  block.Dwork(10).UsedAsDiscState = true;
%% xkplus1_prd 控制器中当前时刻中对下个时间步的预测值 x(k+1|k) = [u v r x y psi]
  block.Dwork(11).Name            = 'xkplus1_prd';
  block.Dwork(11).Dimensions      = 6;
  block.Dwork(11).DatatypeID      = 0;      % double
  block.Dwork(11).Complexity      = 'Real'; % real
  block.Dwork(11).UsedAsDiscState = true;
%% xFinal_prd 控制器中最后一个预测步Np的系统状态 x(k+Np|k) = [u v r x y psi]
  block.Dwork(12).Name            = 'xFinal_prd';
  block.Dwork(12).Dimensions      = 6;
  block.Dwork(12).DatatypeID      = 0;      % double
  block.Dwork(12).Complexity      = 'Real'; % real
  block.Dwork(12).UsedAsDiscState = true;
%% abnormalUtimes QP病态产生的不正常推力次数
  block.Dwork(13).Name            = 'abnormalUtimes';
  block.Dwork(13).Dimensions      = 1;
  block.Dwork(13).DatatypeID      = 0;      % double
  block.Dwork(13).Complexity      = 'Real'; % real
  block.Dwork(13).UsedAsDiscState = true;
  
%% ForeceMomentK QP求解的完整的NumC长的控制序列（已转换成Fx Fy Moment）
  block.Dwork(14).Name            = 'ForeceMomentK';
  block.Dwork(14).Dimensions      = 3*5; %维度为3*NumC
  block.Dwork(14).DatatypeID      = 0;      % double
  block.Dwork(14).Complexity      = 'Real'; % real
  block.Dwork(14).UsedAsDiscState = true;
  
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
%% Initialize Dwork 对函数内的Dwork状态变量进行模拟开始前的初始化
% 和MPC控制器相关的Dwork不用初始化 这里基本都是对推进器初始状态进行设置
  
  block.Dwork(1).Data = 1e3 * block.DialogPrm(8).Data.*block.DialogPrm(10).Data; %Dwork1存储各推进器推力 由于DialogPrm(8)中推力单位为KN，因此乘上1e3换算为N
  block.Dwork(3).Data = block.DialogPrm(9).Data;
  block.Dwork(13).Data(1) = 0;
  
  % 推进器的坐标值
  %L = [72 0; 65 0; -65 0; 55 0; -55 0; -72 20; -72 -20];     % 需按实际设置
  L = [15.7 35.5; 47.02 24.58; 47.02 -24.58; 15.7 -35.5; -15.7 -35.5; ...
             -47.02 -24.58; -47.02 24.58; -15.7 35.5];     % 需按实际设置 此处是8行2列的矩阵，第一列是推进器在随体坐标系下的x坐标值，第二列是y坐标值
   
  block.Dwork(5).Data = thrusters_configuration(block.DialogPrm(9).Data*pi/180,L)*block.Dwork(1).Data;   %推进器配置矩阵B(alpha,L)乘以推力值列向量得到当前分配后总推力向量(N,N,N*m)
  
  
%% plot animation
N = 8;  %quantity of the thrusters
if block.DialogPrm(11).Data == 1

    close(findobj('type','figure','name','Thrust & Azimuth Angle'))
    figure('Name','Thrust & Azimuth Angle','units','normalized','position',[0.1,0.1,0.8,0.8],'color',[0.8 1 0.8]);
    xlim([-150 150]);ylim([-150 150]); %上面的position Name-Value pair决定了figure框的位置和长宽
%   这里xlim ylim决定了使用函数语句绘图的坐标轴
    axis off  %将坐标轴隐藏
    axis equal
    hold on
%% draw the vessel
    rectangle('Position',[-100,40,200,32],'linewidth',2.5,'facecolor','white');
    rectangle('Position',[-100,-72,200,32],'linewidth',2.5,'facecolor','white');
    rectangle('Position',[-72,40,32,32],'linewidth',2.5,'facecolor','white');
    rectangle('Position',[42,40,32,32],'linewidth',2.5,'facecolor','white');
    rectangle('Position',[-72,-72,32,32],'linewidth',2.5,'facecolor','white');
    rectangle('Position',[42,-72,32,32],'linewidth',2.5,'facecolor','white');
    rectangle('Position',[50,-40,12,80],'linewidth',2.5,'facecolor','white');
    rectangle('Position',[-62,-40,12,80],'linewidth',2.5,'facecolor','white');%画出船身
    plot(0,0,'o','markeredgecolor','k','markerfacecolor','k','markersize',8) %画出船中心点
%% draw the azimuth thruster circles
    R = 10;

    center = [30 66; 85 46; 85 -46; 30 -66; -30 -66; -85 -46; -85 46; -30 66;]; %没有使用真实的推进器坐标主要是因为不方便画图
    rectangle('Position',[center(1,1)-R,center(1,2)-R,2*R,2*R],'curvature',[1,1],...
        'linewidth',1.5,'edgecolor','r','facecolor','white');
    %画出圆形也可以用rectangle函数，position的填法和矩形一样，起始点坐标和长度和宽度，
    %'curvature',[1,1]这样才能让所有的角落曲率均为1，这里画出的两个圆形指代全回转推进器
    rectangle('Position',[center(2,1)-R,center(2,2)-R,2*R,2*R],'curvature',[1,1],...
        'linewidth',1.5,'edgecolor','r','facecolor','white');
    rectangle('Position',[center(3,1)-R,center(3,2)-R,2*R,2*R],'curvature',[1,1],...
        'linewidth',1.5,'edgecolor','r','facecolor','white');
    rectangle('Position',[center(4,1)-R,center(4,2)-R,2*R,2*R],'curvature',[1,1],...
        'linewidth',1.5,'edgecolor','r','facecolor','white');
    rectangle('Position',[center(5,1)-R,center(5,2)-R,2*R,2*R],'curvature',[1,1],...
        'linewidth',1.5,'edgecolor','r','facecolor','white');
    rectangle('Position',[center(6,1)-R,center(6,2)-R,2*R,2*R],'curvature',[1,1],...
        'linewidth',1.5,'edgecolor','r','facecolor','white');
    rectangle('Position',[center(7,1)-R,center(7,2)-R,2*R,2*R],'curvature',[1,1],...
        'linewidth',1.5,'edgecolor','r','facecolor','white');
    rectangle('Position',[center(8,1)-R,center(8,2)-R,2*R,2*R],'curvature',[1,1],...
        'linewidth',1.5,'edgecolor','r','facecolor','white');        

    text(35,81,'No.1','fontname','Time New Rome','fontsize',12,'fontangle','oblique');    %fontangle oblique是指字体为斜体，此处打字给推进器进行编号
    text(102,46,'No.2','fontname','Time New Rome','fontsize',12,'fontangle','oblique');
    text(102,-46,'No.3','fontname','Time New Rome','fontsize',12,'fontangle','oblique');
    text(35,-81,'No.4','fontname','Time New Rome','fontsize',12,'fontangle','oblique');
    text(-55,-81,'No.5','fontname','Time New Rome','fontsize',12,'fontangle','oblique');
    text(-123,-46,'No.6','fontname','Time New Rome','fontsize',12,'fontangle','oblique');
    text(-123,46,'No.7','fontname','Time New Rome','fontsize',12,'fontangle','oblique');
    text(-55,81,'No.8','fontname','Time New Rome','fontsize',12,'fontangle','oblique');
    
    plot(center(1,1),center(1,2),'o','markeredgecolor','k','markerfacecolor','k','markersize',2)
    plot(center(2,1),center(2,2),'o','markeredgecolor','k','markerfacecolor','k','markersize',2)
    plot(center(3,1),center(3,2),'o','markeredgecolor','k','markerfacecolor','k','markersize',2)
    plot(center(4,1),center(4,2),'o','markeredgecolor','k','markerfacecolor','k','markersize',2)
    plot(center(5,1),center(5,2),'o','markeredgecolor','k','markerfacecolor','k','markersize',2)
    plot(center(6,1),center(6,2),'o','markeredgecolor','k','markerfacecolor','k','markersize',2)
    plot(center(7,1),center(7,2),'o','markeredgecolor','k','markerfacecolor','k','markersize',2)
    plot(center(8,1),center(8,2),'o','markeredgecolor','k','markerfacecolor','k','markersize',2)
    %此处给推进器在图中的中心画上中心点。
%% draw forbidden sector
%    d2r = pi/180;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%azimuth 1
%    theta = (2:36)*d2r;
%    x = center(1,1) + R*cos(theta);
%    y = center(1,2) + R*sin(theta);
%    x=[x center(1,1) x(1)];
%    y=[y center(1,2) y(1)];
%    fill(x,y,'r','edgecolor','r');
    
%    theta = (144:178)*d2r;
%    x = center(1,1) + R*cos(theta);
%    y = center(1,2) + R*sin(theta);
%    x=[x center(1,1) x(1)];
%    y=[y center(1,2) y(1)];
%    fill(x,y,'r','edgecolor','r');
%%%%%%%%%%% azimuth 2
%%%%%%%%%%% azimuth 3
%%%%%%%%%%% azimuth 4
%%%%%%%%%%% azimuth 5
%%%%%%%%%%% azimuth 6
%%%%%%%%%%% azimuth 7
%%%%%%%%%%% azimuth 8
%% draw thrust and moment
   
    % 转入半径内
    thrust_temp =  2 * 1e-3 * block.Dwork(1).Data * R/800;     % 单个推力器最大800KN 即满推力下推力线段长2R 涉及画图的代码中推力和力矩都换成KN KN*m
    azimuth_temp = block.Dwork(3).Data * pi/180;  %推进器在随船坐标系中的角度，由于图中平台的x坐标朝右，所以只用把弧度换成角度即可，不用再调整角度。
    
    for i_temp = 1:N  
        block.Dwork(8).Data(i_temp) ...   %Dwork(8)是推进器推力线段变量，维度为N,line是绘制线段的函数，它返回的对象被存储到Dwork8中,可利用这个对象进行修改
            = line([center(i_temp,1) center(i_temp,1) + thrust_temp(i_temp) * cos(azimuth_temp(i_temp))],...
                   [center(i_temp,2) center(i_temp,2) + thrust_temp(i_temp) * sin(azimuth_temp(i_temp))],...
                   'linewidth',3,'color','k');
        %line函数语法line(x向量,y向量),此处为line([x1 x2],[y1 y2])
    end
    
    % 总推力向量线段句柄在Dwork(9).Data(1),这个线段长也和全回转的线段一样，长度
    block.Dwork(9).Data(1) = line([0 1e-3*block.Dwork(5).Data(1)/50],...
        [0 1e-3*block.Dwork(5).Data(2)/50],...
        'linewidth',5);
    %下面是画一条用来标记转矩起点的辅助线,颜色m为紫色，因为转矩线段theta的起点是-pi/2所以辅助线要朝下
    line([0 0],[-5 -10],'color','m');
    theta = -pi/2 : -0.01 : -pi/2 + 1e-3*block.Dwork(5).Data(3)/8e3*pi; %这表明只有当转矩KNM到达了2pi*10^4*pi时才会转矩的示意线段才转一圈，差不多是20万
    if isempty(theta)
        theta = -pi/2 : 0.01 : -pi/2 + 1e-3*block.Dwork(5).Data(3)/8e3*pi; %避免当block.Dwork(5).Data(3)为正时，由于:-0.01:的赋值语法导致theta为空
    end    
    x = 0 + 7.5 * cos(theta);
    y = 0 + 7.5 * sin(theta);
    % 总转矩向量线段句柄在Dwork(9).Data(2)    
    block.Dwork(9).Data(2) = line(x,y,'color','m','linewidth',5);
    % display current simulation time 显示当前模拟的时间 Dwork(10)
    text(70,100,'Simulation Time :','fontsize',12);
    block.Dwork(10).Data(1) = text(150,100,'0','fontsize',12); %因为现在是初始化所以时间为0
else
    block.Dwork(8).Data = zeros(N,1);
    block.Dwork(9).Data(1) = 0;
    block.Dwork(9).Data(2) = 0;
    block.Dwork(10).Data(1) = 0;
    
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
block.OutputPort(6).Data = block.Dwork(6).Data; %当设置了实际的推力松弛变量 才有此行
block.OutputPort(7).Data = block.Dwork(7).Data;
block.OutputPort(8).Data = block.Dwork(12).Data;
block.OutputPort(9).Data = block.Dwork(13).Data;
block.OutputPort(10).Data = block.Dwork(14).Data;

end %Outputs

%%
%% Update:
%%   Functionality    : Called to update discrete states
%%                      during simulation step
%%   Required         : No
%%   C-MEX counterpart: mdlUpdate
%%
function Update(block)
%% 该函数在模拟的时间步之前更新离散的状态变量
tSample = 1; %预测模型内部的时间步大小
%% MPC参数设置
%此处状态量是预测模型的参数，而不是MPC的参数
numX = 6; %状态量的个数 [u v r x y psi]
numP = 20;%MPC预测步长 原地定位不动的话 预测步长长也好像没什么意义
numC = 5;%MPC控制步长 控制步长最大值是预测步长-1    
% Row = 10;%松弛因子
N = 8; %全回转推进器个数，用于确定待确定的系统输出向量长度(转角和推力)
h = 0;%MPC校正系数 h属于[0,1] h=0时相当于完全相信模型，h=1时，完全相信上一步的误差，会将上一步的误差全部加到这一步来
%%%%%%%%%
%选择将环境前馈波浪力直接加入预测模型作为环境力或是直接在分配结果上加上
SwitchFeedForward = 0; % SwitchFeedForward为0时作为环境力，为1时直接在当前时间步分配
%%%%%%%%%
Q = block.DialogPrm(4).Data;
R = block.DialogPrm(5).Data;
F = block.DialogPrm(6).Data;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 推力分配ThrustAllocation参数设置
% 将角度转化为弧度
d2r = pi/180;
% 松弛因子的权矩阵
% RelaxWeight = 1e8*diag([60 60 2000]);
% 全回转速度的权矩阵
Omega = block.DialogPrm(12).Data;
% 推进器全回转速度区间(rad/sample_time) 本s函数内sample_time为1
da = [-2 2]*d2r;                                % 需按实际设置
% 单个推进器推力的变化区间(N)
Frange = 1e3*[50 800];                              % 需按实际设置
% 推进器推力的变化速度区间(N/sample_time)
dF = 1e3*[-50 50];                                  % 需按实际设置
% 各推进器的启用情况组成的列向量
u0 = block.DialogPrm(10).Data;
% Semi 708 % 需按实际设置 
L = [15.7 35.5; 47.02 24.58; 47.02 -24.58; 15.7 -35.5; -15.7 -35.5; ...
             -47.02 -24.58; -47.02 24.58; -15.7 35.5]; 
% Semi 807
% L = [15.7 24.58; 47.02 35.5; 47.02 -35.5; 15.7 -24.58; -15.7 -24.58; -47.02 -35.5; -47.02 35.5; -15.7 24.58;];     
% Semi 981
% L = [47.02 35.5; 47.02 24.58; 47.02 -24.58; 47.02 -35.5; -47.02 -35.5; -47.02 -24.58; -47.02 24.58; -47.02 35.5;]; 
%上一时间步推力
f0 = block.Dwork(1).Data; 
%上一时间步推力角
a0 = block.Dwork(3).Data; 
% 上一步推进器转角化为弧度
a0 = a0 * d2r;
% 目标函数奇异位置惩罚项分子常数
pp = 10;
% 目标函数奇异位置惩罚项分母中的常数
ee = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get_Coefficients用于个别涉及求导的系数计算 其中dTf在MPC中也需要用到

[dTf, ddT] = Get_Coefficients(f0,a0,pp,ee);
%% 参考轨迹设置 
%如果是完整的轨迹跟踪，应该要先定好未来一大段时间步的位置和速度信息
%但此处并不是严格意义的轨迹跟踪，只是动力定位的定位点不是一直在同一个地方而已
%对于动力定位问题，除非有规划好未来的速度参考点，否则过程误差中不应包含速度，而只在最终误差计算中含有速度项，
%因为我们希望的是在最后定位点处才无速度，这种需求可以通过设定权重矩阵Q和F实现
x_k = [block.InputPort(3).Data; block.InputPort(2).Data]; %控制器中获得的当前状态
posRef = zeros(3,1); %存储参考状态 前3个元素是速度状态，后3个元素是位置状态
speedRef = zeros(3,1);
posRef(:) = block.InputPort(1).Data; %posRef是参考位置，其实在当前脚本中没用到，当参考位置不是0,0,0的时候，这个posRef要用上，同时后面也要修改
speedRef(1:3) = 0;%参考的速度状态当然是0，不然无法保持位置不变，但是可以考虑在权重矩阵中涉及到速度的元素调小，来减轻对速度的考虑。 
% dotXRef = [0 0 0 0 0 0]';%参考的速度和位置的时间导 即参考轨迹在随体坐标系下加速度和在大地坐标系下的速度
% dotXRef = repmat(dotXRef,numP,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%
Xr = zeros((numP+1)*numX,1);
%1.不生成参考轨迹
Xr(:) = repmat([speedRef;posRef],numP+1,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%2.根据柔化因子生成参考轨迹
%这部分参考轨迹是会不断自动生成的用于增强鲁棒性
%当然目前没有进行对参考轨迹的可行性分析
% Xr(1:numX) = x_k; %[u v r x y psi]
% softAlpha = 0.8; %当柔化因子softAlpha为0时，没有平滑效果，softAlpha越大，系统柔性越大，鲁棒性越强，快速性变差。
% for j = 1:numP
%     rowIndex1 = (4:6) + numX*j;
%     rowIndex2 = rowIndex1 - 3;
%     Xr(rowIndex1) = softAlpha^j * x_k(4:6) + (1-softAlpha^j) * posRef; %Xr用于QP求解中的线性项
%     if j >=2                                                               
%     psik = Xr(rowIndex1(end)-numX);
%     Jpsi = [cos(psik) -sin(psik) 0; sin(psik) cos(psik) 0;0 0 1];
%     Xr(rowIndex2-numX) = Jpsi\((Xr(rowIndex1) - Xr(rowIndex1-numX))/tSample);
%     end
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 关于操作点或者说泰勒展开点的设置
% x_tilde是状态量x(k)[u v r x y psi]与泰勒展开点的差，展开点就定在当前输入的参考状态量上
% 则x_tilde(k)在此算例中各元素均是0  
taylorPoint = [speedRef;posRef];  %这次直接将泰勒展开点定在参考点上，这样正好x_tilde就是误差了 
% taylorPoint = x_k;
x_tilde = x_k-taylorPoint; %x_tilde准确来说是x_tilde(k)   

X_taylor = zeros((numP+1)*numX,1);
X_taylor(:) = repmat(taylorPoint,numP+1,1);   %X_taylor用于QP求解中的线性项

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 矩阵初始化    
M_rigidbody = block.DialogPrm(1).Data;
M_addition = block.DialogPrm(2).Data;
mass = M_rigidbody(1,1);
%Crb = [0 0 -m*v;0 0 m*u; m*v -m*u 0];
%Ca = [0 0 -a_22*v-a_26*r;0 0 a_11*u;a_22*v+a_62*r -a_11*u 0];
%partialC是对Crb+Ca求雅可比矩阵
a_11 = M_addition(1,1);
a_22 = M_addition(2,2);
%%%%%M矩阵给的是3乘3的水平面M矩阵不是6乘6的，因此索引是3不是6
a_26 = M_addition(2,3);%在该算例中a_26为0
a_62 = M_addition(3,2);
u_taylor = taylorPoint(1);
v_taylor = taylorPoint(2);
r_taylor = taylorPoint(3);
psi_taylor = taylorPoint(6);
partialC = [0  -(mass+a_22)*r_taylor  -(mass+a_22)*v_taylor-2*a_26*r_taylor;...
            (a_11+mass)*r_taylor  0  (a_11+mass)*u_taylor;...
            (a_22-a_11)*v_taylor+a_62*r_taylor  (a_22-a_11)*u_taylor  a_62*u_taylor];
Dp = block.DialogPrm(3).Data; %线性的阻尼矩阵
%R = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0;0 0 1];
partialR = [cos(psi_taylor) -sin(psi_taylor) 0 0 0 -u_taylor*sin(psi_taylor)-v_taylor*cos(psi_taylor);...
            sin(psi_taylor) cos(psi_taylor) 0 0 0 u_taylor*cos(psi_taylor)-v_taylor*sin(psi_taylor);...
            0  0  1  0  0  0];
temp = -1*(M_rigidbody+M_addition)\(partialC + Dp); %A_tilde的左上角3*3部分
A_tilde = zeros(6,6);
A_tilde(1:3,1:3) = temp;
A_tilde(4:6,:) = partialR;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%由于此处参与MPC计算的输出不再是合力(长度为3的向量，3DOF)，而是所有推进器的推力和角度
%即原来的合力Tau现在变成了Tau = B(alpha)u，alpha是推进器转角而u是推力。
%从新的Tau的组成也可以看出这是个非线性项(两输出alpha和u相乘)，因此需要线性化， 
% B(alpha)u = B(alpha0)u0 + B(alpha0) * (u-u0) +
% partial(B(alpha0)*u0)/partial(alpha) * (alpha-alpha0)
% 把u_tilde和alpha_tilde并列到一起组成1个2*N长的系统输出列向量 
% u0和alpha0定为上一时间步的推力和转角 只需要读取Dwork1和3就行 因为当前Dwork还没更新
% 新的广义输出1-8为u_tilde，9-16为alpha_tilde
B_tilde = zeros(6,2*N);
%只修改前3行
B_tilde(1:3,1:N) = thrusters_configuration(a0,L);
B_tilde(1:3,1+N:2*N) = dTf;
B_tilde(1:3,1:2*N) = (M_rigidbody + M_addition)\B_tilde(1:3,1:2*N);
    
%上面的矩阵都是连续的状态空间方程中的，下面进行矩阵离散化
A_tildeDisc = eye(6) + tSample*A_tilde;  %其实是e的At次方的近似
B_tildeDisc = tSample*B_tilde;  
    
%首先是各矩阵的初始化M矩阵Matrix_M，M矩阵是由状态空间中的A矩阵构成的
rowA = size(A_tildeDisc,1); %rowA和状态量维度是相同的
colB = size(B_tildeDisc,2);
Matrix_M = [eye(rowA);zeros(numP*rowA,rowA)];
Matrix_C = zeros((numP+1)*rowA,colB*numC); 
Matrix_I = [eye(rowA);zeros(numP*rowA,rowA)];
%定义矩阵M、C、I
temp1 = eye(rowA);
for i = 1:numP
    rowIndex = (1:rowA)+i*rowA;
    Matrix_M(rowIndex,:) = A_tildeDisc*temp1; 
    Matrix_C(rowIndex,:) = [temp1*B_tildeDisc,Matrix_C(rowIndex-rowA,1:end-colB)];
    %%%%%%%%%%%%%%%
    %由于本系统的特殊情况，即U是推力而不是推力变化量，因此对Matrix_C超过控制步长影响的行数进行额外处理，详情看Notability，MPC第7页起
    %或者说超出控制步长的部分按照原来的Matrix_C来看是U就默认为0。而本系统希望是超出控制步长的部分U依然维持最后一个步长的U直到预测步长结束
    %关于以推力变化量为U，即控制律以deltaUn形式给出的情况，可以从Notability中基于状态空间方程的MPC中看到，有一种扩展状态空间模型
    if i > numC %超出控制步长影响的行数 只改最后一大列块（colB） 
        Matrix_C(rowIndex,end-colB+1:end) = Matrix_C(rowIndex,end-colB+1:end) + Matrix_C(rowIndex-rowA,end-colB+1:end);
    end
    %%%%%%%%%%%%%%%
    Matrix_I(rowIndex,:) = eye(rowA);
    temp1 = A_tildeDisc*temp1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 定义Q_bar和R_bar,尤其是R_bar，因为输入不再只是总推力，而是全回转推力和转角 RO_bar是R和Omega共同组成的输入权重矩阵
RO = blkdiag(R,Omega);
R = blkdiag(R,zeros(size(Omega,1)));
s_Q = size(Q,1);   %输入的Q的维度 
s_R = size(R,1);   %输入的R的维度
s_RO = size(RO,1);
Q_bar = zeros(s_Q*(numP+1),s_Q*(numP+1));
for i = 1:numP
    Q_bar(1+(i-1)*s_Q:i*s_Q,1+(i-1)*s_Q:i*s_Q) = Q; %Q_bar的对角线上全是Q，
    %最后一个Q可以自己做一个F权重矩阵,用来专门调整最后一步的x(k+N|k)的权重。
end
Q_bar(numP*s_Q+1:(numP+1)*s_Q,numP*s_Q+1:(numP+1)*s_Q) = F;
    
R_bar = zeros(s_R*(numC),s_R*(numC)); %R_bar与控制区间长度相关，控制区间最长为预测区间-1
for i = 1:numC
    if i == numC
        R_bar(1+(i-1)*s_R:i*s_R,1+(i-1)*s_R:i*s_R) = (numP-numC+1)*R; 
    else
        R_bar(1+(i-1)*s_R:i*s_R,1+(i-1)*s_R:i*s_R) = R;
    end
end

RO_bar = zeros(s_RO*(numC),s_RO*(numC)); %R_bar与控制区间长度相关，控制区间最长为预测区间-1
for i = 1:numC
    if i == numC
        RO_bar(1+(i-1)*s_RO:i*s_RO,1+(i-1)*s_RO:i*s_RO) = (numP-numC+1)*RO; 
    else
        RO_bar(1+(i-1)*s_RO:i*s_RO,1+(i-1)*s_RO:i*s_RO) = RO;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 补偿对非线性模型线性化引起的常数项f(x_r,u_r)-dot(x_r) 其中dot(x_r)是设定的参考值的时间导
%求解f(x_r,u_r)所需的矩阵
Crb = [0 0 -mass*v_taylor;0 0 mass*u_taylor; mass*v_taylor -mass*u_taylor 0];
Ca = [0 0 -a_22*v_taylor-a_26*r_taylor;0 0 a_11*u_taylor;a_22*v_taylor+a_62*r_taylor -a_11*u_taylor 0];
Rpsi = [cos(psi_taylor) -sin(psi_taylor) 0; sin(psi_taylor) cos(psi_taylor) 0;0 0 1];  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tau_env = block.InputPort(6).Data; %随体坐标系下的环境力 block.InputPort(6)是风流力  block.InputPort(5)是obsSlowVaringEnvForces 
% Tau_env = block.InputPort(5).Data;
Tau_env = 0;
Tau_0 = thrusters_configuration(a0,L) * f0;%若泰勒展开时，推力的展开点不选择为0时，则该项不为0

if (block.DialogPrm(7).Data == 1) && (SwitchFeedForward == 0)
    Tau_env = Tau_env + block.InputPort(4).Data; %block.InputPort(4)是feedForwardForces
end

nv_taylor = taylorPoint(1:3); 
fx0u0 = zeros(numX,1);
fx0u0(1:3) = (M_rigidbody + M_addition)\(Tau_env + Tau_0 -(Crb + Ca + Dp) * nv_taylor);
fx0u0(4:6) = Rpsi * nv_taylor;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% f(x_r,u_r)不随时间变化（如果开启波浪前馈，相当于认为波浪力一直以该值持续作用在未来NumP步）
fx0u0 = repmat(fx0u0,numP,1);
% 结束
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% f(x_r,u_r)随时间变化（如果开启波浪前馈，相当于认为波浪力只持续作用在当前时间步）
% if (block.DialogPrm(7).Data == 1) && (SwitchFeedForward == 0)
%     fx0u0 = zeros(numX,1);
%     fx0u0(1:3) = (M_rigidbody + M_addition)\(Tau_env + Tau_0 -(Crb + Ca + Dp) * nv_taylor);
%     fx0u0(4:6) = Rpsi * nv_taylor;
%     fx0u0(7:9) = (M_rigidbody + M_addition)\(Tau_env + Tau_0 - block.InputPort(4).Data -(Crb + Ca + Dp) * nv_taylor);
%     fx0u0(10:12) = Rpsi * nv_taylor;
%     fx0u0(7:numP*numX) = repmat(fx0u0(7:12),numP-1,1);
% end
%结束
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% finalBias = fx0u0 - dotXRef;
finalBias = fx0u0;
Xbias = zeros((numP+1)*rowA,1);  %Xbias的第1-6行为0 因为这几行是当前k时刻的状态

for i = 1:numP
    rowIndex = (1:rowA)+i*rowA;
    Xbias(rowIndex) = A_tildeDisc * Xbias(rowIndex-rowA) + tSample*finalBias(rowIndex-rowA);  %Xbias用于QP求解中的线性项
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MPC模型误差修正项
%     Y_cor=Y0+H*e;
%当前时刻的状态y与上一时刻预测的当前时刻的状态y的差为error，先拿x_k的后三项x,y,psi试试
%     error = zeros(numX,1);
%由于第一次运行update时,block.Dwork(11).Data存储的还不是上一时刻预测的状态,所以此时error不正确，应直接令error为0
if block.CurrentTime == 0
    error = zeros(numX,1);
else
    error = x_tilde(1:6) - block.Dwork(11).Data;
end

for i = 1:1:numP+1 %因为X_k的第一大行留给了x(k|k)所以是numP+1
    if i == 1
        H_cor(1+(i-1)*numX:i*numX,:) = zeros(numX,numX);
    else
        H_cor(1+(i-1)*numX:i*numX,:) = h*eye(numX);
    end
 end
X_cor = H_cor*error; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% 推力分配部分Thrust Allocation 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 用于全回转推进器的dda 且各全回转推进器的旋转速度相同 (由于将推力分配纳入MPC中，待求解的推力不再是单时间步的，而是numC步的) 因此ff要重新改正
dda = zeros(numC*N,2); 
for i = 1 : N
    dda(i,:) = da;  
end

dda(N+1:end,:) = repmat([-1e8*pi,1e8*pi],(numC-1)*N,1); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%用于有禁止角情况的dda
% % 禁止角设置后的可行区间，1、4推进器具有两个区间
% % angle_sector1o = [-15 15;75 285]*d2r;
% % angle_sector2o = [165 425]*d2r;
% % angle_sector3o = [115 375]*d2r;
% % angle_sector4o = [165 195;-105 105]*d2r;
% % angle_sector5o = [-15 245]*d2r;
% % angle_sector6o = [-65 195]*d2r;
% angle_sector1o = [-180 180]*d2r;
% angle_sector2o = [-180 180]*d2r;
% angle_sector3o = [-180 180]*d2r;
% angle_sector4o = [-180 180]*d2r;
% angle_sector5o = [-180 180]*d2r;   
% angle_sector6o = [-180 180]*d2r;
% angle_sector7o = [-180 180]*d2r;   
% angle_sector8o = [-180 180]*d2r;
% 
% % 本步可行区间，考虑多区间推进器的区间选择，未来可能加入动态禁止角
% angle_sector = 1000*ones(N,2);
% angle_sector(1,:) = angle_sector1o;
% angle_sector(2,:) = angle_sector2o;
% angle_sector(3,:) = angle_sector3o;
% angle_sector(4,:) = angle_sector4o;
% angle_sector(5,:) = angle_sector5o;
% angle_sector(6,:) = angle_sector6o;
% angle_sector(7,:) = angle_sector7o;
% angle_sector(8,:) = angle_sector8o;
% 
% % 推进器全回转速度约束区间
% dda = 1000*ones(N,2);     % 初始化
% % a0会在[-200deg 200deg]之间 即[-1.111pi 1.111pi]之间 a0本身也在angel_sector范围内
% % angle_sector的值在[-pi pi]之间
% % 首先考虑可行区间与上步角度的限制,并将dda转入[-2pi 2pi],以便与da比较
% for i = 1 : N
%     dda(i,:) = angle_sector(i,:) - a0(i);  %dda在[-2.111pi 2.111pi]
%     while(dda(i,2) + 100*eps < 0)
%         dda(i,:) = dda(i,:) + 2*pi;
%     end
%     while(dda(i,1) - 100*eps > 0)
%         dda(i,:) = dda(i,:) - 2*pi;
%     end
% end
% % 经过上述计算dda在[-2pi 2pi]
% % 与全回转推进器每步的最大转速da比较，选择最小的变化区间
% for i = 1 : N
%     if da(1) > dda(i,1)
%         dda(i,1) = da(1);
%     end
%     if da(2) < dda(i,2)
%         dda(i,2) = da(2);
%     end
% end
%用于有禁止角情况的dda
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 推进器推力变化区间(由于将推力分配纳入MPC中，待求解的推力不再是单时间步的，而是numC步的) 因此ff要重新改正
F = [ Frange(1)*u0 Frange(2)*u0 ]; % 考虑失效螺旋桨的推力器变化区间 F是N行2列的矩阵
ff = zeros(numC*N,2);
countTable = zeros(N,2);
toBeFilled = -1;
for i = 1 : numC
    rowIndex = (i-1)*N;
    for j = 1 : N
        ff(rowIndex+j,1) = f0(j) + i*dF(1); %% dF = 1e3*[-50 50]; 
        if  ff(rowIndex+j,1) <= F(j,1)
            ff(rowIndex+j,1) = F(j,1);
            countTable(j,1) = 1;
        end

        ff(rowIndex+j,2) = f0(j) + i*dF(2);
        if  ff(rowIndex+j,2) >= F(j,2)
            ff(rowIndex+j,2) = F(j,2);
            countTable(j,2) = 1;
        end
    end
    if sum(countTable,'all') == 2*N %所有的控制步上下界都填到 推力变化区间上限了
        toBeFilled = i*N;
        break                   
    end
end
if (toBeFilled ~= -1) && (toBeFilled ~= numC*N)
    ff(toBeFilled+1:end,1) = repmat(F(:,1),numC-i,1);
    ff(toBeFilled+1:end,2) = repmat(F(:,2),numC-i,1);
end

%到此处ff求出的是T+deltaT的上下界、原推力分配程序需要的就是T+deltaT，但此函数需要的是deltaT
ff_old = ff;
ff = ff - repmat(f0,numC,1);


%% QP
%为二次规划问题准备系数 %目标函数 = 1/2*x'*H*x + fQP'*x 
%以前的老形式：由于目前的展开方式，导致x_tilde(k|k)=0，所以x(k|k) =x_tilde(k|k)+x_taylor，而x_taylor就是x(k|k)
% G = Matrix_M'*Q_bar*Matrix_M; %G矩阵在常数项xGx中所以不需要放到最优化中处理
E = Matrix_M'*Q_bar*Matrix_C;
H = Matrix_C'*Q_bar*Matrix_C + RO_bar;  %(2N*numP)*(2N*numP)的矩阵
%！！！！！！！！！注意一个问题 理论上来讲H应该是对称矩阵 因为我的Qbar和Rbar都是对称的 但是可能因为某些数值计算问题
%matlab提示QP求解超时，H并不是对称矩阵，进行尝试后，发现确实不是，用H-H'计算后发现有个别元素有1e-22量级的数值。
%因此使用H = (H+H')/2将H变为严格对称的矩阵,同时模拟中大量的没有计算完的二次规划
H = (H+H')/2;
% H = blkdiag(H,RelaxWeight); % H加上松弛因子s定位中心的距离s=[s1;s2;s3]，维度变为(2N*numP+3) * (2N*numP+3)
%%%%%%%%%%%%%%%%%%%%%%%%%%
%由于QP的求解对象由原来Thrust_Allocation函数内的[T0+deltaT;deltaAlpha;s]变成了
%[deltaT;deltaAlpha;s]，因此在QP的线性项中要多个(repmat([f0;zeros(N,1)],numC,1)'*R_bar)'，f0是上一时间步的推力列向量
%QP目标函数线性项系数
fbias = (Xbias'*Q_bar*Matrix_C)';
f_taylor = (X_taylor'*Q_bar*Matrix_C)';
f_reference = (Xr'*Q_bar*Matrix_C)';
f_cor = (X_cor'*Q_bar*Matrix_C)';

fQP = zeros(2*N*numC,1);
fQP(1:2*N*numC) = (x_tilde'*E)'+ fbias + f_taylor - f_reference + f_cor + (repmat([f0;zeros(N,1)],numC,1)'*R_bar)';
%f0是原推力也就是推力展开点，由于考虑f0+df的总推力消耗，因此此处有这个线性项
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%ddT用于防止推力角度奇异
fQP(1:2*N*numC) = fQP(1:2*N*numC) + repmat([zeros(N,1);0.5*ddT'],numC,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%QP线性不等式约束,设置成最后的预测步距离目标点在一个范围内 M*x_tilde + C * Uk + Xconstant（Xbias）+ X_taylor - Xr + s <= [xErrorConstraint yErrorConstraint psiErrorConstraint]';
A = Matrix_C(end-2:end,:);
XcFinal = Xbias(end-2:end);
Mx_tilde = Matrix_M(end-2:end,:) * x_tilde;
errorFinalConstraints = [60.0;60.0;60*pi/180];
b = errorFinalConstraints - XcFinal - Mx_tilde - X_taylor(end-2:end) + Xr(end-2:end);
%%%%%%%%%%%%%%下面两行代码将根据上面的上界约束补充对称的下界约束
A = [A;-Matrix_C(end-2:end,:)];
b = [b;errorFinalConstraints + XcFinal + Mx_tilde + X_taylor(end-2:end) - Xr(end-2:end)];
%!!!!!!!!!约束应该有六行 因为一个自由度上要有两个约束(上界和下界) A*x<=b Ax>=-b -Ax<=b
%%%%%%%%%%%%%%采样时间推力向下变化极限(负值) <= deltaU(n+1)-deltaU(n) <= 采样时间推力向上变化极限(正值) 采样时间转角向下变化极限 <= deltaAlpha(n+1)-deltaAlpha(n) <= 采样时间推力向上变化极限 
subMatrix = zeros((numC-1)*(2*N),numC*(2*N));
for i = 1:numC-1
    rowIndex = (1:2*N) + (i-1)*(2*N);
    subMatrix(rowIndex,rowIndex) = -1 * eye(2*N);
    subMatrix(rowIndex,rowIndex + 2*N) = eye(2*N);
end
upperBound = repmat([dF(2)*ones(N,1);da(2)*ones(N,1)],numC-1,1);  %之前解出问题就出在这里 之前da(1)da(2)填成了dda(1) dda(2)
lowerBound = -1 * repmat([dF(1)*ones(N,1);da(1)*ones(N,1)],numC-1,1);   
A = [A;subMatrix;-subMatrix];
b = [b;upperBound;lowerBound];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%QP线性等式约束 Aeq beq为空是因为本次求解的QP问题没有等式约束，只有线性等式约束和变量上下限。
Aeq = [];
beq = [];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%待求解变量上下界 %因为用了deltaU 上下界没那么简单 现在的求解对象UandAlpha长为numC*2*N,即所有控制步长内的推力和转角
%所以这里设置的lb ub应该是长度为2*N*numC + 3列向量
ffdda = zeros(2*N*numC,2);
for i = 1:numC
    rowIndex = (1:2*N) + (i-1)*2*N;
    rowIndex2 = (1:N) + (i-1)*N;
    ffdda(rowIndex,:) = [ff(rowIndex2,:);dda(rowIndex2,:)];
end

lb = ffdda(:,1); %规定了deltaT的上、下界(ff) 和s的上下界（+-1e10）
ub = ffdda(:,2);
%从向量 x0 开始求解上述QP问题 
% x0 = [zeros(N,1);zeros(N,1);zeros(3,1)]; 
x0 = [];
options = optimset('Algorithm','interior-point-convex','MaxIter',100); %内点法求解 默认最大迭代次数为200
Ualpha_k = quadprog(H,fQP,A,b,Aeq,beq,lb,ub,x0,options); %fmincon 非线性目标函数优化
%%%%%%%%%%%%%%%%%%%%%%%%
%简单快速计算一次QP求解的完整的NumC长的控制序列f
B_UalphaTemp = (M_rigidbody + M_addition) * B_tilde(1:3,:);
B_Ualpha = blkdiag(B_UalphaTemp,B_UalphaTemp,B_UalphaTemp,B_UalphaTemp,B_UalphaTemp);
deltaTauSeries = B_Ualpha * Ualpha_k;
ForeceMomentK = deltaTauSeries + repmat(Tau_0,numC,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% QP结果输出
df = Ualpha_k(1:N); %df 相对于上一步的推力值变化 (N)
f = f0 + df;  
count = find(f > Frange(2)); %逻辑值count表示f是否超出最大推力
if count
    f = f0; %维持原推力  
    df = 0*zeros(N,1);
    block.Dwork(13).Data(1) = block.Dwork(13).Data(1) + 1;
end

dalpha = Ualpha_k(N+1:2*N);
a = a0 + dalpha;
dalpha = dalpha / d2r;  %dalpha 相对于上一步的转角变化 (degree)
a = a / d2r;  %a 分配后的各推进器转角向量 (degree)
%% 将a转入[-pi pi]区间，以方便画图  落入[-pi pi]区间却设置200 -200为条件，应该是担心模拟过程中a（i）的值在pi和-pi间乱跳
for i = 1 : N
    while a(i) > 200
        a(i) = a(i) - 360;
    end
    while a(i) < -200
        a(i) = a(i) + 360;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 计算所预测的实际推力和该时刻计算的未来状态
Balpha = thrusters_configuration(a*d2r,L); %Balpha推力配置矩阵
tau_r = Balpha * f; %  tau_r 分配后实际推力向量 (N;N;N*m)
% dtau = tau_r - tau; %  dtau 实际推力与要求推力的差值
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X_k = Matrix_M * x_tilde + Matrix_C * Ualpha_k + Xbias; %此处X_k是x_tilde形式的
xkplus1_prd = X_k(numX+1:2*numX); %注意这个x_prd也是x_tilde形式的，是x(k+1|k)-x(taylorpoint in k step)。
X_k_real = X_k + X_taylor; %X_k_real是状态量x在从当前到未来P步组成的向量
xFinal_prd = X_k_real(end-numX+1:end);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
XrEachState = zeros(numP+1,numX); 
XprdEachState = zeros(numP+1,numX); 
for i = 1:numX
    rowIndex = i + (0:numX:numP*numX);
    XrEachState(:,i) = Xr(rowIndex);
    XprdEachState(:,i) = X_k_real(rowIndex);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% %% 在MPC结果上补充前馈力
% %%该模块目前还存在很多问题，可能跟QP求解有关系，目前应用起来后不知道为什么，反而前面MPC的QP会解出各推进器满推力的情况，即使目前已经超调
% if (block.DialogPrm(7).Data == 1) && (SwitchFeedForward == 1)
%     %tau_target          本步所要求的推力向量 前面update解出的推力加上波浪前馈力  (N;N;N*m)
%     tau_target = tau_r - 1 * block.InputPort(4).Data;
%     %松弛因子
%     relaxWeight2 = 1e10*diag([30 30 10]);
%     % 推力权矩阵
%     R2 = block.DialogPrm(5).Data;
%     % 全回转速度的权矩阵
%     Omega2 = Omega;
%     dda2 = zeros(N,2);
%     for i = 1 : N
%         dda2(i,:) = da;
%     end
%     % 推进器新的推力变化区间
%     ff2 = ff_old(1:N,:);
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     %其他系数
%     T =  thrusters_configuration(a0,L);
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     % 为二次规划问题准备系数
%     H = blkdiag(2*R2,2*Omega2,2*relaxWeight2);
%     I = [zeros(N,1);ddT';zeros(3,1)];
%     A = [];
%     b = [];
%     Aeq = [T,dTf,eye(3)];
%     beq = tau_target  ;
%     lb = [ff2(:,1);dda2(:,1);-1e10*ones(3,1)];
%     ub = [ff2(:,2);dda2(:,2); 1e10*ones(3,1)];
%     x0 = [f0;zeros(N,1);zeros(3,1)];   %% 解向量格式：[Thrust0+dThrust dAngle s ]
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     % 求解二次规划问题
%     options = optimset('Algorithm','interior-point-convex','Display','off');
%     solution1 = quadprog(H,I,A,b,Aeq,beq,lb,ub,x0,options);
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     % 函数输出
%     f = solution1(1:N);
%     df = f - f0;
%     dalpha = solution1(N+1:2*N);
%     a = a0 + dalpha;
%     dalpha = dalpha / d2r;  %dalpha 相对于上一步的转角变化 (degree)
%     a = a / d2r;  %a 分配后的各推进器转角向量 (degree)
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     %将a转入[-pi pi]区间，以方便画图  落入[-pi pi]区间却设置200 -200为条件，应该是担心模拟过程中a（i）的值在pi和-pi间乱跳
%     for i = 1 : N
%         while a(i) > 200
%             a(i) = a(i) - 360;
%         end
%         while a(i) < -200
%             a(i) = a(i) + 360;
%         end
%     end
%     Balpha = thrusters_configuration(a*d2r,L); %Balpha推力配置矩阵
%     tau_r = Balpha * f; %  tau_r 分配后实际推力向量 (N;N;N*m)
%     dtau = tau_r - tau_target; %  dtau 实际推力与要求推力的差值
%     
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% 将指定工作区Dwork变量更新
block.Dwork(1).Data = f;
block.Dwork(2).Data = df; 
block.Dwork(3).Data = a;
block.Dwork(4).Data = dalpha;
block.Dwork(5).Data = tau_r;
if (block.DialogPrm(7).Data == 1) && (SwitchFeedForward == 1)
    block.Dwork(6).Data = dtau; %当设置了实际的推力松弛变量 才有此行
else
    block.Dwork(6).Data = zeros(3,1);
end
block.Dwork(7).Data = zeros(3,1);
block.Dwork(11).Data = xkplus1_prd;
block.Dwork(12).Data = xFinal_prd;
block.Dwork(14).Data = ForeceMomentK;

%% plot animation 在InitializeConditions函数中已经创建了绘图窗格
if block.DialogPrm(11).Data == 1
    center = [30 66; 85 46; 85 -46; 30 -66; -30 -66; -85 -46; -85 46; -30 66;];
    Radius = 10;
    % 转入半径内
    thrust_temp = 1e-3 * 2*block.Dwork(1).Data * Radius/800;     
    azimuth_temp = block.Dwork(3).Data * pi/180;
    %Dwork(8)中存储的是各推进器推力线段的对象 用set函数 set(Dwork(8).Data,'xdata',[x1 x2],'ydata',[y1 y2])
    for i_temp = 1:8
        set(block.Dwork(8).Data(i_temp),...
            'xdata',[center(i_temp,1) center(i_temp,1) + thrust_temp(i_temp) * cos(azimuth_temp(i_temp))],...
            'ydata',[center(i_temp,2) center(i_temp,2) + thrust_temp(i_temp) * sin(azimuth_temp(i_temp))]);
    end
    %Dwork(9)中存储的是合力线段和扭矩,Dwork(5).Data(2)是合力的y方向分量,Dwork(5).Data(1)是合力的x方向分量
    set(block.Dwork(9).Data(1), 'xdata',[0 1e-3 * block.Dwork(5).Data(1)/50],...
        'ydata',[0 1e-3 * block.Dwork(5).Data(2)/50]);
    
    theta = -pi/2 : -0.01 : -pi/2 + 1e-3 * block.Dwork(5).Data(3)/8e3*pi;
    x = 7.5 * cos(theta);
    y = 7.5 * sin(theta);
    set(block.Dwork(9).Data(2), 'xdata',x,'ydata',y);
    set(block.Dwork(10).Data,'string',num2str(block.CurrentTime));
    drawnow
else
    return
end

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
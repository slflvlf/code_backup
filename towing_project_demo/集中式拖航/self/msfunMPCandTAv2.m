function msfunMPCandTAv2(block)
%msfunMPCandTAv2 is used to setup a thrust-allocation block
%��־
%9��24�տ����ҵ���֮ǰQP�������½���������ֹ������ԭ�� 858��upperbound
%9��25�շ���block.DialogPrm(7)����ǰ�������صĶԻ������δʹ�ã���֮ǰ��ģ�ⶼ��ȫû����ǰ����
%9��25�� ԭ������daǰ���������ֲ�ͬ���� ���ǵ�λʱ��ת�Ǳ仯���� �����������һ����ת�Ǳ仯 ���ڽ����ֱ���ʹ�ò�ͬ��������������
%v2ȡ����ԭ����λ���ɳڱ���s 
% ����������ڲ��ο��켣(���ڶ��㶨λ)��û�н��жԲο��켣�Ŀ����Է���
%10��8�� �������QP������������������ Dwork(14) �Ժ��޸�NumCʱ��ص�����ش���

% INPUT:
%  eta_ref               
%  eta_est
%  Nv_est
%  feedForwardForces
%  obsSlowVaringEnvForces
%  windCurrentForces
% OUTPUT:
%  f                �����ĸ��ƽ�������ֵ���� (N)
%  df               �������һ��������ֵ�仯 (N)
%  a                �����ĸ��ƽ���ת������ (degree)
%  dalpha           �������һ����ת�Ǳ仯 (degree)
%  tau_r            �����ʵ���������� (N;N;N*m)
%  dtau             ʵ��������Ҫ�������Ĳ�ֵ
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
%%   Required(������)         : Yes
%%   C-Mex counterpart: mdlInitializeSizes
%%
function setup(block)

% Register number of ports �Ǽ����롢���portģ�����
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
block.InputPort(1).DirectFeedthrough = false; %���ø������Ƿ�ֱ���������Outputs block
block.InputPort(1).SamplingMode = 'Sample';
%% eta_est (m;m;rad)
block.InputPort(2).Dimensions        = 3;
block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = false; %���ø������Ƿ�ֱ���������Outputs block
block.InputPort(2).SamplingMode = 'Sample';
%% Nv_est (m/s;m/s;rad/s)
block.InputPort(3).Dimensions        = 3;
block.InputPort(3).DatatypeID  = 0;  % double
block.InputPort(3).Complexity  = 'Real';
block.InputPort(3).DirectFeedthrough = false; %���ø������Ƿ�ֱ���������Outputs block
block.InputPort(3).SamplingMode = 'Sample';
%% feedForwardForces (N;N;N*m)
block.InputPort(4).Dimensions        = 3;
block.InputPort(4).DatatypeID  = 0;  % double
block.InputPort(4).Complexity  = 'Real';
block.InputPort(4).DirectFeedthrough = false; %���ø������Ƿ�ֱ���������Outputs block
block.InputPort(4).SamplingMode = 'Sample';
%% obsSlowVaringEnvForces (N;N;N*m)
block.InputPort(5).Dimensions        = 3;
block.InputPort(5).DatatypeID  = 0;  % double
block.InputPort(5).Complexity  = 'Real';
block.InputPort(5).DirectFeedthrough = false; %���ø������Ƿ�ֱ���������Outputs block
block.InputPort(5).SamplingMode = 'Sample';
%% windCurrentForces (N;N;N*m)
block.InputPort(6).Dimensions        = 3;
block.InputPort(6).DatatypeID  = 0;  % double
block.InputPort(6).Complexity  = 'Real';
block.InputPort(6).DirectFeedthrough = false; %���ø������Ƿ�ֱ���������Outputs block
block.InputPort(6).SamplingMode = 'Sample';

%%
% Override output port properties
N = 8;  %�ƽ�������
%% f �������������ĸ��ƽ�������ֵ����(ά�����ƽ���������ͬ)(N)
block.OutputPort(1).Dimensions       = N;
block.OutputPort(1).DatatypeID  = 0; % double
block.OutputPort(1).Complexity  = 'Real';
block.OutputPort(1).SamplingMode = 'Sample';
%% df �������һ��������ֵ�仯 (ά�����ƽ���������ͬ)(N)
block.OutputPort(2).Dimensions       = N;
block.OutputPort(2).DatatypeID  = 0; % double
block.OutputPort(2).Complexity  = 'Real';
block.OutputPort(2).SamplingMode = 'Sample';
%% a �����ĸ��ƽ���ת������ (ά�����ƽ���������ͬ)(degree)
block.OutputPort(3).Dimensions       = N;
block.OutputPort(3).DatatypeID  = 0; % double
block.OutputPort(3).Complexity  = 'Real';
block.OutputPort(3).SamplingMode = 'Sample';
%% dalpha �������һ����ת�Ǳ仯 (ά�����ƽ���������ͬ)(degree)
block.OutputPort(4).Dimensions       = N;
block.OutputPort(4).DatatypeID  = 0; % double
block.OutputPort(4).Complexity  = 'Real';
block.OutputPort(4).SamplingMode = 'Sample';
%% tau_r �����ʵ������������ (N;N;N*m)
block.OutputPort(5).Dimensions       = 3;
block.OutputPort(5).DatatypeID  = 0; % double
block.OutputPort(5).Complexity  = 'Real';
block.OutputPort(5).SamplingMode = 'Sample';
%% dtau ʵ��������Ҫ������(������������)�Ĳ�ֵ(N;N;N*m)
block.OutputPort(6).Dimensions       = 3;
block.OutputPort(6).DatatypeID  = 0; % double
block.OutputPort(6).Complexity  = 'Real';
block.OutputPort(6).SamplingMode = 'Sample';
%% s ������Ԥ�ⲽ����ο�λ�õ������ɳ���(m;m;rad)
block.OutputPort(7).Dimensions       = 3;
block.OutputPort(7).DatatypeID  = 0; % double
block.OutputPort(7).Complexity  = 'Real';
block.OutputPort(7).SamplingMode = 'Sample';
%% xFinal_prd �����������һ��Ԥ�ⲽNp��ϵͳ״̬ x(k+Np|k) = [u v r x y psi]
block.OutputPort(8).Dimensions       = 6;
block.OutputPort(8).DatatypeID  = 0; % double
block.OutputPort(8).Complexity  = 'Real';
block.OutputPort(8).SamplingMode = 'Sample';
%% abnormalUtimes QP��̬�����Ĳ�������������
block.OutputPort(9).Dimensions       = 1;
block.OutputPort(9).DatatypeID  = 0; % double
block.OutputPort(9).Complexity  = 'Real';
block.OutputPort(9).SamplingMode = 'Sample';
%% ForeceMomentK QP����������NumC���Ŀ������У���ת����Fx Fy Moment��
block.OutputPort(10).Dimensions       = 3*5; %ά��Ϊ3*NumC
block.OutputPort(10).DatatypeID  = 0; % double
block.OutputPort(10).Complexity  = 'Real';
block.OutputPort(10).SamplingMode = 'Sample';

%%
% Register parameters �ڷ�װ��������Ĳ�����������
block.NumDialogPrms = 12;
% �����ֱ���M,Ma,D,Q,R,F,FeedForwardSwitch,f0,a0,u0,wa
%1.M ��������������
%2.Ma �����ĸ�����������
%3.D �������������
%4.Q Position Error Weight Matrix Q 
%5.R Input Weight Matrix R 
%6.F Final Position Error Weight Matrix F 
%7.FeedForwardSwitch
%8.f0 ���ƽ�����ʼ������ɵ�������(KN) 
%9.a0 ���ƽ���ת����ɵ����� (degree)(�洬����ϵ��)
%10.u0 ���ƽ��������������ɵ�������
%11.wa �Ƿ���ʾ�������ƽ���ת�ǵĶ���
%12.Omega ȫ��ת�ƽ���ת�ǵ�Ȩ����

% Register sample times �Ǽǲ���ʱ�䣬Ϊ0.05s
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
%�ڴ˴�setup������ע�����Ӻ������ܲ����ں����������Ӻ������ܣ������û�����뿪ʼ���󵼵��ӹ���
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
%% ���ô洢�ڸú���ר�ù������ı�����Ҳ���Ǻ����ڵ�״̬������Dwork������Simulink�����ģ����ÿ��S����ʵ���Ĵ洢�ռ�顣
%����ͬS������֮����Ҫͨ��ȫ�ֱ������߾�̬�����������ݽ���ʱ��������S������ʹ��Dwork���������б����洢����������InitializeConditions�����ж���ı�������Update��������û�еģ�
%Dwork(1)��Dwork(8)��OutputPort(1)��OutputPort(8)һһ��Ӧ

N = 8;  %�ƽ�������
block.NumDworks = 14;
%% f �������������ĸ��ƽ�������ֵ����(ά�����ƽ���������ͬ)(N)
  block.Dwork(1).Name            = 'f';
  block.Dwork(1).Dimensions      = N;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
%% df �������һ��������ֵ�仯 (ά�����ƽ���������ͬ)(N)
  block.Dwork(2).Name            = 'df';
  block.Dwork(2).Dimensions      = N;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
%% a �����ĸ��ƽ���ת������ (ά�����ƽ���������ͬ)(degree)
  block.Dwork(3).Name            = 'a';
  block.Dwork(3).Dimensions      = N;
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;
%% dalpha �������һ����ת�Ǳ仯 (ά�����ƽ���������ͬ)(degree) 
  block.Dwork(4).Name            = 'dalpha';
  block.Dwork(4).Dimensions      = N;
  block.Dwork(4).DatatypeID      = 0;      % double
  block.Dwork(4).Complexity      = 'Real'; % real
  block.Dwork(4).UsedAsDiscState = true;
%% tau_r �����ʵ������������ (N;N;N*m)  
  block.Dwork(5).Name            = 'tau_r';
  block.Dwork(5).Dimensions      = 3;
  block.Dwork(5).DatatypeID      = 0;      % double
  block.Dwork(5).Complexity      = 'Real'; % real
  block.Dwork(5).UsedAsDiscState = true;
%% sTau QP�е������ɳڱ����൱��ʵ��������Ҫ������(������������)�Ĳ�ֵ(N;N;N*m) %��������ʵ�ʵ������ɳڱ��� ���д���
  block.Dwork(6).Name            = 'dtau';
  block.Dwork(6).Dimensions      = 3;
  block.Dwork(6).DatatypeID      = 0;      % double
  block.Dwork(6).Complexity      = 'Real'; % real
  block.Dwork(6).UsedAsDiscState = true;
%% s  ������Ԥ�ⲽ����ο�λ�õ������ɳ���(m;m;rad)
  block.Dwork(7).Name            = 's';
  block.Dwork(7).Dimensions      = 3;
  block.Dwork(7).DatatypeID      = 0;      % double
  block.Dwork(7).Complexity      = 'Real'; % real
  block.Dwork(7).UsedAsDiscState = true;
%% azimuth_line �����ƽ����������߶Σ�ά��Ϊ��ΪN����Ϊ�洢��N���ƽ���line�����Ķ���
  block.Dwork(8).Name            = 'azimuth_line';
  block.Dwork(8).Dimensions      = N;
  block.Dwork(8).DatatypeID      = 0;      % double
  block.Dwork(8).Complexity      = 'Real'; % real
  block.Dwork(8).UsedAsDiscState = true;
%% t_thrust_line �������Լ�ת�ص��߶Σ�ά��Ϊ��Ϊ2����Ϊ������ת�ظ���Ҫ1����
  block.Dwork(9).Name            = 't_thrust_line';
  block.Dwork(9).Dimensions      = 2;
  block.Dwork(9).DatatypeID      = 0;      % double
  block.Dwork(9).Complexity      = 'Real'; % real
  block.Dwork(9).UsedAsDiscState = true;
%% time ģ���ʱ��
  block.Dwork(10).Name            = 'time';
  block.Dwork(10).Dimensions      = 1;
  block.Dwork(10).DatatypeID      = 0;      % double
  block.Dwork(10).Complexity      = 'Real'; % real
  block.Dwork(10).UsedAsDiscState = true;
%% xkplus1_prd �������е�ǰʱ���ж��¸�ʱ�䲽��Ԥ��ֵ x(k+1|k) = [u v r x y psi]
  block.Dwork(11).Name            = 'xkplus1_prd';
  block.Dwork(11).Dimensions      = 6;
  block.Dwork(11).DatatypeID      = 0;      % double
  block.Dwork(11).Complexity      = 'Real'; % real
  block.Dwork(11).UsedAsDiscState = true;
%% xFinal_prd �����������һ��Ԥ�ⲽNp��ϵͳ״̬ x(k+Np|k) = [u v r x y psi]
  block.Dwork(12).Name            = 'xFinal_prd';
  block.Dwork(12).Dimensions      = 6;
  block.Dwork(12).DatatypeID      = 0;      % double
  block.Dwork(12).Complexity      = 'Real'; % real
  block.Dwork(12).UsedAsDiscState = true;
%% abnormalUtimes QP��̬�����Ĳ�������������
  block.Dwork(13).Name            = 'abnormalUtimes';
  block.Dwork(13).Dimensions      = 1;
  block.Dwork(13).DatatypeID      = 0;      % double
  block.Dwork(13).Complexity      = 'Real'; % real
  block.Dwork(13).UsedAsDiscState = true;
  
%% ForeceMomentK QP����������NumC���Ŀ������У���ת����Fx Fy Moment��
  block.Dwork(14).Name            = 'ForeceMomentK';
  block.Dwork(14).Dimensions      = 3*5; %ά��Ϊ3*NumC
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
%% Initialize Dwork �Ժ����ڵ�Dwork״̬��������ģ�⿪ʼǰ�ĳ�ʼ��
% ��MPC��������ص�Dwork���ó�ʼ�� ����������Ƕ��ƽ�����ʼ״̬��������
  
  block.Dwork(1).Data = 1e3 * block.DialogPrm(8).Data.*block.DialogPrm(10).Data; %Dwork1�洢���ƽ������� ����DialogPrm(8)��������λΪKN����˳���1e3����ΪN
  block.Dwork(3).Data = block.DialogPrm(9).Data;
  block.Dwork(13).Data(1) = 0;
  
  % �ƽ���������ֵ
  %L = [72 0; 65 0; -65 0; 55 0; -55 0; -72 20; -72 -20];     % �谴ʵ������
  L = [15.7 35.5; 47.02 24.58; 47.02 -24.58; 15.7 -35.5; -15.7 -35.5; ...
             -47.02 -24.58; -47.02 24.58; -15.7 35.5];     % �谴ʵ������ �˴���8��2�еľ��󣬵�һ�����ƽ�������������ϵ�µ�x����ֵ���ڶ�����y����ֵ
   
  block.Dwork(5).Data = thrusters_configuration(block.DialogPrm(9).Data*pi/180,L)*block.Dwork(1).Data;   %�ƽ������þ���B(alpha,L)��������ֵ�������õ���ǰ���������������(N,N,N*m)
  
  
%% plot animation
N = 8;  %quantity of the thrusters
if block.DialogPrm(11).Data == 1

    close(findobj('type','figure','name','Thrust & Azimuth Angle'))
    figure('Name','Thrust & Azimuth Angle','units','normalized','position',[0.1,0.1,0.8,0.8],'color',[0.8 1 0.8]);
    xlim([-150 150]);ylim([-150 150]); %�����position Name-Value pair������figure���λ�úͳ���
%   ����xlim ylim������ʹ�ú�������ͼ��������
    axis off  %������������
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
    rectangle('Position',[-62,-40,12,80],'linewidth',2.5,'facecolor','white');%��������
    plot(0,0,'o','markeredgecolor','k','markerfacecolor','k','markersize',8) %���������ĵ�
%% draw the azimuth thruster circles
    R = 10;

    center = [30 66; 85 46; 85 -46; 30 -66; -30 -66; -85 -46; -85 46; -30 66;]; %û��ʹ����ʵ���ƽ���������Ҫ����Ϊ�����㻭ͼ
    rectangle('Position',[center(1,1)-R,center(1,2)-R,2*R,2*R],'curvature',[1,1],...
        'linewidth',1.5,'edgecolor','r','facecolor','white');
    %����Բ��Ҳ������rectangle������position����;���һ������ʼ������ͳ��ȺͿ�ȣ�
    %'curvature',[1,1]�������������еĽ������ʾ�Ϊ1�����ﻭ��������Բ��ָ��ȫ��ת�ƽ���
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

    text(35,81,'No.1','fontname','Time New Rome','fontsize',12,'fontangle','oblique');    %fontangle oblique��ָ����Ϊб�壬�˴����ָ��ƽ������б��
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
    %�˴����ƽ�����ͼ�е����Ļ������ĵ㡣
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
   
    % ת��뾶��
    thrust_temp =  2 * 1e-3 * block.Dwork(1).Data * R/800;     % �������������800KN ���������������߶γ�2R �漰��ͼ�Ĵ��������������ض�����KN KN*m
    azimuth_temp = block.Dwork(3).Data * pi/180;  %�ƽ������洬����ϵ�еĽǶȣ�����ͼ��ƽ̨��x���곯�ң�����ֻ�ðѻ��Ȼ��ɽǶȼ��ɣ������ٵ����Ƕȡ�
    
    for i_temp = 1:N  
        block.Dwork(8).Data(i_temp) ...   %Dwork(8)���ƽ��������߶α�����ά��ΪN,line�ǻ����߶εĺ����������صĶ��󱻴洢��Dwork8��,�����������������޸�
            = line([center(i_temp,1) center(i_temp,1) + thrust_temp(i_temp) * cos(azimuth_temp(i_temp))],...
                   [center(i_temp,2) center(i_temp,2) + thrust_temp(i_temp) * sin(azimuth_temp(i_temp))],...
                   'linewidth',3,'color','k');
        %line�����﷨line(x����,y����),�˴�Ϊline([x1 x2],[y1 y2])
    end
    
    % �����������߶ξ����Dwork(9).Data(1),����߶γ�Ҳ��ȫ��ת���߶�һ��������
    block.Dwork(9).Data(1) = line([0 1e-3*block.Dwork(5).Data(1)/50],...
        [0 1e-3*block.Dwork(5).Data(2)/50],...
        'linewidth',5);
    %�����ǻ�һ���������ת�����ĸ�����,��ɫmΪ��ɫ����Ϊת���߶�theta�������-pi/2���Ը�����Ҫ����
    line([0 0],[-5 -10],'color','m');
    theta = -pi/2 : -0.01 : -pi/2 + 1e-3*block.Dwork(5).Data(3)/8e3*pi; %�����ֻ�е�ת��KNM������2pi*10^4*piʱ�Ż�ת�ص�ʾ���߶β�תһȦ�������20��
    if isempty(theta)
        theta = -pi/2 : 0.01 : -pi/2 + 1e-3*block.Dwork(5).Data(3)/8e3*pi; %���⵱block.Dwork(5).Data(3)Ϊ��ʱ������:-0.01:�ĸ�ֵ�﷨����thetaΪ��
    end    
    x = 0 + 7.5 * cos(theta);
    y = 0 + 7.5 * sin(theta);
    % ��ת�������߶ξ����Dwork(9).Data(2)    
    block.Dwork(9).Data(2) = line(x,y,'color','m','linewidth',5);
    % display current simulation time ��ʾ��ǰģ���ʱ�� Dwork(10)
    text(70,100,'Simulation Time :','fontsize',12);
    block.Dwork(10).Data(1) = text(150,100,'0','fontsize',12); %��Ϊ�����ǳ�ʼ������ʱ��Ϊ0
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
block.OutputPort(6).Data = block.Dwork(6).Data; %��������ʵ�ʵ������ɳڱ��� ���д���
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
%% �ú�����ģ���ʱ�䲽֮ǰ������ɢ��״̬����
tSample = 1; %Ԥ��ģ���ڲ���ʱ�䲽��С
%% MPC��������
%�˴�״̬����Ԥ��ģ�͵Ĳ�����������MPC�Ĳ���
numX = 6; %״̬���ĸ��� [u v r x y psi]
numP = 20;%MPCԤ�ⲽ�� ԭ�ض�λ�����Ļ� Ԥ�ⲽ����Ҳ����ûʲô����
numC = 5;%MPC���Ʋ��� ���Ʋ������ֵ��Ԥ�ⲽ��-1    
% Row = 10;%�ɳ�����
N = 8; %ȫ��ת�ƽ�������������ȷ����ȷ����ϵͳ�����������(ת�Ǻ�����)
h = 0;%MPCУ��ϵ�� h����[0,1] h=0ʱ�൱����ȫ����ģ�ͣ�h=1ʱ����ȫ������һ�������Ὣ��һ�������ȫ���ӵ���һ����
%%%%%%%%%
%ѡ�񽫻���ǰ��������ֱ�Ӽ���Ԥ��ģ����Ϊ����������ֱ���ڷ������ϼ���
SwitchFeedForward = 0; % SwitchFeedForwardΪ0ʱ��Ϊ��������Ϊ1ʱֱ���ڵ�ǰʱ�䲽����
%%%%%%%%%
Q = block.DialogPrm(4).Data;
R = block.DialogPrm(5).Data;
F = block.DialogPrm(6).Data;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ��������ThrustAllocation��������
% ���Ƕ�ת��Ϊ����
d2r = pi/180;
% �ɳ����ӵ�Ȩ����
% RelaxWeight = 1e8*diag([60 60 2000]);
% ȫ��ת�ٶȵ�Ȩ����
Omega = block.DialogPrm(12).Data;
% �ƽ���ȫ��ת�ٶ�����(rad/sample_time) ��s������sample_timeΪ1
da = [-2 2]*d2r;                                % �谴ʵ������
% �����ƽ��������ı仯����(N)
Frange = 1e3*[50 800];                              % �谴ʵ������
% �ƽ��������ı仯�ٶ�����(N/sample_time)
dF = 1e3*[-50 50];                                  % �谴ʵ������
% ���ƽ��������������ɵ�������
u0 = block.DialogPrm(10).Data;
% Semi 708 % �谴ʵ������ 
L = [15.7 35.5; 47.02 24.58; 47.02 -24.58; 15.7 -35.5; -15.7 -35.5; ...
             -47.02 -24.58; -47.02 24.58; -15.7 35.5]; 
% Semi 807
% L = [15.7 24.58; 47.02 35.5; 47.02 -35.5; 15.7 -24.58; -15.7 -24.58; -47.02 -35.5; -47.02 35.5; -15.7 24.58;];     
% Semi 981
% L = [47.02 35.5; 47.02 24.58; 47.02 -24.58; 47.02 -35.5; -47.02 -35.5; -47.02 -24.58; -47.02 24.58; -47.02 35.5;]; 
%��һʱ�䲽����
f0 = block.Dwork(1).Data; 
%��һʱ�䲽������
a0 = block.Dwork(3).Data; 
% ��һ���ƽ���ת�ǻ�Ϊ����
a0 = a0 * d2r;
% Ŀ�꺯������λ�óͷ�����ӳ���
pp = 10;
% Ŀ�꺯������λ�óͷ����ĸ�еĳ���
ee = 1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Get_Coefficients���ڸ����漰�󵼵�ϵ������ ����dTf��MPC��Ҳ��Ҫ�õ�

[dTf, ddT] = Get_Coefficients(f0,a0,pp,ee);
%% �ο��켣���� 
%����������Ĺ켣���٣�Ӧ��Ҫ�ȶ���δ��һ���ʱ�䲽��λ�ú��ٶ���Ϣ
%���˴��������ϸ�����Ĺ켣���٣�ֻ�Ƕ�����λ�Ķ�λ�㲻��һֱ��ͬһ���ط�����
%���ڶ�����λ���⣬�����й滮��δ�����ٶȲο��㣬�����������в�Ӧ�����ٶȣ���ֻ�������������к����ٶ��
%��Ϊ����ϣ�����������λ�㴦�����ٶȣ������������ͨ���趨Ȩ�ؾ���Q��Fʵ��
x_k = [block.InputPort(3).Data; block.InputPort(2).Data]; %�������л�õĵ�ǰ״̬
posRef = zeros(3,1); %�洢�ο�״̬ ǰ3��Ԫ�����ٶ�״̬����3��Ԫ����λ��״̬
speedRef = zeros(3,1);
posRef(:) = block.InputPort(1).Data; %posRef�ǲο�λ�ã���ʵ�ڵ�ǰ�ű���û�õ������ο�λ�ò���0,0,0��ʱ�����posRefҪ���ϣ�ͬʱ����ҲҪ�޸�
speedRef(1:3) = 0;%�ο����ٶ�״̬��Ȼ��0����Ȼ�޷�����λ�ò��䣬���ǿ��Կ�����Ȩ�ؾ������漰���ٶȵ�Ԫ�ص�С����������ٶȵĿ��ǡ� 
% dotXRef = [0 0 0 0 0 0]';%�ο����ٶȺ�λ�õ�ʱ�䵼 ���ο��켣����������ϵ�¼��ٶȺ��ڴ������ϵ�µ��ٶ�
% dotXRef = repmat(dotXRef,numP,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%
Xr = zeros((numP+1)*numX,1);
%1.�����ɲο��켣
Xr(:) = repmat([speedRef;posRef],numP+1,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%2.�����ữ�������ɲο��켣
%�ⲿ�ֲο��켣�ǻ᲻���Զ����ɵ�������ǿ³����
%��ȻĿǰû�н��жԲο��켣�Ŀ����Է���
% Xr(1:numX) = x_k; %[u v r x y psi]
% softAlpha = 0.8; %���ữ����softAlphaΪ0ʱ��û��ƽ��Ч����softAlphaԽ��ϵͳ����Խ��³����Խǿ�������Ա�
% for j = 1:numP
%     rowIndex1 = (4:6) + numX*j;
%     rowIndex2 = rowIndex1 - 3;
%     Xr(rowIndex1) = softAlpha^j * x_k(4:6) + (1-softAlpha^j) * posRef; %Xr����QP����е�������
%     if j >=2                                                               
%     psik = Xr(rowIndex1(end)-numX);
%     Jpsi = [cos(psik) -sin(psik) 0; sin(psik) cos(psik) 0;0 0 1];
%     Xr(rowIndex2-numX) = Jpsi\((Xr(rowIndex1) - Xr(rowIndex1-numX))/tSample);
%     end
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ���ڲ��������˵̩��չ���������
% x_tilde��״̬��x(k)[u v r x y psi]��̩��չ����Ĳչ����Ͷ��ڵ�ǰ����Ĳο�״̬����
% ��x_tilde(k)�ڴ������и�Ԫ�ؾ���0  
taylorPoint = [speedRef;posRef];  %���ֱ�ӽ�̩��չ���㶨�ڲο����ϣ���������x_tilde��������� 
% taylorPoint = x_k;
x_tilde = x_k-taylorPoint; %x_tilde׼ȷ��˵��x_tilde(k)   

X_taylor = zeros((numP+1)*numX,1);
X_taylor(:) = repmat(taylorPoint,numP+1,1);   %X_taylor����QP����е�������

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% �����ʼ��    
M_rigidbody = block.DialogPrm(1).Data;
M_addition = block.DialogPrm(2).Data;
mass = M_rigidbody(1,1);
%Crb = [0 0 -m*v;0 0 m*u; m*v -m*u 0];
%Ca = [0 0 -a_22*v-a_26*r;0 0 a_11*u;a_22*v+a_62*r -a_11*u 0];
%partialC�Ƕ�Crb+Ca���ſɱȾ���
a_11 = M_addition(1,1);
a_22 = M_addition(2,2);
%%%%%M���������3��3��ˮƽ��M������6��6�ģ����������3����6
a_26 = M_addition(2,3);%�ڸ�������a_26Ϊ0
a_62 = M_addition(3,2);
u_taylor = taylorPoint(1);
v_taylor = taylorPoint(2);
r_taylor = taylorPoint(3);
psi_taylor = taylorPoint(6);
partialC = [0  -(mass+a_22)*r_taylor  -(mass+a_22)*v_taylor-2*a_26*r_taylor;...
            (a_11+mass)*r_taylor  0  (a_11+mass)*u_taylor;...
            (a_22-a_11)*v_taylor+a_62*r_taylor  (a_22-a_11)*u_taylor  a_62*u_taylor];
Dp = block.DialogPrm(3).Data; %���Ե��������
%R = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0;0 0 1];
partialR = [cos(psi_taylor) -sin(psi_taylor) 0 0 0 -u_taylor*sin(psi_taylor)-v_taylor*cos(psi_taylor);...
            sin(psi_taylor) cos(psi_taylor) 0 0 0 u_taylor*cos(psi_taylor)-v_taylor*sin(psi_taylor);...
            0  0  1  0  0  0];
temp = -1*(M_rigidbody+M_addition)\(partialC + Dp); %A_tilde�����Ͻ�3*3����
A_tilde = zeros(6,6);
A_tilde(1:3,1:3) = temp;
A_tilde(4:6,:) = partialR;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%���ڴ˴�����MPC�������������Ǻ���(����Ϊ3��������3DOF)�����������ƽ����������ͽǶ�
%��ԭ���ĺ���Tau���ڱ����Tau = B(alpha)u��alpha���ƽ���ת�Ƕ�u��������
%���µ�Tau�����Ҳ���Կ������Ǹ���������(�����alpha��u���)�������Ҫ���Ի��� 
% B(alpha)u = B(alpha0)u0 + B(alpha0) * (u-u0) +
% partial(B(alpha0)*u0)/partial(alpha) * (alpha-alpha0)
% ��u_tilde��alpha_tilde���е�һ�����1��2*N����ϵͳ��������� 
% u0��alpha0��Ϊ��һʱ�䲽��������ת�� ֻ��Ҫ��ȡDwork1��3���� ��Ϊ��ǰDwork��û����
% �µĹ������1-8Ϊu_tilde��9-16Ϊalpha_tilde
B_tilde = zeros(6,2*N);
%ֻ�޸�ǰ3��
B_tilde(1:3,1:N) = thrusters_configuration(a0,L);
B_tilde(1:3,1+N:2*N) = dTf;
B_tilde(1:3,1:2*N) = (M_rigidbody + M_addition)\B_tilde(1:3,1:2*N);
    
%����ľ�����������״̬�ռ䷽���еģ�������о�����ɢ��
A_tildeDisc = eye(6) + tSample*A_tilde;  %��ʵ��e��At�η��Ľ���
B_tildeDisc = tSample*B_tilde;  
    
%�����Ǹ�����ĳ�ʼ��M����Matrix_M��M��������״̬�ռ��е�A���󹹳ɵ�
rowA = size(A_tildeDisc,1); %rowA��״̬��ά������ͬ��
colB = size(B_tildeDisc,2);
Matrix_M = [eye(rowA);zeros(numP*rowA,rowA)];
Matrix_C = zeros((numP+1)*rowA,colB*numC); 
Matrix_I = [eye(rowA);zeros(numP*rowA,rowA)];
%�������M��C��I
temp1 = eye(rowA);
for i = 1:numP
    rowIndex = (1:rowA)+i*rowA;
    Matrix_M(rowIndex,:) = A_tildeDisc*temp1; 
    Matrix_C(rowIndex,:) = [temp1*B_tildeDisc,Matrix_C(rowIndex-rowA,1:end-colB)];
    %%%%%%%%%%%%%%%
    %���ڱ�ϵͳ�������������U�����������������仯������˶�Matrix_C�������Ʋ���Ӱ����������ж��⴦�����鿴Notability��MPC��7ҳ��
    %����˵�������Ʋ����Ĳ��ְ���ԭ����Matrix_C������U��Ĭ��Ϊ0������ϵͳϣ���ǳ������Ʋ����Ĳ���U��Ȼά�����һ��������Uֱ��Ԥ�ⲽ������
    %�����������仯��ΪU������������deltaUn��ʽ��������������Դ�Notability�л���״̬�ռ䷽�̵�MPC�п�������һ����չ״̬�ռ�ģ��
    if i > numC %�������Ʋ���Ӱ������� ֻ�����һ���п飨colB�� 
        Matrix_C(rowIndex,end-colB+1:end) = Matrix_C(rowIndex,end-colB+1:end) + Matrix_C(rowIndex-rowA,end-colB+1:end);
    end
    %%%%%%%%%%%%%%%
    Matrix_I(rowIndex,:) = eye(rowA);
    temp1 = A_tildeDisc*temp1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ����Q_bar��R_bar,������R_bar����Ϊ���벻��ֻ��������������ȫ��ת������ת�� RO_bar��R��Omega��ͬ��ɵ�����Ȩ�ؾ���
RO = blkdiag(R,Omega);
R = blkdiag(R,zeros(size(Omega,1)));
s_Q = size(Q,1);   %�����Q��ά�� 
s_R = size(R,1);   %�����R��ά��
s_RO = size(RO,1);
Q_bar = zeros(s_Q*(numP+1),s_Q*(numP+1));
for i = 1:numP
    Q_bar(1+(i-1)*s_Q:i*s_Q,1+(i-1)*s_Q:i*s_Q) = Q; %Q_bar�ĶԽ�����ȫ��Q��
    %���һ��Q�����Լ���һ��FȨ�ؾ���,����ר�ŵ������һ����x(k+N|k)��Ȩ�ء�
end
Q_bar(numP*s_Q+1:(numP+1)*s_Q,numP*s_Q+1:(numP+1)*s_Q) = F;
    
R_bar = zeros(s_R*(numC),s_R*(numC)); %R_bar��������䳤����أ����������ΪԤ������-1
for i = 1:numC
    if i == numC
        R_bar(1+(i-1)*s_R:i*s_R,1+(i-1)*s_R:i*s_R) = (numP-numC+1)*R; 
    else
        R_bar(1+(i-1)*s_R:i*s_R,1+(i-1)*s_R:i*s_R) = R;
    end
end

RO_bar = zeros(s_RO*(numC),s_RO*(numC)); %R_bar��������䳤����أ����������ΪԤ������-1
for i = 1:numC
    if i == numC
        RO_bar(1+(i-1)*s_RO:i*s_RO,1+(i-1)*s_RO:i*s_RO) = (numP-numC+1)*RO; 
    else
        RO_bar(1+(i-1)*s_RO:i*s_RO,1+(i-1)*s_RO:i*s_RO) = RO;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% �����Է�����ģ�����Ի�����ĳ�����f(x_r,u_r)-dot(x_r) ����dot(x_r)���趨�Ĳο�ֵ��ʱ�䵼
%���f(x_r,u_r)����ľ���
Crb = [0 0 -mass*v_taylor;0 0 mass*u_taylor; mass*v_taylor -mass*u_taylor 0];
Ca = [0 0 -a_22*v_taylor-a_26*r_taylor;0 0 a_11*u_taylor;a_22*v_taylor+a_62*r_taylor -a_11*u_taylor 0];
Rpsi = [cos(psi_taylor) -sin(psi_taylor) 0; sin(psi_taylor) cos(psi_taylor) 0;0 0 1];  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Tau_env = block.InputPort(6).Data; %��������ϵ�µĻ����� block.InputPort(6)�Ƿ�����  block.InputPort(5)��obsSlowVaringEnvForces 
% Tau_env = block.InputPort(5).Data;
Tau_env = 0;
Tau_0 = thrusters_configuration(a0,L) * f0;%��̩��չ��ʱ��������չ���㲻ѡ��Ϊ0ʱ������Ϊ0

if (block.DialogPrm(7).Data == 1) && (SwitchFeedForward == 0)
    Tau_env = Tau_env + block.InputPort(4).Data; %block.InputPort(4)��feedForwardForces
end

nv_taylor = taylorPoint(1:3); 
fx0u0 = zeros(numX,1);
fx0u0(1:3) = (M_rigidbody + M_addition)\(Tau_env + Tau_0 -(Crb + Ca + Dp) * nv_taylor);
fx0u0(4:6) = Rpsi * nv_taylor;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% f(x_r,u_r)����ʱ��仯�������������ǰ�����൱����Ϊ������һֱ�Ը�ֵ����������δ��NumP����
fx0u0 = repmat(fx0u0,numP,1);
% ����
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% f(x_r,u_r)��ʱ��仯�������������ǰ�����൱����Ϊ������ֻ���������ڵ�ǰʱ�䲽��
% if (block.DialogPrm(7).Data == 1) && (SwitchFeedForward == 0)
%     fx0u0 = zeros(numX,1);
%     fx0u0(1:3) = (M_rigidbody + M_addition)\(Tau_env + Tau_0 -(Crb + Ca + Dp) * nv_taylor);
%     fx0u0(4:6) = Rpsi * nv_taylor;
%     fx0u0(7:9) = (M_rigidbody + M_addition)\(Tau_env + Tau_0 - block.InputPort(4).Data -(Crb + Ca + Dp) * nv_taylor);
%     fx0u0(10:12) = Rpsi * nv_taylor;
%     fx0u0(7:numP*numX) = repmat(fx0u0(7:12),numP-1,1);
% end
%����
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% finalBias = fx0u0 - dotXRef;
finalBias = fx0u0;
Xbias = zeros((numP+1)*rowA,1);  %Xbias�ĵ�1-6��Ϊ0 ��Ϊ�⼸���ǵ�ǰkʱ�̵�״̬

for i = 1:numP
    rowIndex = (1:rowA)+i*rowA;
    Xbias(rowIndex) = A_tildeDisc * Xbias(rowIndex-rowA) + tSample*finalBias(rowIndex-rowA);  %Xbias����QP����е�������
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% MPCģ�����������
%     Y_cor=Y0+H*e;
%��ǰʱ�̵�״̬y����һʱ��Ԥ��ĵ�ǰʱ�̵�״̬y�Ĳ�Ϊerror������x_k�ĺ�����x,y,psi����
%     error = zeros(numX,1);
%���ڵ�һ������updateʱ,block.Dwork(11).Data�洢�Ļ�������һʱ��Ԥ���״̬,���Դ�ʱerror����ȷ��Ӧֱ����errorΪ0
if block.CurrentTime == 0
    error = zeros(numX,1);
else
    error = x_tilde(1:6) - block.Dwork(11).Data;
end

for i = 1:1:numP+1 %��ΪX_k�ĵ�һ����������x(k|k)������numP+1
    if i == 1
        H_cor(1+(i-1)*numX:i*numX,:) = zeros(numX,numX);
    else
        H_cor(1+(i-1)*numX:i*numX,:) = h*eye(numX);
    end
 end
X_cor = H_cor*error; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% �������䲿��Thrust Allocation 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ����ȫ��ת�ƽ�����dda �Ҹ�ȫ��ת�ƽ�������ת�ٶ���ͬ (���ڽ�������������MPC�У����������������ǵ�ʱ�䲽�ģ�����numC����) ���ffҪ���¸���
dda = zeros(numC*N,2); 
for i = 1 : N
    dda(i,:) = da;  
end

dda(N+1:end,:) = repmat([-1e8*pi,1e8*pi],(numC-1)*N,1); 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%�����н�ֹ�������dda
% % ��ֹ�����ú�Ŀ������䣬1��4�ƽ���������������
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
% % �����������䣬���Ƕ������ƽ���������ѡ��δ�����ܼ��붯̬��ֹ��
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
% % �ƽ���ȫ��ת�ٶ�Լ������
% dda = 1000*ones(N,2);     % ��ʼ��
% % a0����[-200deg 200deg]֮�� ��[-1.111pi 1.111pi]֮�� a0����Ҳ��angel_sector��Χ��
% % angle_sector��ֵ��[-pi pi]֮��
% % ���ȿ��ǿ����������ϲ��Ƕȵ�����,����ddaת��[-2pi 2pi],�Ա���da�Ƚ�
% for i = 1 : N
%     dda(i,:) = angle_sector(i,:) - a0(i);  %dda��[-2.111pi 2.111pi]
%     while(dda(i,2) + 100*eps < 0)
%         dda(i,:) = dda(i,:) + 2*pi;
%     end
%     while(dda(i,1) - 100*eps > 0)
%         dda(i,:) = dda(i,:) - 2*pi;
%     end
% end
% % ������������dda��[-2pi 2pi]
% % ��ȫ��ת�ƽ���ÿ�������ת��da�Ƚϣ�ѡ����С�ı仯����
% for i = 1 : N
%     if da(1) > dda(i,1)
%         dda(i,1) = da(1);
%     end
%     if da(2) < dda(i,2)
%         dda(i,2) = da(2);
%     end
% end
%�����н�ֹ�������dda
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% �ƽ��������仯����(���ڽ�������������MPC�У����������������ǵ�ʱ�䲽�ģ�����numC����) ���ffҪ���¸���
F = [ Frange(1)*u0 Frange(2)*u0 ]; % ����ʧЧ���������������仯���� F��N��2�еľ���
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
    if sum(countTable,'all') == 2*N %���еĿ��Ʋ����½綼� �����仯����������
        toBeFilled = i*N;
        break                   
    end
end
if (toBeFilled ~= -1) && (toBeFilled ~= numC*N)
    ff(toBeFilled+1:end,1) = repmat(F(:,1),numC-i,1);
    ff(toBeFilled+1:end,2) = repmat(F(:,2),numC-i,1);
end

%���˴�ff�������T+deltaT�����½硢ԭ�������������Ҫ�ľ���T+deltaT�����˺�����Ҫ����deltaT
ff_old = ff;
ff = ff - repmat(f0,numC,1);


%% QP
%Ϊ���ι滮����׼��ϵ�� %Ŀ�꺯�� = 1/2*x'*H*x + fQP'*x 
%��ǰ������ʽ������Ŀǰ��չ����ʽ������x_tilde(k|k)=0������x(k|k) =x_tilde(k|k)+x_taylor����x_taylor����x(k|k)
% G = Matrix_M'*Q_bar*Matrix_M; %G�����ڳ�����xGx�����Բ���Ҫ�ŵ����Ż��д���
E = Matrix_M'*Q_bar*Matrix_C;
H = Matrix_C'*Q_bar*Matrix_C + RO_bar;  %(2N*numP)*(2N*numP)�ľ���
%������������������ע��һ������ ����������HӦ���ǶԳƾ��� ��Ϊ�ҵ�Qbar��Rbar���ǶԳƵ� ���ǿ�����ΪĳЩ��ֵ��������
%matlab��ʾQP��ⳬʱ��H�����ǶԳƾ��󣬽��г��Ժ󣬷���ȷʵ���ǣ���H-H'��������и���Ԫ����1e-22��������ֵ��
%���ʹ��H = (H+H')/2��H��Ϊ�ϸ�ԳƵľ���,ͬʱģ���д�����û�м�����Ķ��ι滮
H = (H+H')/2;
% H = blkdiag(H,RelaxWeight); % H�����ɳ�����s��λ���ĵľ���s=[s1;s2;s3]��ά�ȱ�Ϊ(2N*numP+3) * (2N*numP+3)
%%%%%%%%%%%%%%%%%%%%%%%%%%
%����QP����������ԭ��Thrust_Allocation�����ڵ�[T0+deltaT;deltaAlpha;s]�����
%[deltaT;deltaAlpha;s]�������QP����������Ҫ���(repmat([f0;zeros(N,1)],numC,1)'*R_bar)'��f0����һʱ�䲽������������
%QPĿ�꺯��������ϵ��
fbias = (Xbias'*Q_bar*Matrix_C)';
f_taylor = (X_taylor'*Q_bar*Matrix_C)';
f_reference = (Xr'*Q_bar*Matrix_C)';
f_cor = (X_cor'*Q_bar*Matrix_C)';

fQP = zeros(2*N*numC,1);
fQP(1:2*N*numC) = (x_tilde'*E)'+ fbias + f_taylor - f_reference + f_cor + (repmat([f0;zeros(N,1)],numC,1)'*R_bar)';
%f0��ԭ����Ҳ��������չ���㣬���ڿ���f0+df�����������ģ���˴˴������������
%%%%%%%%%%%%%%%%%%%%%%%%%%%
%ddT���ڷ�ֹ�����Ƕ�����
fQP(1:2*N*numC) = fQP(1:2*N*numC) + repmat([zeros(N,1);0.5*ddT'],numC,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%QP���Բ���ʽԼ��,���ó�����Ԥ�ⲽ����Ŀ�����һ����Χ�� M*x_tilde + C * Uk + Xconstant��Xbias��+ X_taylor - Xr + s <= [xErrorConstraint yErrorConstraint psiErrorConstraint]';
A = Matrix_C(end-2:end,:);
XcFinal = Xbias(end-2:end);
Mx_tilde = Matrix_M(end-2:end,:) * x_tilde;
errorFinalConstraints = [60.0;60.0;60*pi/180];
b = errorFinalConstraints - XcFinal - Mx_tilde - X_taylor(end-2:end) + Xr(end-2:end);
%%%%%%%%%%%%%%�������д��뽫����������Ͻ�Լ������ԳƵ��½�Լ��
A = [A;-Matrix_C(end-2:end,:)];
b = [b;errorFinalConstraints + XcFinal + Mx_tilde + X_taylor(end-2:end) - Xr(end-2:end)];
%!!!!!!!!!Լ��Ӧ�������� ��Ϊһ�����ɶ���Ҫ������Լ��(�Ͻ���½�) A*x<=b Ax>=-b -Ax<=b
%%%%%%%%%%%%%%����ʱ���������±仯����(��ֵ) <= deltaU(n+1)-deltaU(n) <= ����ʱ���������ϱ仯����(��ֵ) ����ʱ��ת�����±仯���� <= deltaAlpha(n+1)-deltaAlpha(n) <= ����ʱ���������ϱ仯���� 
subMatrix = zeros((numC-1)*(2*N),numC*(2*N));
for i = 1:numC-1
    rowIndex = (1:2*N) + (i-1)*(2*N);
    subMatrix(rowIndex,rowIndex) = -1 * eye(2*N);
    subMatrix(rowIndex,rowIndex + 2*N) = eye(2*N);
end
upperBound = repmat([dF(2)*ones(N,1);da(2)*ones(N,1)],numC-1,1);  %֮ǰ�������ͳ������� ֮ǰda(1)da(2)�����dda(1) dda(2)
lowerBound = -1 * repmat([dF(1)*ones(N,1);da(1)*ones(N,1)],numC-1,1);   
A = [A;subMatrix;-subMatrix];
b = [b;upperBound;lowerBound];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%QP���Ե�ʽԼ�� Aeq beqΪ������Ϊ��������QP����û�е�ʽԼ����ֻ�����Ե�ʽԼ���ͱ��������ޡ�
Aeq = [];
beq = [];
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%�����������½� %��Ϊ����deltaU ���½�û��ô�� ���ڵ�������UandAlpha��ΪnumC*2*N,�����п��Ʋ����ڵ�������ת��
%�����������õ�lb ubӦ���ǳ���Ϊ2*N*numC + 3������
ffdda = zeros(2*N*numC,2);
for i = 1:numC
    rowIndex = (1:2*N) + (i-1)*2*N;
    rowIndex2 = (1:N) + (i-1)*N;
    ffdda(rowIndex,:) = [ff(rowIndex2,:);dda(rowIndex2,:)];
end

lb = ffdda(:,1); %�涨��deltaT���ϡ��½�(ff) ��s�����½磨+-1e10��
ub = ffdda(:,2);
%������ x0 ��ʼ�������QP���� 
% x0 = [zeros(N,1);zeros(N,1);zeros(3,1)]; 
x0 = [];
options = optimset('Algorithm','interior-point-convex','MaxIter',100); %�ڵ㷨��� Ĭ������������Ϊ200
Ualpha_k = quadprog(H,fQP,A,b,Aeq,beq,lb,ub,x0,options); %fmincon ������Ŀ�꺯���Ż�
%%%%%%%%%%%%%%%%%%%%%%%%
%�򵥿��ټ���һ��QP����������NumC���Ŀ�������f
B_UalphaTemp = (M_rigidbody + M_addition) * B_tilde(1:3,:);
B_Ualpha = blkdiag(B_UalphaTemp,B_UalphaTemp,B_UalphaTemp,B_UalphaTemp,B_UalphaTemp);
deltaTauSeries = B_Ualpha * Ualpha_k;
ForeceMomentK = deltaTauSeries + repmat(Tau_0,numC,1);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% QP������
df = Ualpha_k(1:N); %df �������һ��������ֵ�仯 (N)
f = f0 + df;  
count = find(f > Frange(2)); %�߼�ֵcount��ʾf�Ƿ񳬳��������
if count
    f = f0; %ά��ԭ����  
    df = 0*zeros(N,1);
    block.Dwork(13).Data(1) = block.Dwork(13).Data(1) + 1;
end

dalpha = Ualpha_k(N+1:2*N);
a = a0 + dalpha;
dalpha = dalpha / d2r;  %dalpha �������һ����ת�Ǳ仯 (degree)
a = a / d2r;  %a �����ĸ��ƽ���ת������ (degree)
%% ��aת��[-pi pi]���䣬�Է��㻭ͼ  ����[-pi pi]����ȴ����200 -200Ϊ������Ӧ���ǵ���ģ�������a��i����ֵ��pi��-pi������
for i = 1 : N
    while a(i) > 200
        a(i) = a(i) - 360;
    end
    while a(i) < -200
        a(i) = a(i) + 360;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ������Ԥ���ʵ�������͸�ʱ�̼����δ��״̬
Balpha = thrusters_configuration(a*d2r,L); %Balpha�������þ���
tau_r = Balpha * f; %  tau_r �����ʵ���������� (N;N;N*m)
% dtau = tau_r - tau; %  dtau ʵ��������Ҫ�������Ĳ�ֵ
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
X_k = Matrix_M * x_tilde + Matrix_C * Ualpha_k + Xbias; %�˴�X_k��x_tilde��ʽ��
xkplus1_prd = X_k(numX+1:2*numX); %ע�����x_prdҲ��x_tilde��ʽ�ģ���x(k+1|k)-x(taylorpoint in k step)��
X_k_real = X_k + X_taylor; %X_k_real��״̬��x�ڴӵ�ǰ��δ��P����ɵ�����
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
% %% ��MPC����ϲ���ǰ����
% %%��ģ��Ŀǰ�����ںܶ����⣬���ܸ�QP����й�ϵ��ĿǰӦ��������֪��Ϊʲô������ǰ��MPC��QP�������ƽ������������������ʹĿǰ�Ѿ�����
% if (block.DialogPrm(7).Data == 1) && (SwitchFeedForward == 1)
%     %tau_target          ������Ҫ����������� ǰ��update������������ϲ���ǰ����  (N;N;N*m)
%     tau_target = tau_r - 1 * block.InputPort(4).Data;
%     %�ɳ�����
%     relaxWeight2 = 1e10*diag([30 30 10]);
%     % ����Ȩ����
%     R2 = block.DialogPrm(5).Data;
%     % ȫ��ת�ٶȵ�Ȩ����
%     Omega2 = Omega;
%     dda2 = zeros(N,2);
%     for i = 1 : N
%         dda2(i,:) = da;
%     end
%     % �ƽ����µ������仯����
%     ff2 = ff_old(1:N,:);
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     %����ϵ��
%     T =  thrusters_configuration(a0,L);
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     % Ϊ���ι滮����׼��ϵ��
%     H = blkdiag(2*R2,2*Omega2,2*relaxWeight2);
%     I = [zeros(N,1);ddT';zeros(3,1)];
%     A = [];
%     b = [];
%     Aeq = [T,dTf,eye(3)];
%     beq = tau_target  ;
%     lb = [ff2(:,1);dda2(:,1);-1e10*ones(3,1)];
%     ub = [ff2(:,2);dda2(:,2); 1e10*ones(3,1)];
%     x0 = [f0;zeros(N,1);zeros(3,1)];   %% ��������ʽ��[Thrust0+dThrust dAngle s ]
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     % �����ι滮����
%     options = optimset('Algorithm','interior-point-convex','Display','off');
%     solution1 = quadprog(H,I,A,b,Aeq,beq,lb,ub,x0,options);
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     % �������
%     f = solution1(1:N);
%     df = f - f0;
%     dalpha = solution1(N+1:2*N);
%     a = a0 + dalpha;
%     dalpha = dalpha / d2r;  %dalpha �������һ����ת�Ǳ仯 (degree)
%     a = a / d2r;  %a �����ĸ��ƽ���ת������ (degree)
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%     %��aת��[-pi pi]���䣬�Է��㻭ͼ  ����[-pi pi]����ȴ����200 -200Ϊ������Ӧ���ǵ���ģ�������a��i����ֵ��pi��-pi������
%     for i = 1 : N
%         while a(i) > 200
%             a(i) = a(i) - 360;
%         end
%         while a(i) < -200
%             a(i) = a(i) + 360;
%         end
%     end
%     Balpha = thrusters_configuration(a*d2r,L); %Balpha�������þ���
%     tau_r = Balpha * f; %  tau_r �����ʵ���������� (N;N;N*m)
%     dtau = tau_r - tau_target; %  dtau ʵ��������Ҫ�������Ĳ�ֵ
%     
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ��ָ��������Dwork��������
block.Dwork(1).Data = f;
block.Dwork(2).Data = df; 
block.Dwork(3).Data = a;
block.Dwork(4).Data = dalpha;
block.Dwork(5).Data = tau_r;
if (block.DialogPrm(7).Data == 1) && (SwitchFeedForward == 1)
    block.Dwork(6).Data = dtau; %��������ʵ�ʵ������ɳڱ��� ���д���
else
    block.Dwork(6).Data = zeros(3,1);
end
block.Dwork(7).Data = zeros(3,1);
block.Dwork(11).Data = xkplus1_prd;
block.Dwork(12).Data = xFinal_prd;
block.Dwork(14).Data = ForeceMomentK;

%% plot animation ��InitializeConditions�������Ѿ������˻�ͼ����
if block.DialogPrm(11).Data == 1
    center = [30 66; 85 46; 85 -46; 30 -66; -30 -66; -85 -46; -85 46; -30 66;];
    Radius = 10;
    % ת��뾶��
    thrust_temp = 1e-3 * 2*block.Dwork(1).Data * Radius/800;     
    azimuth_temp = block.Dwork(3).Data * pi/180;
    %Dwork(8)�д洢���Ǹ��ƽ��������߶εĶ��� ��set���� set(Dwork(8).Data,'xdata',[x1 x2],'ydata',[y1 y2])
    for i_temp = 1:8
        set(block.Dwork(8).Data(i_temp),...
            'xdata',[center(i_temp,1) center(i_temp,1) + thrust_temp(i_temp) * cos(azimuth_temp(i_temp))],...
            'ydata',[center(i_temp,2) center(i_temp,2) + thrust_temp(i_temp) * sin(azimuth_temp(i_temp))]);
    end
    %Dwork(9)�д洢���Ǻ����߶κ�Ť��,Dwork(5).Data(2)�Ǻ�����y�������,Dwork(5).Data(1)�Ǻ�����x�������
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
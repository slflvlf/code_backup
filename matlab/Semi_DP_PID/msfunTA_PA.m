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
N = 8;  %quantity of the thrusters
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
N = 8;  %quantity of the thrusters
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
        L = [15.7 35.5; 47.02 24.58; 47.02 -24.58; 15.7 -35.5; -15.7 -35.5; ...
             -47.02 -24.58; -47.02 24.58; -15.7 35.5];     % 需按实际设置
        a1 = a(1); a2 = a(2); a3 = a(3); a4 = a(4); a5 = a(5); a6 = a(6); a7 = a(7); a8 = a(8);
        T = [cos(a1) cos(a2) cos(a3) cos(a4) cos(a5) cos(a6) cos(a7) cos(a8);
             sin(a1) sin(a2) sin(a3) sin(a4) sin(a5) sin(a6) sin(a7) sin(a8);
             L(1,1)*sin(a1)-L(1,2)*cos(a1) L(2,1)*sin(a2)-L(2,2)*cos(a2)...
             L(3,1)*sin(a3)-L(3,2)*cos(a3) L(4,1)*sin(a4)-L(4,2)*cos(a4)...
             L(5,1)*sin(a5)-L(5,2)*cos(a5) L(6,1)*sin(a6)-L(6,2)*cos(a6)...
             L(7,1)*sin(a7)-L(7,2)*cos(a7) L(8,1)*sin(a8)-L(8,2)*cos(a8)];
    end
  %% plot animation
if block.DialogPrm(3).Data == 1
    close(findobj('type','figure','name','Thrust & Azimuth Angle'))
    figure('Name','Thrust & Azimuth Angle','units','normalized','position',[0.1,0.1,0.8,0.8],'color',[0.8 1 0.8]);
    xlim([-150 150]);ylim([-150 150]);
    axis off
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
    rectangle('Position',[-62,-40,12,80],'linewidth',2.5,'facecolor','white');
    plot(0,0,'o','markeredgecolor','k','markerfacecolor','k','markersize',8)
%% draw the azimuth thruster circles
    N = 8;  %quantity of the thrusters
    R = 10;
    center = [30 66; 85 46; 85 -46; 30 -66; -30 -66; -85 -46; -85 46; -30 66;];
    rectangle('Position',[center(1,1)-R,center(1,2)-R,2*R,2*R],'curvature',[1,1],...
        'linewidth',1.5,'edgecolor','r','facecolor','white');
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
    text(35,81,'No.1','fontname','Time New Rome','fontsize',12,'fontangle','oblique');
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
%% draw forbidden sector
%    d2r = pi/180;
    %% azimuth 1
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
%% azimuth 2
    
%% azimuth 3
   
%% azimuth 4
   
%% azimuth 5
    
%% azimuth 6

%% azimuth 7
    
%% azimuth 8

%%
    thrust_temp = 2*block.Dwork(1).Data * R/800;     % 转入半径内
    azimuth_temp = block.Dwork(3).Data * pi/180;
    for i_temp = 1:N
        block.Dwork(7).Data(i_temp) ...
            = line([center(i_temp,1) center(i_temp,1) + thrust_temp(i_temp) * cos(azimuth_temp(i_temp))],...
                   [center(i_temp,2) center(i_temp,2) + thrust_temp(i_temp) * sin(azimuth_temp(i_temp))],...
                   'linewidth',3,'color','k');
    end
    block.Dwork(8).Data(1) = line([0 block.Dwork(5).Data(1)/50],...
        [0 block.Dwork(5).Data(2)/50],...
        'linewidth',5);
    line([0 0],[-10 -5],'color','m');
    theta = -pi/2 : 0.01 : -pi/2 + block.Dwork(5).Data(3)/8e3*pi;
    if isempty(theta)
        theta = -pi/2 : -0.01 : -pi/2 + block.Dwork(5).Data(3)/8e3*pi;
    end
    x = 7.5 * cos(theta);
    y = 7.5 * sin(theta);
    block.Dwork(9).Data(1) = line(x,y,'color','m','linewidth',5);
    % display current simulation time
    text(70,100,'Simulation Time :','fontsize',12);
    block.Dwork(10).Data(1) = text(150,100,'0','fontsize',12);
else
    block.Dwork(7).Data = zeros(8,1);
    block.Dwork(8).Data = 0;
    block.Dwork(9).Data = 0;
    block.Dwork(10).Data = 0;
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
%% plot animation
if block.DialogPrm(3).Data == 1
    center = [30 66; 85 46; 85 -46; 30 -66; -30 -66; -85 -46; -85 46; -30 66;];
    R = 10;
    thrust_temp = 2*block.Dwork(1).Data * R/800;     % 转入半径内
    azimuth_temp = block.Dwork(3).Data * pi/180;
    
    for i_temp = 1:8
        set(block.Dwork(7).Data(i_temp),...
            'xdata',[center(i_temp,1) center(i_temp,1) + thrust_temp(i_temp) * cos(azimuth_temp(i_temp))],...
            'ydata',[center(i_temp,2) center(i_temp,2) + thrust_temp(i_temp) * sin(azimuth_temp(i_temp))]);
    end
    
    set(block.Dwork(8).Data, 'xdata',[0 block.Dwork(5).Data(1)/50],...
        'ydata',[0 block.Dwork(5).Data(2)/50]);
    
    theta = -pi/2 : -0.01 : -pi/2 + block.Dwork(5).Data(3)/8e3*pi;
    x = 7.5 * cos(theta);
    y = 7.5 * sin(theta);
    set(block.Dwork(9).Data, 'xdata',x,'ydata',y);
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
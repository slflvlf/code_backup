function msfunMotion_Display(block)
% display the animation of the vessel's motion
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
function setup(block)

% Register number of ports
block.NumInputPorts  = 1;
block.NumOutputPorts = 0;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties

%% eta
block.InputPort(1).Dimensions        = 15;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = false;
block.InputPort(1).SamplingMode = 'Sample';

%%
% Override output port properties

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
% block.RegBlockMethod('Outputs', @Outputs);     % Required
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
block.NumDworks = 8;
  
  % 母船图像
  block.Dwork(1).Name            = 'h0';
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  % 拖船1图像
  block.Dwork(2).Name            = 'h1';
  block.Dwork(2).Dimensions      = 1;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  % 拖船2图像
  block.Dwork(3).Name            = 'h2';
  block.Dwork(3).Dimensions      = 1;
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;
  
  % p0
  block.Dwork(4).Name            = 'p0';
  block.Dwork(4).Dimensions      = 3;
  block.Dwork(4).DatatypeID      = 0;      % double
  block.Dwork(4).Complexity      = 'Real'; % real
  block.Dwork(4).UsedAsDiscState = true;
  
  % p1
  block.Dwork(5).Name            = 'p1';
  block.Dwork(5).Dimensions      = 3;
  block.Dwork(5).DatatypeID      = 0;      % double
  block.Dwork(5).Complexity      = 'Real'; % real
  block.Dwork(5).UsedAsDiscState = true;
  
  % p2
  block.Dwork(6).Name            = 'p2';
  block.Dwork(6).Dimensions      = 3;
  block.Dwork(6).DatatypeID      = 0;      % double
  block.Dwork(6).Complexity      = 'Real'; % real
  block.Dwork(6).UsedAsDiscState = true;
  
  % 缆绳1图像
  block.Dwork(7).Name            = 'c1';
  block.Dwork(7).Dimensions      = 1;
  block.Dwork(7).DatatypeID      = 0;      % double
  block.Dwork(7).Complexity      = 'Real'; % real
  block.Dwork(7).UsedAsDiscState = true;
  
  % 缆绳2图像
  block.Dwork(8).Name            = 'c2';
  block.Dwork(8).Dimensions      = 1;
  block.Dwork(8).DatatypeID      = 0;      % double
  block.Dwork(8).Complexity      = 'Real'; % real
  block.Dwork(8).UsedAsDiscState = true;
  


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
  block.Dwork(4).Data = block.DialogPrm(1).Data
  block.Dwork(5).Data = block.DialogPrm(2).Data;
  block.Dwork(6).Data = block.DialogPrm(3).Data;
  % 将初始位置的角度转为弧度单位
  block.Dwork(4).Data(3) = block.Dwork(4).Data(3) / 180 * pi;
  block.Dwork(5).Data(3) = block.Dwork(5).Data(3) / 180 * pi;
  block.Dwork(6).Data(3) = block.Dwork(6).Data(3) / 180 * pi;
% if block.DialogPrm(1).Data == 1
    %%
    close(findobj('type','figure','name','Motion of the vessels'))
    figure('Name','Motion of the vessel','units','normalized','position',...
           [0.6,0.2,0.4,0.4],'color',[0.8 1 0.8]);
    axis equal;
    grid on;hold on
%     line([-25 25],[0 0],'linewidth',2);
%     line([0 0],[-25 25],'linewidth',2);
%     line([16 24],[23 23],'linewidth',3,'color','b');
%     line([16 17],[23 23.6],'linewidth',3,'color','b');
%     line([16 17],[23.1 22.5],'linewidth',3,'color','b');
%     text(14,25,'Wave','fontname','Time New Roman','fontangle','oblique',...
%                                  'fontsize',14,'color','b');
%         xlim([-25 25]);ylim([-25 25]);

% xmax = max([block.Dwork(4).Data(1), block.Dwork(5).Data(1), block.Dwork(6).Data(1)]);
% xmin = min([block.Dwork(4).Data(1), block.Dwork(5).Data(1), block.Dwork(6).Data(1)]);
% ymax = max([block.Dwork(4).Data(2), block.Dwork(5).Data(2), block.Dwork(6).Data(2)]);
% ymin = min([block.Dwork(4).Data(2), block.Dwork(5).Data(2), block.Dwork(6).Data(2)]);
% xlim([xmin-(xmax-xmin)/1, xmax+(xmax-xmin)/1]);
% ylim([ymin-(ymax-ymin)/1, ymax+(ymax-ymin)/1]);    
% axis equal;
    
    %% draw the vessel
    x0 = [-75, 75, 75, -75, -75];
    y0 = [50, 50, -50, -50, 50];
    x1 = [-16, 12, 16, 12, -16, -16];
    y1 = [6, 6, 0, -6, -6, 6];
    x2 = [-16, 12, 16, 12, -16, -16];
    y2 = [6, 6, 0, -6, -6, 6];
    % 母船0
    block.Dwork(1).Data(1) = hgtransform;
    fill(x0,y0,'y','parent',block.Dwork(1).Data);
    % 拖轮1
    block.Dwork(2).Data(1) = hgtransform;
    fill(x1,y1,'r','parent',block.Dwork(2).Data);
    % 拖轮2
    block.Dwork(3).Data(1) = hgtransform;
    fill(x2,y2,'r','parent',block.Dwork(3).Data);
%     fill(x2,y2,'r','parent',block.Dwork(1).Data);
%     fill(x3,y3,'r','parent',block.Dwork(1).Data);
%     fill(x4,y4,'r','parent',block.Dwork(1).Data);
    %% center of the gravity
    line(0,0,'marker','o','markeredgecolor','k','markerfacecolor','k',...
        'parent',block.Dwork(1).Data);
    line(0,0,'marker','o','markeredgecolor','k','markerfacecolor','k',...
        'parent',block.Dwork(2).Data);
    line(0,0,'marker','o','markeredgecolor','k','markerfacecolor','k',...
        'parent',block.Dwork(3).Data);
    %% translate and rotate
    M0 = makehgtform('translate',[block.Dwork(4).Data(1) block.Dwork(4).Data(2) 0])*...
        makehgtform('zrotate',block.Dwork(4).Data(3));
    set(block.Dwork(1).Data, 'Matrix',M0);
    
    M1 = makehgtform('translate',[block.Dwork(5).Data(1) block.Dwork(5).Data(2) 0])*...
        makehgtform('zrotate',block.Dwork(5).Data(3));
    set(block.Dwork(2).Data, 'Matrix',M1);
    
    M2 = makehgtform('translate',[block.Dwork(6).Data(1) block.Dwork(6).Data(2) 0])*...
        makehgtform('zrotate',block.Dwork(6).Data(3));
    set(block.Dwork(3).Data, 'Matrix',M2);
    
    %% 锚链
    pc1_start = local_to_global([x0(2); y0(2)], block.Dwork(4).data);
    pc1_end = local_to_global([(x1(1)+x1(5))/2; (y1(1)+y1(5))/2], block.Dwork(5).data);
    pc2_start = local_to_global([x0(3); y0(3)], block.Dwork(4).data);
    pc2_end = local_to_global([(x2(1)+x2(5))/2; (y2(1)+y2(5))/2], block.Dwork(6).data);
    block.Dwork(7).Data(1) = line([pc1_start(1), pc1_end(1)], [pc1_start(2), pc1_end(2)]);
    block.Dwork(8).Data(1) = line([pc2_start(1), pc2_end(1)], [pc2_start(2), pc2_end(2)]);


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
% function Outputs(block)
% 
% 
% end %Outputs

%%
%% Update:
%%   Functionality    : Called to update discrete states
%%                      during simulation step
%%   Required         : No
%%   C-MEX counterpart: mdlUpdate
%%
function Update(block)

%% 赋值各船的位置
block.Dwork(4).Data = [block.InputPort(1).Data(1), block.InputPort(1).Data(2), ...
                        block.InputPort(1).Data(3)]'; %p0
block.Dwork(5).Data = [block.InputPort(1).Data(4), block.InputPort(1).Data(5), ...
                        block.InputPort(1).Data(6)]'; %p1
block.Dwork(6).Data = [block.InputPort(1).Data(10), block.InputPort(1).Data(11), ...
                        block.InputPort(1).Data(12)]'; %p2
% 将初始位置的角度转为弧度单位
block.Dwork(4).Data(3) = block.Dwork(4).Data(3) / 180 * pi;
block.Dwork(5).Data(3) = block.Dwork(5).Data(3) / 180 * pi;
block.Dwork(6).Data(3) = block.Dwork(6).Data(3) / 180 * pi;  

%% 锚链
    x0 = [-75, 75, 75, -75, -75];
    y0 = [50, 50, -50, -50, 50];
    x1 = [-16, 12, 16, 12, -16, -16];
    y1 = [6, 6, 0, -6, -6, 6];
    x2 = [-16, 12, 16, 12, -16, -16];
    y2 = [6, 6, 0, -6, -6, 6];
pc1_start = local_to_global([x0(2); y0(2)], block.Dwork(4).data);
pc1_end = local_to_global([(x1(1)+x1(5))/2; (y1(1)+y1(5))/2], block.Dwork(5).data);
pc2_start = local_to_global([x0(3); y0(3)], block.Dwork(4).data);
pc2_end = local_to_global([(x2(1)+x2(5))/2; (y2(1)+y2(5))/2], block.Dwork(6).data);

set(block.Dwork(7).Data, 'xdata', [pc1_start(1), pc1_end(1)], 'ydata', [pc1_start(2), pc1_end(2)]);
set(block.Dwork(8).Data, 'xdata', [pc2_start(1), pc2_end(1)], 'ydata', [pc2_start(2), pc2_end(2)]);

%% 坐标尺度
xmax = max([block.Dwork(4).Data(1), block.Dwork(5).Data(1), block.Dwork(6).Data(1)]);
xmin = min([block.Dwork(4).Data(1), block.Dwork(5).Data(1), block.Dwork(6).Data(1)]);
ymax = max([block.Dwork(4).Data(2), block.Dwork(5).Data(2), block.Dwork(6).Data(2)]);
ymin = min([block.Dwork(4).Data(2), block.Dwork(5).Data(2), block.Dwork(6).Data(2)]);
xlim([xmin-(xmax-xmin)/4, xmax+(xmax-xmin)/4]);
ylim([ymin-(ymax-ymin)/4, ymax+(ymax-ymin)/4]); 

    %%
    %% translate and rotate
    M0 = makehgtform('translate',[block.Dwork(4).Data(1) block.Dwork(4).Data(2) 0])*...
        makehgtform('zrotate',block.Dwork(4).Data(3));
    set(block.Dwork(1).Data, 'Matrix',M0);
    
    M1 = makehgtform('translate',[block.Dwork(5).Data(1) block.Dwork(5).Data(2) 0])*...
        makehgtform('zrotate',block.Dwork(5).Data(3));
    set(block.Dwork(2).Data, 'Matrix',M1);
    
    M2 = makehgtform('translate',[block.Dwork(6).Data(1) block.Dwork(6).Data(2) 0])*...
        makehgtform('zrotate',block.Dwork(6).Data(3));
    set(block.Dwork(3).Data, 'Matrix',M2);

    drawnow


end

%% 寻找锚链连接点(大地坐标系下）
function pc_g = local_to_global(pc_l, p0)
    fai = p0(3);
    R = [cos(fai), -sin(fai);
        sin(fai), cos(fai)];
    pc_g = R * pc_l + p0(1:2);
    
end

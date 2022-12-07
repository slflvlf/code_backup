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

%% p0, p1, v1, p2, v2, p3, v3, p4, v4
block.InputPort(1).Dimensions        = 27;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = false;
block.InputPort(1).SamplingMode = 'Sample';

%%
% Override output port properties

%%
% Register parameters
block.NumDialogPrms     = 5;

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
block.NumDworks = 14;
  
  % 母船0图像
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
  
  % 拖船3图像
  block.Dwork(4).Name            = 'h3';
  block.Dwork(4).Dimensions      = 1;
  block.Dwork(4).DatatypeID      = 0;      % double
  block.Dwork(4).Complexity      = 'Real'; % real
  block.Dwork(4).UsedAsDiscState = true;
  
  % 拖船4图像
  block.Dwork(5).Name            = 'h4';
  block.Dwork(5).Dimensions      = 1;
  block.Dwork(5).DatatypeID      = 0;      % double
  block.Dwork(5).Complexity      = 'Real'; % real
  block.Dwork(5).UsedAsDiscState = true;
  
  % p0
  block.Dwork(6).Name            = 'p0';
  block.Dwork(6).Dimensions      = 3;
  block.Dwork(6).DatatypeID      = 0;      % double
  block.Dwork(6).Complexity      = 'Real'; % real
  block.Dwork(6).UsedAsDiscState = true;
  
  % p1
  block.Dwork(7).Name            = 'p1';
  block.Dwork(7).Dimensions      = 3;
  block.Dwork(7).DatatypeID      = 0;      % double
  block.Dwork(7).Complexity      = 'Real'; % real
  block.Dwork(7).UsedAsDiscState = true;
  
  % p2
  block.Dwork(8).Name            = 'p2';
  block.Dwork(8).Dimensions      = 3;
  block.Dwork(8).DatatypeID      = 0;      % double
  block.Dwork(8).Complexity      = 'Real'; % real
  block.Dwork(8).UsedAsDiscState = true;
  
  % p3
  block.Dwork(9).Name            = 'p3';
  block.Dwork(9).Dimensions      = 3;
  block.Dwork(9).DatatypeID      = 0;      % double
  block.Dwork(9).Complexity      = 'Real'; % real
  block.Dwork(9).UsedAsDiscState = true;
  
  % p4
  block.Dwork(10).Name            = 'p4';
  block.Dwork(10).Dimensions      = 3;
  block.Dwork(10).DatatypeID      = 0;      % double
  block.Dwork(10).Complexity      = 'Real'; % real
  block.Dwork(10).UsedAsDiscState = true;
  
  % 缆绳1图像
  block.Dwork(11).Name            = 'c1';
  block.Dwork(11).Dimensions      = 1;
  block.Dwork(11).DatatypeID      = 0;      % double
  block.Dwork(11).Complexity      = 'Real'; % real
  block.Dwork(11).UsedAsDiscState = true;
  
  % 缆绳2图像
  block.Dwork(12).Name            = 'c2';
  block.Dwork(12).Dimensions      = 1;
  block.Dwork(12).DatatypeID      = 0;      % double
  block.Dwork(12).Complexity      = 'Real'; % real
  block.Dwork(12).UsedAsDiscState = true;
  
  % 缆绳3图像
  block.Dwork(13).Name            = 'c3';
  block.Dwork(13).Dimensions      = 1;
  block.Dwork(13).DatatypeID      = 0;      % double
  block.Dwork(13).Complexity      = 'Real'; % real
  block.Dwork(13).UsedAsDiscState = true;
  
  % 缆绳4图像
  block.Dwork(14).Name            = 'c4';
  block.Dwork(14).Dimensions      = 1;
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

  %% Initialize Dwork
  block.Dwork(6).Data = block.DialogPrm(1).Data
  block.Dwork(7).Data = block.DialogPrm(2).Data;
  block.Dwork(8).Data = block.DialogPrm(3).Data;
  block.Dwork(9).Data = block.DialogPrm(4).Data;
  block.Dwork(10).Data = block.DialogPrm(5).Data;
  % 将初始位置的角度转为弧度单位
  block.Dwork(6).Data(3) = block.Dwork(6).Data(3) / 180 * pi;
  block.Dwork(7).Data(3) = block.Dwork(7).Data(3) / 180 * pi;
  block.Dwork(8).Data(3) = block.Dwork(8).Data(3) / 180 * pi;
  block.Dwork(9).Data(3) = block.Dwork(9).Data(3) / 180 * pi;
  block.Dwork(10).Data(3) = block.Dwork(10).Data(3) / 180 * pi;
% if block.DialogPrm(1).Data == 1
    %%
    close(findobj('type','figure','name','Motion of the vessels'))
    figure('Name','Motion of the vessel','units','normalized','position',...
           [0.6,0.2,0.4,0.4],'color',[0.8 1 0.8]);
    axis equal;
    grid on;hold on

    
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
    % 拖轮3
    block.Dwork(4).Data(1) = hgtransform;
    fill(x1,y1,'r','parent',block.Dwork(4).Data);
    % 拖轮4
    block.Dwork(5).Data(1) = hgtransform;
    fill(x2,y2,'r','parent',block.Dwork(5).Data);
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
    line(0,0,'marker','o','markeredgecolor','k','markerfacecolor','k',...
        'parent',block.Dwork(4).Data);
    line(0,0,'marker','o','markeredgecolor','k','markerfacecolor','k',...
        'parent',block.Dwork(5).Data);
    %% translate and rotate
    M0 = makehgtform('translate',[block.Dwork(6).Data(1) block.Dwork(6).Data(2) 0])*...
        makehgtform('zrotate',block.Dwork(6).Data(3));
    set(block.Dwork(1).Data, 'Matrix',M0);
    
    M1 = makehgtform('translate',[block.Dwork(7).Data(1) block.Dwork(7).Data(2) 0])*...
        makehgtform('zrotate',block.Dwork(7).Data(3));
    set(block.Dwork(2).Data, 'Matrix',M1);
    
    M2 = makehgtform('translate',[block.Dwork(8).Data(1) block.Dwork(8).Data(2) 0])*...
        makehgtform('zrotate',block.Dwork(8).Data(3));
    set(block.Dwork(3).Data, 'Matrix',M2);
    
    M3 = makehgtform('translate',[block.Dwork(9).Data(1) block.Dwork(9).Data(2) 0])*...
        makehgtform('zrotate',block.Dwork(9).Data(3));
    set(block.Dwork(4).Data, 'Matrix',M3);
    
    M4 = makehgtform('translate',[block.Dwork(10).Data(1) block.Dwork(10).Data(2) 0])*...
        makehgtform('zrotate',block.Dwork(10).Data(3));
    set(block.Dwork(5).Data, 'Matrix',M4);
    
    %% 锚链
    pc1_start = local_to_global([x0(2); y0(2)], block.Dwork(6).data);
    pc1_end = local_to_global([(x1(1)+x1(5))/2; (y1(1)+y1(5))/2], block.Dwork(7).data);
    pc2_start = local_to_global([x0(3); y0(3)], block.Dwork(6).data);
    pc2_end = local_to_global([(x2(1)+x2(5))/2; (y2(1)+y2(5))/2], block.Dwork(8).data);
    pc3_start = local_to_global([x0(4); y0(4)], block.Dwork(6).data);
    pc3_end = local_to_global([(x1(1)+x1(5))/2; (y1(1)+y1(5))/2], block.Dwork(9).data);
    pc4_start = local_to_global([x0(5); y0(5)], block.Dwork(6).data);
    pc4_end = local_to_global([(x2(1)+x2(5))/2; (y2(1)+y2(5))/2], block.Dwork(10).data);
    
    block.Dwork(11).Data(1) = line([pc1_start(1), pc1_end(1)], [pc1_start(2), pc1_end(2)]);
    block.Dwork(12).Data(1) = line([pc2_start(1), pc2_end(1)], [pc2_start(2), pc2_end(2)]);
    block.Dwork(13).Data(1) = line([pc3_start(1), pc3_end(1)], [pc3_start(2), pc3_end(2)]);
    block.Dwork(14).Data(1) = line([pc4_start(1), pc4_end(1)], [pc4_start(2), pc4_end(2)]);

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
block.Dwork(6).Data = [block.InputPort(1).Data(1), block.InputPort(1).Data(2), ...
                        block.InputPort(1).Data(3)]'; %p0
block.Dwork(7).Data = [block.InputPort(1).Data(4), block.InputPort(1).Data(5), ...
                        block.InputPort(1).Data(6)]'; %p1
block.Dwork(8).Data = [block.InputPort(1).Data(10), block.InputPort(1).Data(11), ...
                        block.InputPort(1).Data(12)]'; %p2
block.Dwork(9).Data = [block.InputPort(1).Data(16), block.InputPort(1).Data(17), ...
                        block.InputPort(1).Data(18)]'; %p3
block.Dwork(10).Data = [block.InputPort(1).Data(22), block.InputPort(1).Data(23), ...
                        block.InputPort(1).Data(24)]'; %p4
% 将初始位置的角度转为弧度单位
block.Dwork(6).Data(3) = block.Dwork(6).Data(3) / 180 * pi;
block.Dwork(7).Data(3) = block.Dwork(7).Data(3) / 180 * pi;
block.Dwork(8).Data(3) = block.Dwork(8).Data(3) / 180 * pi;  
block.Dwork(9).Data(3) = block.Dwork(9).Data(3) / 180 * pi;
block.Dwork(10).Data(3) = block.Dwork(10).Data(3) / 180 * pi;  

%% 锚链
    x0 = [-75, 75, 75, -75, -75];
    y0 = [50, 50, -50, -50, 50];
    x1 = [-16, 12, 16, 12, -16, -16];
    y1 = [6, 6, 0, -6, -6, 6];
    x2 = [-16, 12, 16, 12, -16, -16];
    y2 = [6, 6, 0, -6, -6, 6];
    pc1_start = local_to_global([x0(2); y0(2)], block.Dwork(6).data);
    pc1_end = local_to_global([(x1(1)+x1(5))/2; (y1(1)+y1(5))/2], block.Dwork(7).data);
    pc2_start = local_to_global([x0(3); y0(3)], block.Dwork(6).data);
    pc2_end = local_to_global([(x2(1)+x2(5))/2; (y2(1)+y2(5))/2], block.Dwork(8).data);
    pc3_start = local_to_global([x0(4); y0(4)], block.Dwork(6).data);
    pc3_end = local_to_global([(x1(1)+x1(5))/2; (y1(1)+y1(5))/2], block.Dwork(9).data);
    pc4_start = local_to_global([x0(5); y0(5)], block.Dwork(6).data);
    pc4_end = local_to_global([(x2(1)+x2(5))/2; (y2(1)+y2(5))/2], block.Dwork(10).data);
% pc1_start = local_to_global([x0(2); y0(2)], block.Dwork(4).data);
% pc1_end = local_to_global([(x1(1)+x1(5))/2; (y1(1)+y1(5))/2], block.Dwork(5).data);
% pc2_start = local_to_global([x0(3); y0(3)], block.Dwork(4).data);
% pc2_end = local_to_global([(x2(1)+x2(5))/2; (y2(1)+y2(5))/2], block.Dwork(6).data);

set(block.Dwork(11).Data, 'xdata', [pc1_start(1), pc1_end(1)], 'ydata', [pc1_start(2), pc1_end(2)]);
set(block.Dwork(12).Data, 'xdata', [pc2_start(1), pc2_end(1)], 'ydata', [pc2_start(2), pc2_end(2)]);
set(block.Dwork(13).Data, 'xdata', [pc3_start(1), pc3_end(1)], 'ydata', [pc3_start(2), pc3_end(2)]);
set(block.Dwork(14).Data, 'xdata', [pc4_start(1), pc4_end(1)], 'ydata', [pc4_start(2), pc4_end(2)]);

%% 坐标尺度
xmax = max([block.Dwork(6).Data(1), block.Dwork(7).Data(1), block.Dwork(8).Data(1), block.Dwork(9).Data(1), block.Dwork(10).Data(1)]);
xmin = min([block.Dwork(6).Data(1), block.Dwork(7).Data(1), block.Dwork(8).Data(1), block.Dwork(9).Data(1), block.Dwork(10).Data(1)]);
ymax = max([block.Dwork(6).Data(2), block.Dwork(7).Data(2), block.Dwork(8).Data(2), block.Dwork(9).Data(2), block.Dwork(10).Data(2)]);
ymin = min([block.Dwork(6).Data(2), block.Dwork(7).Data(2), block.Dwork(8).Data(2), block.Dwork(9).Data(2), block.Dwork(10).Data(2)]);

xlim([xmin-(xmax-xmin)/4, xmax+(xmax-xmin)/4]);
ylim([ymin-(ymax-ymin)/4, ymax+(ymax-ymin)/4]); 

    %%
    %% translate and rotate
    M0 = makehgtform('translate',[block.Dwork(6).Data(1) block.Dwork(6).Data(2) 0])*...
        makehgtform('zrotate',block.Dwork(6).Data(3));
    set(block.Dwork(1).Data, 'Matrix',M0);
    
    M1 = makehgtform('translate',[block.Dwork(7).Data(1) block.Dwork(7).Data(2) 0])*...
        makehgtform('zrotate',block.Dwork(7).Data(3));
    set(block.Dwork(2).Data, 'Matrix',M1);
    
    M2 = makehgtform('translate',[block.Dwork(8).Data(1) block.Dwork(8).Data(2) 0])*...
        makehgtform('zrotate',block.Dwork(8).Data(3));
    set(block.Dwork(3).Data, 'Matrix',M2);
    
    M3 = makehgtform('translate',[block.Dwork(9).Data(1) block.Dwork(9).Data(2) 0])*...
        makehgtform('zrotate',block.Dwork(9).Data(3));
    set(block.Dwork(4).Data, 'Matrix',M3);
    
    M4 = makehgtform('translate',[block.Dwork(10).Data(1) block.Dwork(10).Data(2) 0])*...
        makehgtform('zrotate',block.Dwork(10).Data(3));
    set(block.Dwork(5).Data, 'Matrix',M4);

    drawnow


end

%% 寻找锚链连接点(大地坐标系下）
function pc_g = local_to_global(pc_l, p0)
    fai = p0(3);
    R = [cos(fai), -sin(fai);
        sin(fai), cos(fai)];
    pc_g = R * pc_l + p0(1:2);
    
end

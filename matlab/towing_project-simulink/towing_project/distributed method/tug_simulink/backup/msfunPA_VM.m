function msfunPA_VM(block)
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
block.NumInputPorts  = 3;
block.NumOutputPorts = 0;

% Setup port properties to be inherited or dynamic
block.SetPreCompInpPortInfoToDynamic;
block.SetPreCompOutPortInfoToDynamic;

% Override input port properties

%% eta
block.InputPort(1).Dimensions        = 3;
block.InputPort(1).DatatypeID  = 0;  % double
block.InputPort(1).Complexity  = 'Real';
block.InputPort(1).DirectFeedthrough = false;
block.InputPort(1).SamplingMode = 'Sample';
%% initial position
block.InputPort(2).Dimensions        = 3;
block.InputPort(2).DatatypeID  = 0;  % double
block.InputPort(2).Complexity  = 'Real';
block.InputPort(2).DirectFeedthrough = false;
block.InputPort(2).SamplingMode = 'Sample';
%% tau in earth-fixed coordinate
block.InputPort(3).Dimensions        = 3;
block.InputPort(3).DatatypeID  = 0;  % double
block.InputPort(3).Complexity  = 'Real';
block.InputPort(3).DirectFeedthrough = false;
block.InputPort(3).SamplingMode = 'Sample';
%%
% Override output port properties

%%
% Register parameters
block.NumDialogPrms     = 1;

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
%block.RegBlockMethod('SetInputPortDimensions', @SetInpPortDims);

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
block.NumDworks = 3;
  
  block.Dwork(1).Name            = 'h';
  block.Dwork(1).Dimensions      = 1;
  block.Dwork(1).DatatypeID      = 0;      % double
  block.Dwork(1).Complexity      = 'Real'; % real
  block.Dwork(1).UsedAsDiscState = true;
  
  block.Dwork(2).Name            = 'thrust';
  block.Dwork(2).Dimensions      = 1;
  block.Dwork(2).DatatypeID      = 0;      % double
  block.Dwork(2).Complexity      = 'Real'; % real
  block.Dwork(2).UsedAsDiscState = true;
  
  block.Dwork(3).Name            = 'moment';
  block.Dwork(3).Dimensions      = 1;
  block.Dwork(3).DatatypeID      = 0;      % double
  block.Dwork(3).Complexity      = 'Real'; % real
  block.Dwork(3).UsedAsDiscState = true;

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
if block.DialogPrm(1).Data == 1
    %%
    close(findobj('type','figure','name','Motion of the vessel'))
    figure('Name','Motion of the vessel','units','normalized','position',...
           [0.6,0.2,0.4,0.4],'color',[0.8 1 0.8])
    axis equal;grid on;hold on
    line([-25 25],[0 0],'linewidth',2);
    line([0 0],[-25 25],'linewidth',2);
    line([16 24],[23 23],'linewidth',3,'color','b');
    line([16 17],[23 23.6],'linewidth',3,'color','b');
    line([16 17],[23.1 22.5],'linewidth',3,'color','b');
    text(14,25,'Wave','fontname','Time New Roman','fontangle','oblique',...
                                 'fontsize',14,'color','b');
    xlim([-25 25]);ylim([-25 25]);
    
    %% draw the vessel
    x1 = [-8 -8 8 10 8 -8];
    y1 = [-7.5 -2.5 -2.5 -5 -7.5 -7.5];
    x2 = [-8 -8 8 10 8 -8];
    y2 = [2.5 7.5 7.5 5 2.5 2.5];
    x3 = [-4 -4 -2 -2 -4];
    y3 = [-2.5 2.5 2.5 -2.5 -2.5];
    x4 = [2 2 4 4 2];
    y4 = [-2.5 2.5 2.5 -2.5 -2.5];    
    block.Dwork(1).Data(1) = hgtransform;
    fill(x1,y1,'r','parent',block.Dwork(1).Data);
    fill(x2,y2,'r','parent',block.Dwork(1).Data);
    fill(x3,y3,'r','parent',block.Dwork(1).Data);
    fill(x4,y4,'r','parent',block.Dwork(1).Data);
    %% center of the gravity
    line(0,0,'marker','o','markeredgecolor','k','markerfacecolor','k',...
        'parent',block.Dwork(1).Data);
    %% symbol of total thrust
    block.Dwork(2).Data(1) = line([0  block.InputPort(3).Data(1)/1e5],...
         [0  block.InputPort(3).Data(2)/1e5],[0.01 0.01],...
         'linewidth',5,'parent',block.Dwork(1).Data,'color','k');
    %% symbol of total moment 
    line([0 0],[-5 -3],'color','k','linewidth',5,'parent',block.Dwork(1).Data);
    theta = -pi/2 : 0.01 : -pi/2 + block.InputPort(3).Data(3)/1e7*pi;
    x =  4 * cos(theta);
    y =  4 * sin(theta);
    block.Dwork(3).Data(1) = line(x,y,'color','k','linewidth',5,'parent',block.Dwork(1).Data);
    %% translate and rotate
    M = makehgtform('translate',[block.InputPort(2).Data(1) block.InputPort(2).Data(2) 0])*...
        makehgtform('zrotate',block.InputPort(2).Data(3));
    set(block.Dwork(1).Data, 'Matrix',M);
else
    return
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

%%
if block.DialogPrm(1).Data == 1
    set(block.Dwork(2).Data(1), 'xdata',[0 block.InputPort(3).Data(1)/1e5],...
        'ydata',[0 block.InputPort(3).Data(2)/1e5]);
    
    theta = -pi/2 : -0.01 : -pi/2 + block.InputPort(3).Data(3)/1e7*pi;
    x = 4 * cos(theta);
    y = 4 * sin(theta);
    set(block.Dwork(3).Data, 'xdata',x,'ydata',y);
    %%
    M = makehgtform('translate',[block.InputPort(1).Data(1),...
                                 block.InputPort(1).Data(2) 0])*...
        makehgtform('zrotate',block.InputPort(1).Data(3));    
    set(block.Dwork(1).Data, 'Matrix',M);
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
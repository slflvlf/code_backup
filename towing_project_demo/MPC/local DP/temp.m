clear all;  clc;

getShipDynamicsAndJacobian;
%MPC 控制器

nx = 6;
ny = 6;
nu = 6;

nlobj = nlmpc(nx, ny, nu);

nlobj.Model.StateFcn = "ShipStateFcn";
nlobj.Jacobian.StateFcn = @ShipStateJacobianFcn;

rng(0)

validateFcns(nlobj,rand(nx,1),rand(nu,1));


Ts = 0.5;
p = 15;
m = 3;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = m;


nlobj.MV = struct('Min',{0; 0; 0; 0; 0; 0},'Max',{8e5; 8e5; 8e5; 2*pi; 2*pi; 2*pi},...
    'RateMin', {-5e4; -5e4; -5e4; -pi/6; -pi/6; -pi/6}, 'RateMax', {5e4; 5e4; 5e4; pi/6; pi/6; pi/6});

nlobj.Weights.OutputVariables = 10*[10 10 10 0 0 0];

nlobj.Weights.ManipulatedVariables = [1 1 1 0 0 0];

nlobj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1 0.1 0.1 0.1];

% Specify the initial conditions
x = [0;0;0;0;0;0];
% Nominal control that keeps the quadrotor floating
nloptions = nlmpcmoveopt;
nloptions.MVTarget = [1e5 1e5 1e5 pi/3 pi/3 pi/3]; 
mv = nloptions.MVTarget;
% nloptions.MV0 = [2e5 2e5 2e5 pi/3 pi/3 pi/3]; 
% mv = nloptions.MV0;

Duration = 200;
% hbar = waitbar(0,'Simulation Progress');
xHistory = x';
lastMV = mv;
uHistory = lastMV;

for k = 1:(Duration/Ts)
    % Set references for previewing
    t = linspace(k*Ts, (k+p-1)*Ts, p);
    yref = ShipReferenceTrajectory(t);
    % Compute the control moves with reference previewing.
    xk = xHistory(k,:);
    
    [uk,nloptions,info] = nlmpcmove(nlobj,xk,lastMV,yref',[],nloptions);
    uHistory(k+1,:) = uk';
    lastMV = uk;
    % Update states.
    ODEFUN = @(t,xk) ShipStateFcn(xk,uk);
    [TOUT,YOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');
    xHistory(k+1,:) = YOUT(end,:);
%     waitbar(k*Ts/Duration,hbar);
end

figure(1)
subplot(3, 1, 1)
plot(xHistory(:, 1));
subplot(3, 1, 2)
plot(xHistory(:, 2));
subplot(3, 1, 3)
plot(xHistory(:, 3)/pi*180);


figure(2)
subplot(3, 2, 1)
plot(uHistory(:, 1)/1e3);
subplot(3, 2, 3)
plot(uHistory(:, 2)/1e3);
subplot(3, 2, 5)
plot(uHistory(:, 3)/1e3);
subplot(3, 2, 2)
plot(uHistory(:, 4)/pi*180);
subplot(3, 2, 4)
plot(uHistory(:, 5)/pi*180);
subplot(3, 2, 6)
plot(uHistory(:, 6)/pi*180);



% close(hbar)





function [ xdesired ] = ShipReferenceTrajectory( t )
    x = ones(1, length(t)) * 1;
    y = ones(1, length(t)) * 1;
    phi = ones(1, length(t)) * pi/3;
%     x = 6*sin(t/3);
%     y = -6*sin(t/3).*cos(t/3);
%     phi = zeros(1,length(t));
    
    xdot = zeros(1,length(t));
    ydot = zeros(1,length(t));
    phidot = zeros(1,length(t));

    xdesired = [x; y; phi; xdot; ydot; phidot];
end   
    







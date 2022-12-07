function [p_hat, v_hat, state, output] = nonlinear_passive_observer(p, tau, state, output, observer_paramters)

    step_size = observer_paramters.step_size;

    Cw = zeros(3,6);
    Cw(1,2) = 1;Cw(2,4) = 1;Cw(3,6) = 1;
    
    t = 0;
    input = p;
    
    [t, state] = ode45(@(t,state) odeSystemFunction(t, state, input, output, tau, observer_paramters),...
        0:step_size/2:step_size, state);
    state;
    state = state(end, :)';
    
    p_hat = state(7:9);
    v_hat = state(13:15);
    output = state(7:9)+Cw*state(1:6);
end


function dy = odeSystemFunction(t, state, input, output, tau, observer_paramters)
    % 缓变载荷时间常数
    T = observer_paramters.T;
    T_inv = 1/T*eye(3);
    
    lambda = observer_paramters.lambda;
    % 波浪谱的谱峰频率
    w_o = observer_paramters.w;
    %截至频率
    w_c =observer_paramters.wc;
    
    


    
    
    Zeta = 1;
    Aw = blkdiag([0 1;-w_o(1)^2 -2*lambda(1)*w_o(1)],...
             [0 1;-w_o(2)^2 -2*lambda(2)*w_o(2)],...
             [0 1;-w_o(3)^2 -2*lambda(3)*w_o(3)]);
    K1 = zeros(6,3);
    for i =1:3
        K1(2*i-1,i) = -2*(Zeta - lambda(i))*w_c/w_o(i);
        K1(2*i,i) = 2*w_o(i)*(Zeta - lambda(i));
    end
    K1;
    K2 = diag([w_c,w_c,w_c]);
    Cw = zeros(3,6);
    Cw(1,2) = 1;Cw(2,4) = 1;Cw(3,6) = 1;

    K3 = observer_paramters.K3;
    K4 = observer_paramters.K4;
    M = observer_paramters.M;
    D = observer_paramters.D;
    G = observer_paramters.G;
    
    dy = zeros(15, 1);
    xi = state(1:6);
    eta = state(7:9);
    b = state(10:12);
    nu = state(13:15);

    output = eta + Cw * xi;
    error = input - output;

    y3 = state(9);
    R = rotate_matrix(y3);

    dy(1:6) = Aw*xi+K1*error;
    dy(7:9) = R*nu+K2*error;
    dy(10:12) = T_inv*b+K3*error;
    dy(13:15) = inv(M)*(-D*nu-G*eta+R'*b+tau+R'*K4*error);
    
end

function R = rotate_matrix(fai)
    R = [cos(fai), -sin(fai), 0;
        sin(fai), cos(fai), 0;
        0, 0, 1];
end
function [p_hat, v_hat, state, output] = nonlinear_passive_observer2(p, tau, M, D, G, state, output)



%步长
    step_size = 0.1;
    %波浪模型的阻尼系数
    %波频运动的输出矩阵
    Cw = zeros(3, 6);
    Cw(:, 4:6) = eye(3);
    
    t = 0;
    input = p;
    
    [t, state] = ode45(@(t,state) odeSystemFunction(t, state, input, output, tau, M, D, G), 0:step_size/2:step_size, state);
    state;
    state = state(end, :)';
    
    p_hat = state(7:9);
    v_hat = state(13:15);
    output = state(7:9)+Cw*state(1:6);
end


function dy = odeSystemFunction(t, state, input, output, tau, M, D, G)
    % 缓变载荷时间常数
    T = 1000;
    T_inv = 1/T*ones(3);
    
    lambda = 0.1;
    % 波浪谱的谱峰频率
    w = 0.6;
    %截至频率
    wc = 0.7;
    
    %波浪模型的状态矩阵
    Aw = zeros(6, 6);
    Aw(1:3, 4:6) = eye(3);
    Aw(4:6, 1:3) = -w^2*eye(3);
    Aw(4:6, 4:6) = -2 * lambda * w * eye(3);
    
    %波频运动的输出矩阵
    Cw = zeros(3, 6);
    Cw(:, 4:6) = eye(3);
    
    K1 = zeros(6, 3);
    for i = 1 : 3
        K1(i, i) = -2*wc*(1-lambda)/w;
        K1(i+3, i) = 2*w*(1-lambda);
    end
    K2 = wc*eye(3);
    K3 = 5e5*diag([2 2 3500]);
    K4 = 6e5*diag([2 2 3500]);
    K3 = diag([1e5, 1e5, 1e8]);
    K4 = diag([1e6, 1e6, 1e9]);

    dy = zeros(15, 1);
    xi = state(1:6);
    eta = state(7:9);
    b = state(10:12);
    nu = state(13:15);

    
    error = input - output;
    y3 = input(3);
    R = rotate_matrix(y3);
    dy(1:6) = Aw*xi+K1*error;
    dy(7:9) = R*nu+K2*error;
    dy(10:12) = -T_inv*b+K3*error;
    dy(13:15) = inv(M)*(-D*nu-G*eta+R'*b+tau+R'*K4*error);
    
end

function R = rotate_matrix(fai)
    R = [cos(fai), -sin(fai), 0;
        sin(fai), cos(fai), 0;
        0, 0, 1];
end


function [pd1, pd2, pd3, pd4, coop_error, K] = LQR_consensus_guidance(pd0, p1, p2, p3, p4,v1, v2, v3, v4, wn, etaD, Q, R)

    global delta10 delta20 delta30 delta40;

% ���ת��Ϊ�������ϵ��
    R0 = rotate_matrix(pd0);
    delta10_g = R0 * delta10;
    delta20_g = R0 * delta20;
    delta30_g = R0 * delta30;
    delta40_g = R0 * delta40;
    
    % ����dot_p ��ƫ��
    delta10_g = [delta10_g; 0; 0; 0];
    delta20_g = [delta20_g; 0; 0; 0];
    delta30_g = [delta30_g; 0; 0; 0];
    delta40_g = [delta40_g; 0; 0; 0];
    
    delta12_g = delta10_g - delta20_g;  delta21_g = -delta12_g;
    delta13_g = delta10_g - delta30_g;  delta31_g = -delta13_g;
    delta23_g = delta20_g - delta30_g;  delta32_g = -delta23_g;
    delta24_g = delta20_g - delta40_g;  delta42_g = -delta24_g;
    delta34_g = delta30_g - delta40_g;  delta43_g = -delta34_g;
    delta41_g = delta40_g - delta10_g;  delta14_g = -delta41_g;
    
    %���ٶ�vת��Ϊdot_p
    R1 = rotate_matrix(p1);
    R2 = rotate_matrix(p2);
    R3 = rotate_matrix(p3);
    R4 = rotate_matrix(p4);
    

    
    %����Эͬ������һ���ԣ�
    dot_pd0 = zeros(3, 1);
    x0 = [pd0; dot_pd0];
    x1 = [p1; R1 * v1];
    x2 = [p2; R2 * v2];
    x3 = [p3; R3 * v3];
    x4 = [p4; R4 * v4];
    
    error1 = (x2 - x1 + delta12_g) + (x0 - x1 + delta10_g) + (x4 - x1 - delta41_g);
    error2 = (x1 - x2 - delta12_g) + (x0 - x2 + delta20_g) + (x3 - x2 - delta32_g);
    error3 = (x2 - x3 - delta23_g) + (x0 - x3 + delta30_g) + (x4 - x3 - delta43_g);
    error4 = (x1 - x4 - delta14_g) + (x0 - x4 + delta40_g) + (x3 - x4 - delta34_g);
    coop_error = [error1, error2, error3, error4];
    
    K = LQRcomputeGain(wn, etaD, Q, R);
   
    

    u1 = K * error1;
    u2 = K * error2;
    u3 = K * error3;
    u4 = K * error4;

    pd1 = u1 + p1;
    pd2 = u2 + p2;
    pd3 = u3 + p3;
    pd4 = u4 + p4;
end

%% LQR��������
function K = LQRcomputeGain(wn, cd, Q, R)
%      wn = 0.05; 
%     cd = 0.7;

    A1 = -2 * cd * wn * eye(3);
    A0 = -wn^2 * eye(3);
    B1 = -A0;

    A = [zeros(3, 3), eye(3); A0, A1];
    B = [zeros(3, 3); B1];
% lqr ������ָ��
%     Q = diag([10, 10, 10, 1, 1, 1]);
% % R = diag([0.01, 0.01, 4e-8]);
% %     Q = 40 * eye(6);
% %     Q(4, 4) = 1; Q(5, 5) = 1; Q(6, 6) = 1;
%     R = 1 * eye(3);
    N = [];
    [K, S, e] = lqr(A, B, Q, R, N);

end

%% ��ת����
function R= rotate_matrix(p)
    fai = p(3);
    R = [cos(fai), -sin(fai), 0;
        sin(fai), cos(fai), 0;
        0, 0, 1];
end

%% ���Ƕ�ת��Ϊ��-pi, pi)
function y = transfer_deg(x)
    y = -sign(x) * pi + rem((x + sign(x) * pi), 2 * pi);
end







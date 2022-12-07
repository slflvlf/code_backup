function[f,df,a,da,tau_r,dtau] = Thrust_Allocation(f0,a0,tau)
% function[f,df,a,da,tau_r,dtau,tau_r6] = Thrust_Allocation(f0,a0,tau)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Thrust_Allcation is used to allocate commanded thrust among the azimuth
% thrusters (only)
%
% INPUT:
%  f0               ��һ��������ֵ���� (KN)
%  a0               ��һ�����ƽ���ת������ (degree)
%  tau              ������Ҫ����������� (KN;KN;KN*m)
%
% OUTPUT:
%  f                �����ĸ��ƽ�������ֵ���� (KN)
%  df               �������һ��������ֵ�仯 (KN)
%  a                �����ĸ��ƽ���ת������ (degree)
%  da               �������һ����ת�Ǳ仯 (degree)
%  tau_r            �����ʵ���������� (KN;KN;KN*m)
%  dtau             ʵ��������Ҫ�������Ĳ�ֵ
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%%%%%%%%%%%%%%%%%%%%%%%%%% ���� %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ���Ƕ�ת��Ϊ����
d2r = pi/180;
% ��һ���ƽ���ת�ǻ�Ϊ����
a0 = a0 * d2r;
% ȫ��ת�ƽ�������
N = 3;
% �ɳ����ӵ�Ȩ����
Q = 1e12*diag([30 30 1]);
% ȫ��ת�ٶȵ�Ȩ����
Omega = 10*eye(N);
% �ƽ���ȫ��ת�ٶ�����(rad/sample_time)
da = [-15 15]*d2r;                                % �谴ʵ������
% �����ƽ��������ı仯����(KN)
Frange = [0 600];                              % �谴ʵ������
% ������ʹ�����
% delta_u = [0; 1; 1; 1; 0; 1; 1; 1;];            % ����������ʧЧ���
% �ƽ��������ı仯�ٶ�����(KN/sample_time)
dF = [-100 100];                                  % �谴ʵ������
% �ƽ�������ֵ
L = [-5.02 0; 11.956 2.7; 11.956 -2.7];
% config = [1.4 190; 0.87 205; 0.57 275; 1.7 17; 0.87 125; 0.57 94];
% L =zeros(N, 2);
% for i = 1 : N
%     L(i, 1) = config(i, 1) * cos(config(i, 2)/180*pi);
%     L(i, 2) = config(i, 1) * sin(config(i, 2)/180*pi);
% end

% L = [76 0; 51 26; -51 26; -76 0; -51 -26; 51 -26];     % �谴ʵ������
% Semi 708
% L = [15.7 35.5 -21.5; 47.02 24.58 -21.5; 47.02 -24.58 -21.5; 15.7 -35.5 -21.5; ...
%      -15.7 -35.5 -21.5;-47.02 -24.58 -21.5; -47.02 24.58 -21.5; -15.7 35.5 -21.5;];
% Semi 807
% L = [15.7 24.58; 47.02 35.5; 47.02 -35.5; 15.7 -24.58; -15.7 -24.58; -47.02 -35.5; -47.02 35.5; -15.7 24.58;];     % �谴ʵ������ Semi807
% Semi 981
% L = [47.02 35.5; 47.02 24.58; 47.02 -24.58; 47.02 -35.5; -47.02 -35.5; -47.02 -24.58; -47.02 24.58; -47.02 35.5;];     % �谴ʵ������

% % Ŀ�꺯������λ�óͷ�����ӳ���
% pp = 1000;
% % Ŀ�꺯������λ�óͷ����ĸ�еĳ���
% ee = 1;

% ��ֹ�����ú�Ŀ������䣬1��4�ƽ���������������
% angle_sector1o = [-15 15;75 285]*d2r;
% angle_sector2o = [165 425]*d2r;
% angle_sector3o = [115 375]*d2r;
% angle_sector4o = [165 195;-105 105]*d2r;
% angle_sector5o = [-15 245]*d2r;
% angle_sector6o = [-65 195]*d2r;
angle_sector1o = [-180 180]*d2r;
angle_sector2o = [-180 180]*d2r;
angle_sector3o = [-180 180]*d2r;
% angle_sector4o = [-180 180]*d2r;
% angle_sector5o = [-180 180]*d2r;   
% angle_sector6o = [-180 180]*d2r;
% angle_sector7o = [-180 180]*d2r;   
% angle_sector8o = [-180 180]*d2r;
% �����������䣬���Ƕ������ƽ���������ѡ��δ�����ܼ��붯̬��ֹ��
angle_sector = 1000*ones(N,2);
angle_sector(1,:) = angle_sector1o;
angle_sector(2,:) = angle_sector2o;
angle_sector(3,:) = angle_sector3o;
% angle_sector(4,:) = angle_sector4o;
% angle_sector(5,:) = angle_sector5o;
% angle_sector(6,:) = angle_sector6o;
% angle_sector(7,:) = angle_sector7o;
% angle_sector(8,:) = angle_sector8o;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% �ƽ���ȫ��ת�ٶ�Լ������
dda = 1000*ones(N,2);     % ��ʼ��
% ���ȿ��ǿ����������ϲ��Ƕȵ�����,����ddaת��[-2pi 2pi],�Ա���da�Ƚ�
for i = 1 : N
    dda(i,:) = angle_sector(i,:) - a0(i);
    while(dda(i,2) + 100*eps < 0)
        dda(i,:) = dda(i,:) + 2*pi;
    end
    while(dda(i,1) - 100*eps > 0)
        dda(i,:) = dda(i,:) - 2*pi;
    end
end
% ��ȫ��ת�ƽ���ÿ�������ת��da�Ƚϣ�ѡ����С�ı仯����
for i = 1 : N
    if da(1) > dda(i,1)
        dda(i,1) = da(1);
    end
    if da(2) < dda(i,2)
        dda(i,2) = da(2);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% �ƽ��������仯����
F = [ Frange(1) Frange(2)]; % ����ʧЧ���������������仯����
ff = 1e10*ones(N,2);
for i = 1 : N
    ff(i,1) = f0(i) + dF(1);
    if ff(i,1) <F(1)
        ff(i,1) = F(1);
    end
    
    ff(i,2) = f0(i) + dF(2);
    if ff(i,2) > F(2)
        ff(i,2) = F(2);
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ����ϵ��
T =  thrusters_configuration(a0,L);
dTf = Get_Coefficients(f0,a0);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Ϊ���ι滮����׼��ϵ��
H = blkdiag(2*eye(N),2*Omega,2*Q);
I = [];
A = [];
b = [];
Aeq = [T,dTf,eye(3)];
beq = tau ;
lb = [ff(:,1);dda(:,1);-1e10*ones(3,1)];
ub = [ff(:,2);dda(:,2); 1e10*ones(3,1)];
x0 = [f0;zeros(N,1);zeros(3,1)];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% �����ι滮����
options = optimset('Algorithm','interior-point-convex','Display','off');
x = quadprog(H,I,A,b,Aeq,beq,lb,ub,x0,options);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% �������
f = x(1:N);
df = f - f0;
da = x(N+1:2*N);
a = a0 + da;
da = da / d2r;
a = a / d2r;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ��aת��[-pi pi]���䣬�Է��㻭ͼ
for i = 1 : N
    while a(i) > 200
        a(i) = a(i) - 360;
    end
    while a(i) <-200
        a(i) = a(i) + 360;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% ʵ������
T =  thrusters_configuration(a*d2r, L);
% T1 = thrusters_configuration6(a*d2r);
tau_r = T*f;
% tau_r6 = T1*f;
dtau = tau_r - tau;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end


%% Bf΢��
function dTf = Get_Coefficients(f0,a0)
%
f1 = f0(1); f2 = f0(2); f3 = f0(3); 

a1 = a0(1); a2 = a0(2); a3 = a0(3); 
%
N = length(f0);
% config = [1.4 190; 0.87 205; 0.57 275; 1.7 17; 0.87 125; 0.57 94];
L =zeros(N, 2);
L = [-5.02 0; 11.956 2.7; 11.956 -2.7];
% for i = 1 : N
%     L(i, 1) = config(i, 1) * cos(config(i, 2)/180*pi);
%     L(i, 2) = config(i, 1) * sin(config(i, 2)/180*pi);
% end

dTf = zeros(3, N);
for i = 1 : N
    dTf(1, i) = -f0(i) * sin(a0(i));
    dTf(2, i) = f0(i) * cos(a0(i));
    dTf(3, i) = f0(i) * L(i, 2) * sin(a0(i)) + f0(i) * L(i, 1) * cos(a0(i));
end

end


%% ���þ���
function T = thrusters_configuration(a,L)
% function thruster_configuration is used to obtain the configuration
% matrix
N_Col = length(a);

if N_Col ~= length(L)
    error('the length of a and L do not match');
end

T = zeros(3,N_Col);   % initialization

for i = 1 : N_Col
    T(1,i) = cos(a(i));
    T(2,i) = sin(a(i));
    T(3,i) = L(i,1)*sin(a(i)) - L(i,2)*cos(a(i));
end

end



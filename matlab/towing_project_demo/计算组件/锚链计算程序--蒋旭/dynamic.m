clear all;
g=9.80665;
L=50;            %�ֶγ�m
H=1500;          %ˮ��
Time=100;        %ʱ�� s
p0=[0;0;0];      %����m
p1=[3800;0;0];
%% �������
[dt,k0,k1,k2,k3,k4,k5,CDn,CAn,pho_water,current,nt]=constant(Time);
%% �ֶβ���
[N,W,EA,D]=element_parameter;
%% ��������
[Ms]=mass_par_length(N,L,W);
[Ms_add]=mass_add(N,L,W);
 %% ����
 [G]=gravity(N,Ms,g);
%% �ʵ������ʼ��
[Y_N, p_init0]=initial_position_horizon(p0,p1,H,N);
% [Y_N]=initial_position(p0,p1,H,N);
%%%%%%%%%%%%%  ��ֵ���� ��ʼ����  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while 1
 %% ��Ԫʵ�ʳ��� 
 [l]=element_length(Y_N,N);
 %% ����֧����
 [Fb]=seabed(N,Ms,Y_N,H,g);
 %% ����
 [T]=tension(N,EA,L,l);
 %% �������ſɱȾ���
 [KT]=tension_K(N,EA,L,l,Y_N);
 %% ����֧�������ſɱȾ���
 [KB]=seabed_K(N,Ms,H,Y_N,g);
 %% �ܵ��ſ˱Ⱦ���
 K=KT+KB;
 %% �߽�����
 BON=zeros(3*N-3,1);
 BON(1:3,1)=-T(1:3,1:3)*Y_N(1:3,1);
 BON(3*N-5:3*N-3,1)=-T(3*N+1:3*N+3,3*N+1:3*N+3)*Y_N(3*N+1:3*N+3,1);
 %% �������
 FY=T(4:3*N,4:3*N)*Y_N(4:3*N,1)+G(4:3*N,1)+Fb(4:3*N,1)+BON;
%% ���
R=1/2;
Y_N1=Y_N(4:3*N,1)-K(4:3*N,4:3*N)\FY*R;
%% ������ֹ����
if norm(Y_N1-Y_N(4:3*N,1))<1e-7
    Y_N(4:3*N,1)=Y_N1;
    break
end
%% ��һ������
Y_N(4:3*N,1)=Y_N1;
end

p_static_final = zeros(N+1, 3);
for i = 1:N+1
    p_static_final(i, 1) = Y_N(3*i-2, 1);
    p_static_final(i, 2) = Y_N(3*i-1, 1);
    p_static_final(i, 3) = Y_N(3*i, 1);
end
title("static");

figure(2)
% plot(p_init0(:, 1), p_init0(:, 3))
% hold on
plot(p_static_final(:, 1), p_static_final(:, 3))
% TE1=EA(1,1)*(l(1,1)/L-1)*(Y_N(4:6,1)-Y_N(1:3,1))/norm(Y_N(4:6,1)-Y_N(1:3,1))+G(1:3,1);
% T1=norm(TE1);
%%%%%%%%%%%%%%%%%%�����������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% ��������
%% �����˶����¿׵�λ�� �� ������ʼ��
aa=zeros(3*N,nt+1);
vv=zeros(3*N,nt+1);
[P_initial]=motion(nt,N,Y_N,dt);
%% ʱ�����
for t=2:100
    
    t
    P_initial(4:3*N,t)=P_initial(4:3*N,t-1);
    Y_Nk=P_initial(:,t-1);
    Y_N=P_initial(:,t);
   %% ��Ԫʵ�ʳ��� ��һʱ���
    [lk]=element_length(Y_Nk,N);
   %% �������ſɱȾ��� ��һʱ���
    [KT]=tension_K_k(N,EA,L,lk,Y_Nk);
   %% ����֧�������ſɱȾ��� ��һʱ���
    [KB]=seabed_K_k(N,Ms,H,Y_Nk,g);
   %% �����ſɱȾ���ʹ�õ��������� ��һʱ���
   [Msk]=mass_K(N,Y_Nk,pho_water,D,CAn,L);
    K=KT(4:3*N,4:3*N)+KB(4:3*N,4:3*N)-k0*(Msk(4:3*N,4:3*N)+Ms_add(4:3*N,4:3*N))+10^4*eye(3*N-3);
   %% ������ʼ��
   delta_y=zeros(3*N-3,1);
   %% ��ǰʱ�����
while 1
   %% ��Ԫʵ�ʳ���  ��ʱ���
    [l]=element_length(Y_N,N);   
   %% ��������    ��ʱ���
   [MsN]=mass_N(N,Y_N,pho_water,D,CAn,L);
   %% ����֧����   ��ʱ���
   [Fb]=seabed(N,Ms,Y_N,H,g);
   %% ����   ��ʱ���
   [T]=tension(N,EA,L,l);
   %% �ʵ�����ȼ��ٶ�  ��ʱ���
   [aa,vv]=vel_acc(delta_y,vv,aa,k0,k1,k2,k3,k4,k5,t,N,Y_N,Y_Nk);
   %% ��ҷ��  ��ʱ���
   [Fd]=drag_F(N,Y_N,H,current,vv,t,pho_water,D,L,CDn);
   %% �߽�����  ��ʱ���
    BON=zeros(3*N-3,1);
    BON(1:3,1)=EA(1,1)*(1/L-1/l(1,1))*eye(3)*Y_N(1:3,1);
    BON(3*N-5:3*N-3,1)=EA(N,1)*(1/L-1/l(N,1))*eye(3)*Y_N(3*N+1:3*N+3,1);
   %% �������   ��ʱ���
   FY=T(4:3*N,4:3*N)*Y_N(4:3*N,1)+G(4:3*N,1)+Fb(4:3*N,1)+BON+Fd-(MsN(4:3*N,4:3*N)+Ms_add(4:3*N,4:3*N))*aa(4:3*N,t); 
   %% �ⷽ��    ��ʱ���
    y=-K\FY;
    Y_N(4:3*N,1)=Y_N(4:3*N,1)-K\FY;
    delta_y=delta_y-K\FY;   
    if norm(y)<1e-7
       aa(4:3*N,t)=k0*delta_y-k1*vv(4:3*N,t-1)-k2*aa(4:3*N,t-1);                          
       vv(4:3*N,t)=k3*delta_y+k4*vv(4:3*N,t-1)-k5*aa(4:3*N,t-1);
       break    
    end
end
P_initial(4:3*N,t)=Y_N(4:3*N,1); 
end

p_final = zeros(N+1, 3);
for i = 1:N+1
    p_final(i, 1) = Y_N(3*i-2, 1);
    p_final(i, 2) = Y_N(3*i-1, 1);
    p_final(i, 3) = Y_N(3*i, 1);
end

figure(1)
plot(p_init0(:, 1), p_init0(:, 3))
hold on
plot(p_final(:, 1), p_final(:, 3))
hold off
legend('��ʼ��״', '������״');
title('ê����״�仯');























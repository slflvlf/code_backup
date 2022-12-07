function [Waveloads] = MeanWaveDrift(WaveDirection, WindVelocity)
% Generate the second order mean wave drift forces in horizontal plane
%
% INPUT:
%  DirectionLoop         Direction in the DP_Cap_Main.m (Degree)
%  VelocityLoop          Wind Velocity in the DP_Cap_Main.m (m/s)

%
% OUTPUT:
%  Waveloads             Vector of wave loads (KN, KN, KN*m) in the
%                        horizontal plane
%%
load('WaveLoads.mat')
Hs = interp1(WavePara(:,1),WavePara(:,2),WindVelocity); %#ok<NODEF>
Tp = interp1(WavePara(:,1),WavePara(:,3),WindVelocity);
gamma = 3.3;
nfr = 200;
[omec,zeta_a,~,~] = jonspec_fixed(Hs,Tp,gamma,nfr);
%%
Dof1 = [1, 2, 3]';
[WaveDirection, omec, Dof1] = meshgrid(WaveDirection, omec, Dof1);
[Dir,Freq , Dof] = meshgrid(Dir, Freq, Dof); %#ok<NODEF>
P = interp3(Dir, Freq, Dof, MeanP, WaveDirection, omec, Dof1 );%前面的是样本点
P = squeeze(P);%删除单一维度
Zeta_a2 = zeta_a.^2;
Waveloads = Zeta_a2' * P / 1e3;%转换成kN

end %MeanWaveDrift
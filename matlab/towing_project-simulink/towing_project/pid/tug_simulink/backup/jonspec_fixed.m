function [omec,zeta_a,wavenum,phase]=jonspec_fixed(Hs,Tp,gamma,nfr,displayplots)
%JONSPEC Sample a JONSWAP spectrum at constant frequency intervals, generate random phases.
%
% INPUT:
%  Hs               Significant wave height (m)
%  Tp               Peak period (s)
%  gamma            gamma value for Jonswap spectrum
%  nfr              Number of spectrum samples
%  displayplots     1-display 0-don¡¯t display
%
% OUTPUT:
%  omec             Vector of harmonic wave frequencies (rad/s)
%  zeta_a           Vector of harmonic wave amplitudes
%  wavenum          Vector of harmonic wave numbers
%  phase            Vector of harmonic wave phases (random)(rad)

omegap = 2*pi/Tp;

% Select minimum and maximum omega
omemin = 0.01;
omemax = 1.47;
omega= linspace(omemin,omemax,nfr);

% Plot the spectrum with a fixed sampling
if displayplots
    plot(omega,jonswap(omega), 'LineWidth',2);
    grid on; hold on;
    title( 'JONSWAP spectrum');
    xlabel( '\omega(rad/s)');
    ylabel( 'S(\omega)(m^2 s/rad)');
    xlim([omemin omemax]);
end

domega = (omemax-omemin)/(nfr-1);
rng(23)
omec = omega + domega*rand(1,nfr) + domega;
omec = omec';

% Values of spectrum for each frequency
s = jonswap(omec);

% Plot samples over spectrum
if displayplots
  stem(omec,s, 'ro');
  plot(omemin,jonswap(omemin), 'go');
  plot(omemax,jonswap(omemax), 'go');
end

% Random phases of each wave component
rng(4423);
phase = 2*pi * rand(nfr,1);

% Wave number of each wave component
wavenum = omec.^2/9.81;

% Wave amplitude of each wave component
zeta_a = sqrt(2*s*domega);

%%%%%% Functions

function S_J=jonswap(omega)
% Calculate the value of the JONSWAP spectrum
S_PM=5/16 * Hs^2 * omegap^4 * omega.^-5.* ...
    exp(-5/4*(omega./omegap).^-4);
Agamma=1-0.287*log(gamma);
sigma= zeros( size(omega));
for i=1:length(omega)
    if omega(i)<=omegap
        sigma(i)=0.07;
    else
        sigma(i)=0.09;
    end
end
S_J=Agamma * S_PM.* gamma.^( exp(-0.5 * ...
((omega-omegap)./(sigma*omegap)).^2));
end

end
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
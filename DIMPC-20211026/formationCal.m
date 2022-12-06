function [form_cal]=formationCal(output_v1,output_v2)
% This function is used to calculate the distance between two vessels.
% Input
% X1 X2     the position of vessel_1 and vessel_2   
% Output
% Y         the distance between vessel_1 and vessel_2

step=size(output_v1,2);
form_cal=zeros(2,step);
for i=1:step
    form_cal(1,i)=sqrt((output_v1(1,i)-output_v2(1,i))^2+(output_v1(2,i)-output_v2(2,i))^2);%Œª÷√æ‡¿Î
    form_cal(2,i)=rad2deg(output_v1(3,i)-output_v2(3,i));%ÙºœÚΩ«∆´≤Ó
end

end
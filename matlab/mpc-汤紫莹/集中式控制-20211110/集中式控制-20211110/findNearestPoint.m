function [index]=findNearestPoint(velPos,refPoints)

global t
%�ҵ�����ǰ������Ĳο��켣�� 
col=size(refPoints,2);%����
dist=zeros(2,col);

dist(1,:)=sqrt((velPos(1)-refPoints(1,:)).^2+(velPos(2)-refPoints(2,:)).^2);
dist(2,:)=1:col;

sortedDist=(sortrows(dist'))';
index=max(sortedDist(2,1:2));%������������� �±����Ǹ�

if t==1 || t==2 
    index=1;
end

end
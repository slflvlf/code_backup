function [index]=findNearestPoint(velPos,refPoints)
%找到船舶前方最近的参考轨迹点 

col=size(refPoints,2);%列数
dist=zeros(2,col);

dist(1,:)=sqrt((velPos(1)-refPoints(1,:)).^2+(velPos(2)-refPoints(2,:)).^2);
dist(2,:)=1:col;

sortedDist=(sortrows(dist'))';
index=max(sortedDist(2,1:2));%最近的两个点中 下标大的那个

end
function [P_initial]=motion2(obj, nt,N,Y_N,p0_next, p1_next)

P_initial=zeros(3*N+3,nt+1);


% for i=2:nt+1
% 
%     P_initial(1:3,i) = p0_next;
%     P_initial(3*N-2,i)=Y_N(3*N-2,1);
%     P_initial(3*N-1,i)=Y_N(3*N-1,1);
%     P_initial(3*N,i)=Y_N(3*N,1);
%     P_initial(3*N+1:end,i)=p1_next;
% end
P_initial(:,1)=Y_N;
P_initial(:, 2) = Y_N;
P_initial(1:3, 2) = p0_next;
P_initial(3*N+1:end, 2) = p1_next;
end
function [pd, dot_pd, ddot_pd] = ref_model(T, t, p0, r)
    
    global fai_old;
    pd = r * (1 -  exp(-t / T)) + p0;
    
%     if(p0(3)<=180&&p0(3)>=90&&r(3)>=-180&&r(3)<=-90)
%         if(pd(3)-fai_old<=0)
%             pd(3) = pd(3)-180;
%         end
%     if(p0(3)<=-90&&p0(3)>=-180&&r(3)<=180&&r3)>=90)
%         pd(3) = 
        
        

    dot_pd = r / T * exp(-t / T);
    
    ddot_pd = -r / T / T * exp(-t / T);

end
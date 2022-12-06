function [pd, dot_pd, ddot_pd] = ref_model(T, t, p0, r)

    pd = r * (1 -  exp(-t / T)) + p0;

    dot_pd = r / T * exp(-t / T);
    
    ddot_pd = -r / T / T * exp(-t / T);

end
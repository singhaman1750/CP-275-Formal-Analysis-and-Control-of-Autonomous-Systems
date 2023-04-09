function [beta] = switching_func(stl,t)
% Switching Function Summary of this function goes here
% Detailed explanation goes here
alpha = 2;
sz_stl = size(stl);
beta = zeros(1,sz_stl(2));
for i=1:sz_stl(2)
    if (i == 1)
       beta(i) =   1/(1+exp(-alpha*(t + (stl(i).time_range(1) + 0.1)))) ...
                 - 1/(1+exp(-alpha*(t - (stl(i).time_range(2) + 0.1))));
    %if (t >= stl(i).time_range(1) + 0.1) && (t <= stl(i).time_range(2) + 0.1)  
    %   beta(i) = 1;
    %else 
    %   beta(i) = 0;
    %end
    else
       beta(i) =   1/(1+exp(-alpha*(t - (stl(i).time_range(1) + 0.1)))) ...
                 - 1/(1+exp(-alpha*(t - (stl(i).time_range(2) + 0.1))));
    %if (t >= stl(i).time_range(1) + 0.1) && (t <= stl(i).time_range(2) + 0.1)  
    %   beta(i) = 1;
    %else 
    %   beta(i) = 0;
    %end
    end
end
end
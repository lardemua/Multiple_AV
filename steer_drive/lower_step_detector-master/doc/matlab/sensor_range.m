clear all
close all

%theta = 0:pi/100:pi/2;
l = 1.5;
x = 0:pi/180:pi/2;
l_arr = zeros(size(x));
idx = 1;

for theta = 0:pi/180:pi/2
    theta_cur = pi/2-theta;
    if(theta_cur > 0.00001)
        l_arr(idx) = l / sin(theta_cur);
    else
        l_arr(idx) = l_arr(idx - 1);
    end
    idx = idx + 1;
end

x_degree = x * 180 / pi;
plot(x_degree, l_arr);

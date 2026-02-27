function [x_next] = unicycle_dynamics(x,u,dt, mu, sigma)
    x_dot=[x(4)*cos(x(3));x(4)*sin(x(3));u(1);u(2)];
%     x_next = x + x_dot.*dt + mvnrnd(mu,sigma)';

    %Uniform noise
%     a = 0.1;
%     low  = [-sqrt(3)*a/5; -sqrt(3)*a/5; -sqrt(3)*a; -sqrt(3)*a];
%     high = [ sqrt(3)*a/5;  sqrt(3)*a/5;  sqrt(3)*a;  sqrt(3)*a];
    a = 0.15;%0.1732; %0.15;  %sqrt(3) *0.1;
    low  = [-a/5; -a/5; -a; -a];
    high = [ a/5;  a/5; a;  a];
    noise = low+ (high-low).* rand(4, 1);
    
%     lwb=-0.2;ub=0.2;
%     noise = lwb + (ub - lwb) .* rand(4, 1);

    %Laplace noise
%     noise = -0.03 * sign(rand(3, 1) - 0.5) .* log(1 - 2 * abs(rand(3, 1) - 0.5));
%     a = 2; b = 5;
%     lb=-0.05;ub=0.05;
    
    %GMM NOISE
%     [noise,sigma_mix]=zero_mean_gmm_noise(4);
%     noise =  noise';

    %GAUSSIAN NOISE
%     noise =  mvnrnd(mu,sigma)';   %betarnd(a, b, 3, 1) * (ub - lb) + lb;
    
%     noise(1)=0;
%     noise(2)=0;    
%     noise(1:2)=0;
    
    

%     disp(noise)
%     

    
    x_next = x + x_dot.*dt + noise ;
end



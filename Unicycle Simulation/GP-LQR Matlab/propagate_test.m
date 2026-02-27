close all;
% Parameters

P = 200;               % Number of realizations
dt = 0.05;            % Time step
t_0 = 0;  
t_f = 10;  
N = (t_f - t_0) / dt;


t = linspace(t_0, t_f, N + 1);
%GP_GMM0.05_2.mat
U = load('GP_3_U0.1.mat').solution.U; 
X = load('GP_3_U0.1.mat').solution.X;
cov = load('GP_3_U0.1.mat').solution.cov;

% U = load('GP_GMM0.05_2.mat').solution.U; 
% X = load('GP_GMM0.05_2.mat').solution.X;
% cov = load('GP_GMM0.05_2.mat').solution.cov;

% U = load('GP_gaussian_0.05.mat').solution.U; 
% X = load('GP_gaussian_0.05.mat').solution.X;
% cov = load('GP_gaussian_0.05.mat').solution.cov;

%% Load LQR gain along trajectory
K_nom=load('K_array_right_N200.mat');

% Initial state [vx, px, vy, py, omega, theta]
x0 = [0; 0.4 ; 0;0 ];


% sol_GP = load('solution_MLE_GMM0.01sq.mat').solution;

% Noise parameters for true dynamics
mu = zeros(4,1);%[0.0008;-0.0044;0.0005];   %zeros*ones(3,1);
sigma = 0.05^2*eye(4);  %1.0e-03 * [0.8368    0.0068   -0.0550; 0.0068    0.7649   -0.0453;-0.0550   -0.0453    0.8485]; %0.00005*eye(3);   

% Noise parameters for GP dynamics
% mu_hat = zeros(3,1); %load('mean_est.mat').mean_est; %zeros(6,1);
% sigma_hat = 0.00005*eye(3); %load('cov_est.mat').cov_est;%0.2*eye(6);

% Storage for trajectories
traj_true = zeros(4, N, P);
% traj_gp = zeros(3, N, P);
controls_true = zeros(2,N-1,P);

K_array = cell(1, N);
K_array{1} = zeros(2,4);
% Simulation loop
for p = 1:P
    x_true = x0;
%     x_gp = x0;
    traj_true(:,1,p) = x_true;
%     traj_gp(:,1,p) = x_gp;
    for k = 2:N
%         K_array{k}=compute_K(X(:,k-1),U(:,k-1),dt);
%         u_k = U(:,k-1)- K_array{k}* (x_true - X(:,k-1)) ;
        
        u_k = U(:,k-1)- K_nom.K_array{k}* (x_true - X(:,k-1)) ;
        
%         try
%             K_array{k}=compute_K(X(:,k-1),U(:,k-1),dt);
%             u_k = U(:,k-1)- K_array{k}* (x_true - X(:,k-1)) ;
%         catch ME
%             u_k = U(:,k-1)- K_nom.K_array{k}* (x_true - X(:,k-1)) ;
%         end
        

        
        x_true = unicycle_dynamics(x_true, u_k, dt, mu, sigma);
%         x_gp = unicycle_GPdynamics(x_gp, u_k, dt, mu_hat, sigma_hat);
        traj_true(:,k,p) = x_true;
        controls_true(:,k,p) = u_k;
%         traj_gp(:,k,p) = x_gp;
    end
end

save('GP3_for_U0.1_new.mat', 'traj_true','controls_true', 'X', 'U','cov');
% save('GP4_for_GMM0.05_new.mat', 'traj_true','controls_true', 'X', 'U','cov');
% Plotting
figure;
hold on;
% title('Trajectories under Unicycle Dynamics');
xlabel('x');
ylabel('y');
grid on;
gray = [.7 .7 .7];
obs_center = [5, 0];
obs_radius = 1.2;
th = 0:pi/50:2*pi;
xunit = obs_radius * cos(th) + obs_center(1);
yunit = obs_radius * sin(th) + obs_center(2);
h = fill(xunit, yunit, gray );

% yline(2, 'r--', 'LineWidth', 1.5);  % horizontal line at y = 2
fill([0,10,10,0], [2,2,4,4], gray, 'EdgeColor', 'k');

maroon= [0.6350 0.0780 0.1840];
plot(0, 0.4, 'go', 'MarkerSize', 4, 'MarkerFaceColor', 'g'); % start
plot(10, 0.4, 'o', 'MarkerSize', 4, 'MarkerFaceColor', maroon); % goal

% Plot P trajectories for true dynamics
for p = 1:P
    plot(traj_true(1,:,p), traj_true(2,:,p),'b', 'LineWidth', 0.5);
end

% Plot P trajectories for GP dynamics
% for p = 1:P
%     plot(traj_gp(1,:,p), traj_gp(2,:,p), 'r', 'LineWidth', 1);
% end

plot(X(1,:), X(2,:),'r--', 'LineWidth', 1.5);

p = 0.9;
s = chi2inv(p, 2);
for k = 1:2:length(t)
    mu = X(1:2, k);  % mean [x; y]
    Sigma = cov{k}(1:2, 1:2); 
    [V, D] = eig(Sigma);
    theta = linspace(0, 2*pi, 100);
    circle = [cos(theta); sin(theta)];
    ellipse = V * sqrt(D * s) * circle;
    ellipse = ellipse + mu;
    
    plot(ellipse(1, :), ellipse(2, :), 'r', 'LineWidth', 0.05);
end
%  
view([90 -90])

% legend('True Dynamics','GP Dynamics');
axis equal;
hold off;
% ylim([-4,4]);


function [samples,Sigma_mix] = zero_mean_gmm_noise(d, N)
    % Generate N samples from a d-dimensional zero-mean Gaussian mixture
    %
    % Inputs:
    %   d - dimension of each sample
    %   N - number of samples
    %
    % Output:
    %   samples - (N x d) matrix, each row is a sample

    if nargin < 2
        N = 1;  % default number of samples
    end

    % Define mixture components
    K = 3;                             % number of components
    weights = [0.1, 0.8, 0.1];         % must sum to 1

    % Define component means (must be zero-mean overall)
    mu = zeros(K, d);
    mu(1, :) = -0.5;                     % vector [-1, ..., -1]
    mu(2, :) =  0;                     % zero vector
    mu(3, :) =  0.5;                     % vector [1, ..., 1]


    % Verify zero-mean
    mix_mean = sum(weights' .* mu, 1);
    assert(norm(mix_mean) < 1e-10, 'Mixture is not zero mean!');

    % Define covariances (can be identity or varied)
    Sigma = zeros(d, d, K);
    for k = 1:K
%         Sigma_k = 1e-6*eye(d); %zeros(d, d);
%         Sigma_k(end-1:end, end-1:end) = (0.01^2) * eye(2); % noise only in last 2 dims
        Sigma(:, :, k) = 0.05^2*eye(d);     %Sigma_k; %0.01^2*eye(d);      % isotropic Gaussian
    end

    % Gaussian estimate using moment matching% Compute mixture covariance
    Sigma_mix = zeros(d, d);
    for k = 1:K
        mu_k = mu(k, :)';
        Sigma_mix = Sigma_mix + weights(k) * (Sigma(:, :, k) + mu_k * mu_k');
    end

    % Sampling
    samples = zeros(N, d);
%     disp(N);
    for i = 1:N
        k = find(rand <= cumsum(weights), 1);       % pick component
        samples(i, :) = mvnrnd(mu(k, :), Sigma(:, :, k));  % sample from component
    end
end

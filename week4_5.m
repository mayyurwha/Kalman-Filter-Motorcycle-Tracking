load /Users/Xtyalls_city/Downloads/motorcycle.mat
 
% Generate a synthetic OU process
tau = 1; % time constant of the OU process
%dt = 0.1; % time step between observations
%t = 0:dt:10; % time vector
t = x; % time vector
xtrue = y;
 t(134) = t(133);
% Add measurement noise
sigma = 0.1; % standard deviation of measurement noise
z = xtrue + sigma*randn(size(xtrue));
 
% Define the state vector
x = [0; 0];
 
% Define the observation model
H = [1 0];
%dt=0.1;
% Define the system dynamics model

 
% Define the measurement noise covariance matrix
R = sigma^2;
 
% Initialize the state estimate and the error covariance matrix
xhat = [0; 0];
P = eye(2);
 
% Iterate through the dataset
for i = 1:length(z)
    % Obtain the measurement at the current time step
    y = z(i);
    dt = t(i+1) - t(i);
    F = [1 dt; 0 exp(-dt/tau)];
    G = [0; sqrt((1-exp(-2*dt/tau))/(2*tau))];
    Q = G*G';
    % Predict the state at the next time step
    xhatminus = F*xhat;
    Pminus = F*P*F' + Q;
    
    % Update the state estimate based on the measurement
    K = Pminus*H'/(H*Pminus*H' + R);
    xhat = xhatminus + K*(y - H*xhatminus);
    P = (eye(2) - K*H)*Pminus;
    
    % Store the estimated state
    est(i,:) = xhat';
end
 
% Plot the results
figure;
plot(t(1:133), xtrue, 'k--', 'LineWidth', 2); hold on;
plot(t(1:133), z, 'b', 'LineWidth', 2);
plot(t(1:133), est(:,1), 'r', 'LineWidth', 2);
legend('True state', 'Measured state', 'Estimated state');
xlabel('Time (s)');
ylabel('Position');


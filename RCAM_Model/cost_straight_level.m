% Based on Christopher Lum's Tutorial: https://www.youtube.com/watch?v=YzZI1V2mJw8
% Cost function for straight and level flight
function [F0] = cost_straight_level(Z)

X = Z(1:9);
U = Z(10:14);

xdot = RCAM_model(X, U);
theta = X(8);
Va = sqrt( X(1)^2 + X(2)^2 + X(3)^2); % Airspeed
alpha = atan2(X(3), X(1));
gam = theta - alpha; % Flight path angle, Works if wings are straight and level

Q = [xdot;
    Va-85;
    gam;
    X(2);
    X(7);
    X(9)];

H = diag(ones(1,14)); % Penalty Parameter Matrix. Set all alpha's to 1

F0 = Q'*H*Q;
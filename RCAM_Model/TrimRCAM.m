% Followed by Christopher Lum's tutorial: https://www.youtube.com/watch?v=YzZI1V2mJw8
clear;clc;close all

% Initialize z_guess
initialization = 1; % 0: start from initial guess
                    % 1: Used saved values from last run
                    
if (initialization == 0)
    Z_guess = zeros(14,1);
    Z_guess(1) = 85; % Guess that u (fwd velocity) is near 85 m/s
else
    load trim_values_sl
    Z_guess = [XStar;UStar];
end

% Solve unconstrained optimization problem
[ZStar, f0] = fminsearch('cost_straight_level', Z_guess,...
    optimset('TolX', 1e-10, 'MaxFunEvals', 10000, 'MaxIter',10000))

XStar = ZStar(1:9);
UStar = ZStar(10:14);

% Verify this statisfies constraints
XdotStar = RCAM_model(XStar, UStar)
VaStar = sqrt( XStar(1)^2 + XStar(2)^2 + XStar(3)^2)
gammaStar = XStar(8) - atan2(XStar(3), XStar(1))
vStar = XStar(2)
phiStar = XStar(7)
psiStar = XStar(9)

save trim_values_sl XStar UStar

% X:
%84.9905 u 
% 0.0000 v
% 1.2713 w (Indicates we are at a slight non-zero angle of attack), needed
% for lift to aid in offsetting the weight of the aircraft
%-0.0000
% 0.0000
% 0.0000
% 0.0000
% 0.0150 % Pitch Euler angle is non-zero, slight non-zero angle of attack
% present. No climb angle so this might be the exact AoA (radians)
% 0.0000 % Flying north

% U
% 0.0000 % No AIL input
%-0.1780 % Non-Zero HSTAB deflection
% 0.0000 % No RUD
% 0.0821 % Both throttles are matched
% 0.0821


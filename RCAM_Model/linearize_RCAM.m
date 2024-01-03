% Linearize RCAM Model about straight and level flight

clear; clc; close all; 

%% Define linearization point
XDOT0 = [
    0
    0
    0
    0
    0
    0
    0
    0
    0
    ];

X0 = [
    84.9905
    0
    1.2713
    0
    0
    0
    0
    0.0150
    0
    ];

U0 = [
    0
    -0.1780
    0
    0.0821
    0.0821
    ];

%% Define Pertubation matrices
% How much we want to perturb each function in each direction
dxdot_matrix = 10e-12*ones(9,9);
dx_matrix= 10e-12*ones(9,9);
du_matrix = 10e-12*ones(9, 5);

[E, A_P, B_P] = ImplicitLinmod(@RCAM_model_implicit, XDOT0, X0, U0, dxdot_matrix, dx_matrix, du_matrix);

% State space matrices
A = -inv(E) * A_P;
B = -inv(E) * B_P;

A
B

    
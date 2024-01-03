function [E, A_P, B_P] = ImplicitLinmod(MY_FUN, XDOT0, X0, U0, DXDOT, DX, DU)
% Numerically linearizes Nonlinear 6-DOF function (written in implicit
% form) using symmetric difference quotients
% Inputs:
%   XDOT0, X0, U0: Expansion point for linearization
%   DXDOT, DX, DU: Pertubation distances for each variable in each function
% Outputs:
%   A = -inv(E)*A_P
%   B = -inv(E)*B_P

% Number of states and controls
n = length(X0);
m = length(U0);

% Calculate E Matrix
% Calculates function's dependence on pertubations on X_DOT
E = zeros(n, n); 

% fill in each element of the matrix individually
for i = 1:n % Loop through each function in F (our system of equations)
    for j = 1:n % Loop through all dependent variables of X_DOT
        
        % Magnitude of the pertubation to use
        dxdot = DXDOT(i, j); % for function i, independent variable j
        
        % Define both direction pertbutation vectors
        % Column determines which element of xdot we are perturbing
        
        % Start at expansion point
        xdot_plus = XDOT0; 
        xdot_minus = XDOT0;
        
        xdot_plus(j) = xdot_plus(j) + dxdot;
        xdot_minus(j) = xdot_minus(j) - dxdot;
        
        % Calculate F(row) (xdot_plus, x0, u0)
        F = feval(MY_FUN, xdot_plus, X0, U0);
        F_plus_keep = F(i); % keep the one we are focused on 
        
        % Calculate F(row) (xdot_minus, x0, u0)
        F = feval(MY_FUN, xdot_minus, X0, U0);
        F_minus_keep = F(i);
        
        % Calculate E(row, col) rise/run
        E(i, j) = (F_plus_keep - F_minus_keep) / (2*dxdot);
    end
end

% Calculate A Prime matrix
% now we perturb X
A_P = zeros(n, n);

for i = 1:n
    for j = 1:n
        dx = DX(i, j);
        
        x_plus= X0;
        x_minus = X0;
        
        x_plus(j) = x_plus(j) + dx;
        x_minus(j) = x_minus(j) - dx;
        
        F = feval(MY_FUN, XDOT0, x_plus, U0);
        F_plus_keep = F(i);

        F = feval(MY_FUN, XDOT0, x_minus, U0);
        F_minus_keep = F(i);
        
        A_P(i, j) = (F_plus_keep - F_minus_keep) / (2*dx);
    end
end

% Calculate B Prime matrix
% perturb U

B_P = zeros(n, m); 

for i = 1:n
    for j = 1:m
        
        du = DU(i, j);
        
        u_plus = U0;
        u_minus = U0;
        
        u_plus(j) = u_plus(j) + du;
        u_minus(j) = u_minus(j) - du;
        
        F = feval(MY_FUN, XDOT0, X0, u_plus);
        F_plus_keep = F(i);
        
        F = feval(MY_FUN, XDOT0, X0, u_minus);
        F_minus_keep = F(i);
        
        B_P(i, j) = (F_plus_keep - F_minus_keep) / (2*du);
    end
end


        



function [XDOT] = RCAM_model(X, U)
%   From Chris Lum's Video on Youtube: 
%
%
%
%
%

% Notes on reference frames:
% F_e: Earth Fixed Reference Frame:
% Origin (O_E) is located on the runway long axis 
% x_e points north, assume runway is also pointed north
% z_e is pos downward, and y_e is positive east
%
% F_b: Body Fixed Reference Frame
% Origin O_B is at the vehicle CG. X_b is fwd, Z_b is pos
% down and y_b is pos right (starboard side)
%
% F_m: Measurement frame
% O_M is at leading edge of the cbar.
% X_M is positive pointing backwards, Y_M point pos starboard
% Z_M pos pointingup

% Extract and define the state
% Translational Velocity in Body Frame
x1 = X(1); % u
x2 = X(2); % v
x3 = X(3); % w
% Rotational Rates in Body Frame
x4 = X(4); % p
x5 = X(5); % q
x6 = X(6); % r
% Euler Angles in the Inertial or NED Frame
x7 = X(7); % phi
x8 = X(8); % theta
x9 = X(9); % psi

% Extract and define the control
u1 = U(1); % d_A (AIL)
u2 = U(2); % d_T (HSTAB)
u3 = U(3); % d_R (Rudder)
u4 = U(4); % d_th1 (throttle 1)
u5 = U(5); % d_th2 (throttle 2)


% Define AERO and Nominal Vehicle Constants
m = 120000; % Aircraft Total Mass (kg)

cbar = 6.6; % Mean Aerodynamic Chord (m)
lt = 24.8; % Distance from AC of tail and body (m)
S = 260; % Wing planform area (m^2)
St = 64; % Tail planform area (m^2)

% Table 2.5 in RCAM document for Nominal CG values
Xcg = 0.23*cbar; % x position of CG in Fm (m)
Ycg = 0; % y position of CG in Fm (m)
Zcg = 0.10*cbar;% z position of CG in Fm (m)

Xac = 0.12*cbar;
Yac = 0;
Zac = 0;

% Engine Constants (Force Application Points)
Xapt1 = 0; % X Position of engine 1 force in Fm (m)
Yapt1 = -7.94; % Y position of engine 1 force in Fm (m)
Zapt1 = -1.9; % Z position of engine 1 force in Fm (m)

Xapt2 = 0; % x pos of engine 2 force in Fm (m)
Yapt2 = 7.94; % y pos of engine 2 force in Fm (m)
Zapt2 = -1.9; % z pos of engine 2 force in Fm (m)

% Other Constants:
rho = 1.225; % Air density at sea level (kg/m^3), assume air density does not change with alt
g = 9.81; % Gravitational Acceleration (m/s^2)j
depsda = 0.25; % Change in downwash wrt alpha (rad/rad)
alpha_L0 = -11.5*(pi/180); % Zero life angle of attack (rad)
n = 5.5; % Slope of linear region of lift slope (Refer to Cl_wing vs alpha curve)
a3 = -768.5; % Coefficient of alpha^3
a2 = 609.2; % Coefficient of alpha^2
a1 = -155.2; % Coeffiicent of alpha^1
a0 = 15.212; % Coefficient of alpha^0 (Different from RCAM document)
alpha_switch = 14.5*(pi/180); % Alpha where lift slope goes from linear to non-linear

% % Control Limits and Saturations definitions
% 
% % Aileron
% u1min = -25*(pi/180);
% u1max = 25*(pi/180);
% 
% % HStab
% u2min = -25*(pi/180);
% u2max = 10*(pi/180);
% 
% % Rudder
% u3min = -30*(pi/180);
% u3max = 30*(pi/180);
% 
% % Throttle 1
% u4min = 0.5*(pi/180);
% u4max = 10*(pi/180);
% 
% % Throttle 2
% u5min = 0.5*(pi/180);
% u5max = 10*(pi/180);
% 
% % Apply Saturation to controls
% if (u1 > u1max)
%     u1 = u1max;
% elseif (u1 < u1min)
%     u1 = u1min;
% end
% 
% if (u2 > u2max)
%     u2 = u2max;
% elseif (u2 < u2min)
%     u2 = u2min;
% end
% 
% if (u3 > u3max)
%     u3 = u3max;
% elseif (u3 < u3min)
%     u3 = u3min;
% end
% 
% if (u4 > u4max)
%     u4 = u4max;
% elseif (u4 < u4min)
%     u4 = u4min;
% end
% 
% if (u5 > u5max)
%     u5 = u5max;
% elseif (u5 < u5min)
%     u5 = u5min;
% end

% Define all intermediate variables
% Calculate Airspeed
Va = sqrt(x1^2 + x2^2 + x3^2);

% Calculate alpha and beta
alpha = atan2(x3, x1); % Angle of Attack
beta = asin(x2/Va); % Works for beta in small ranges

% Calculate Dynamic Pressure
Q = 0.5*rho*Va^2;

% Also define the vectors wbe_b and V_b
wbe_b = [x4;x5;x6]; % [P Q R], angular rates in body frame
V_b = [x1;x2;x3]; % Translational Velocity in Body Frame

% Calculate Aerodynamic Force Coefficients
% Calculate CL_wb in Fs
if (alpha <= alpha_switch)
    CL_wb = n*(alpha - alpha_L0); % Linear Portion
else
    CL_wb = a3*alpha^3 + a2*alpha^2 + a1*alpha + a0; % Nonlinear
end

% Calculate CL_t in Fs
epsilon = depsda*(alpha - alpha_L0);
alpha_t = alpha - epsilon + u2 + 1.3*x5*lt/Va;
CL_t = 3.1*(St/S)*alpha_t;

% Total Lift Coefficient in Fs
CL = CL_wb + CL_t; % Normalized the same way

% Total Drag Coefficient (Neglecting Drag from Tail) in Fs
CD = 0.13 + 0.07*(5.5*alpha + 0.654)^2; 

% Calculate Sideforce in Fs
CY = -1.6*beta + 0.24*u3;

% Dimensionalize the Aero Coefficients: AERO Forces in Fs

FA_s = [-CD*Q*S; % X
        CY*Q*S;  % Y 
        -CL*Q*S];% Z
% Rotate these forces into Fb (body frame)

% Define rotation matrix from stab to body frame
C_bs = [cos(alpha) 0 -sin(alpha);
        0 1 0;
        sin(alpha) 0 cos(alpha)];

FA_b = C_bs*FA_s;   
    
% Calculate Aero Moment Coefficients about the AC

% Calculate the moments in Fb. Define eta, dCMdx and dCMdu

% Define static moment effect/stability vector
eta1 = -1.4*beta;
eta2 = -0.59 - (3.1*(St*lt)/(S*cbar))*(alpha - epsilon);
eta3 = (1 - alpha * (180/(15*pi)))*beta;
eta = [eta1;
       eta2;
       eta3];
   
% Coefficient on how moments change wrt angular rate change
dCMdx = (cbar/Va) * [-11 0 5;
                     0 (-4.03*(St*lt^2)/(S*cbar^2)) 0;
                     1.7 0 -11.5];
                 
% Control authority/effectiveness
dCMdu = [-0.6 0 0.22;
          0 (-3.1*(St*lt)/(S*cbar)) 0;
          0 0 -0.63];
% Calculate CM = [Cl; Cm; Cn] about the AC in FB
CMac_b = eta + dCMdx*wbe_b + dCMdu*[u1;u2;u3];

% Dimensionalize and get Aero Moment about AC
MAac_b = CMac_b*Q*S*cbar;

% Moment Transfer from AC to CG
rcg_b = [Xcg; Ycg; Zcg];
rac_b = [Xac; Yac; Zac];
MAcg_b = MAac_b + cross(FA_b, rcg_b - rac_b);


% Engine and Propulsion Forces/Moments
% Calcualte Thrust of Each Engine
F1 = u4*m*g;
F2 = u5*m*g;

% Assume engine thrust is aligned with Fb:
FE1_b = [F1;0;0];
FE2_b = [F2;0;0];

FE_b = FE1_b + FE2_b;

% Engine moment present due to offset of engine thrust from CG:
mew1 = [Xcg - Xapt1;
        Yapt1 - Ycg;
        Zcg - Zapt1];

mew2 = [Xcg - Xapt2;
        Yapt2 - Ycg;
        Zcg - Zapt2];

MEcg1_b = cross(mew1, FE1_b);
MEcg2_b = cross(mew2, FE2_b);

MEcg_b = MEcg1_b + MEcg2_b;


% Gravity Effects
% Calculate Gravitational forces in the body frame. 
% Assume gravity is applied at CG so no moment present

g_b = [-g*sin(x8);
        g*cos(x8)*sin(x7);
        g*cos(x8)*cos(x7)];
    
Fg_b = m*g_b;


% State Derivatives

% Define the Inertia Matrix
% Symmetrical about the X-Z body plane
% 
Ib = m*[40.07 0 -2.093;
        0 64 0;
        -2.0923 0 99.92];

% Inverse of Inertia Matrix 
% We took inv(Ib) and hardcoded it in here to save run
% time
% invIB = inv(Ib);
invIb = (1/m) * [0.0249836 0 0.000523151;
                0 0.015625 0;
                0.000523151 0 0.010019];
            
% Form F_b (all forces in Fb) and calculate udot, vdot, wdot
F_b = Fg_b + FE_b + FA_b;
x1to3dot = (1/m)*F_b - cross(wbe_b, V_b);

% Form Mcg_b (all moments about CG in Fb) and calculate pdot, qdot, and
% rdot
Mcg_b = MAcg_b + MEcg_b;
x4to6dot = invIb*(Mcg_b - cross(wbe_b, Ib*wbe_b));

% Calculate phidot, thetadot, and psidot

% Define Euler Kinematical Equations
H_phi = [1 sin(x7)*tan(x8) cos(x7)*tan(x8);
         0 cos(x7) -sin(x7);
         0 sin(x7)/cos(x8) cos(x7)/cos(x8)];
     
x7to9dot = H_phi*wbe_b;


% Assmeble final output
XDOT = [x1to3dot;
        x4to6dot;
        x7to9dot];
    

     


    
    
   



































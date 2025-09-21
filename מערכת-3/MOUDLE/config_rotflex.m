%% CONFIG_ROTFLEX
%
% Find the moment of inertia and stiffness parameters of the SRV02 Rotary 
% Flexible Joint plant.
%
% ************************************************************************
% Input parameters:
% SPRING            Spring type used (default is 2, set to 1, 2 or 3)
% HUB_POSITION      Position of spring on hub ('A', 'B' or 'C')
% ARM_POSITION      Position of Spring Anchor on Arm, set to 1, 2 or 3
% ARM_LOAD          Location of short arm on main arm (set to 0 if not 
%                   connected, 1, 2 or 3 for each other position).
% ROTFLEX_OPTION    Either ROTFLEX or ROTFLEX-E, i.e. w/ potentiomter or w/
%                   encoder.
%
% ************************************************************************
% Output parameters:
% Jarm          Moment of inertia of flexible joint                         (kg.m^2)
% Barm          Viscous damping of flexible link                            (N.m/(rad/s))
% Ks            Stiffness of flexible joint                                 (kg.m^2/s)
% RtflxOp       Sets Simulink modle for potentiometer or encoder use.
% K_POT_ROT     Potentiometer sensitivity for the standard ROTFLEX module   (rad/V)
%
% Copyright (C) 2010 Quanser Consulting Inc.
% Quanser Consulting Inc.
%
%%
%
function [ Jarm, Barm, Ks, RtflxOp, K_POT_ROT ] = config_rotflex ( SPRING, HUB_POSITION, ARM_POSITION, ARM_LOAD, ROTFLEX_OPTION )
%
%
%% Spring Constants
% spring 1 used, LEAST STIFFNESS
aL(1) = 0.0254*1.0;
aFr(1) = 0.85;
aK(1) = 0.22*1000*.85;
% spring used E0240-026-1250 MEDIUM STIFFNESS (DEFAULT)
aL(2) = 0.0254*1.25;
aFr(2) = 1.33;
aK(2) = 0.368*1000*.85;
% spring used E0240-029-1250 STIFFEST
aL(3) = 0.0254*1.25;
aFr(3) = 1.78;
aK(3) = 0.665*1000*.85;
%
%% Hub/Arm Position Constants
% POSITION ON HUB (BASE)
ad(1) = 0.0254*1.25; % from hinge to spring insertion on plate, along arm
ad(2) = 0.0254*1.00; % from hinge to spring insertion on plate, along arm
ad(3) = 0.0254*0.75; % from hinge to spring insertion on plate, along arm
%
% POSITION ON ARM (LINK)
aR(1) = 0.0254*4.0; % from hinge to spring insertion on arm
aR(2) = 0.0254*3.5; % from hinge to spring insertion on arm
aR(3) = 0.0254*3.0; % from hinge to spring insertion on arm
%
% Fixed distance
r = 0.0254*1.25; % Fixed distance
%
%% Arm Mass and Length Constants
m_main_arm = 0.064; % 64 grams
l_main_arm = 11.75 * 0.0254; % 12 inches
m_short_arm = 0.03; % 30 grams
l_short_arm = 6 * 0.0254; % 6 inches
D_short_arm (1) = 9 * 0.0254; % 9 inches away from rotation axis
D_short_arm (2) = 10 * 0.0254; % 10 inches away from rotation axis
D_short_arm (3) = 11 * 0.0254; % 11 inches away from rotation axis
%
%% Find Stiffness
L = aL(SPRING);
Fr = aFr (SPRING);
K = aK(SPRING);
% Setting to current hub position
if strcmp (HUB_POSITION, 'A')
    d = ad(1);
elseif strcmp (HUB_POSITION, 'B')
    d = ad(2);
elseif strcmp (HUB_POSITION, 'C')
    d = ad(3);
end
% Setting to current arm position
R = aR(ARM_POSITION);
D = r^2+(R-d)^2;
%
% Stiffness of rotary flexible joint
% note: derived in the Maple file "K_Stiff_Linear.mws"
Ks = 2*R/D^(3/2)*(D*d-R*r^2)*Fr+2*R/D^(3/2)*(D^(3/2)*d-D*L*d+R*r^2*L)*K;
%
%% Find Equivalent Moment of Inertia
% Main arm moment of inertia (kg.m^2)
J_main_arm = m_main_arm*l_main_arm^2 / 3; % J = m*l^2 / 3
% Short arm moment of inertia (kg.m^2)
J_short_arm = m_short_arm*l_short_arm^2 / 12; % J = m*l^2 / 12 - This is the short arm's J about its center of gravity.
% Complete arm moment of inertia (kg.m^2)
% note: found using parallel axis theorem: J = J_cog + M*D^2
if ARM_LOAD == 0
    Jarm = J_main_arm;
else
    Jarm = J_main_arm + J_short_arm + m_short_arm * (D_short_arm(ARM_LOAD))^2;
end
%
% Arm equivalent viscous damping (N.m/(rad/s))
Barm = 0;
    
%% Setup ROTFLEX Position Source
%
if strcmp( ROTFLEX_OPTION, 'ROTFLEX')
    RtflxOp = 0;
    K_POT_ROT = -(345)*pi/180/24;
elseif strcmp( ROTFLEX_OPTION, 'ROTFLEX-E')
    RtflxOp = 1;
    K_POT_ROT = 0;
else
    error('Error: Set the rotflex option correctly.')
end
%  
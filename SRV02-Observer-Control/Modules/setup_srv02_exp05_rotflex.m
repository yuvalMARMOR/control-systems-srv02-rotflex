%% SETUP_SRV02_EXP05_ROTFLEX
%
% Sets the necessary parameters to run the SRV02 Experiment #5: Rotary
% Flexible Joint laboratory using the "s_srv02_rotflex" and "q_srv02_rotflex" 
% Simulink diagrams.
% 
% Copyright (C) 2010 Quanser Consulting Inc.
%
clear all;
%
%% SRV02 Configuration
% External Gear Configuration: set to 'HIGH' or 'LOW'
EXT_GEAR_CONFIG = 'HIGH';
% Encoder Type: set to 'E' or 'EHR'
ENCODER_TYPE = 'E';
% Is SRV02 equipped with Tachometer? (i.e. option T): set to 'YES' or 'NO'
TACH_OPTION = 'YES';
% Type of Load: set to 'NONE', 'DISC', or 'BAR'
LOAD_TYPE = 'ROTFLEX';
% Amplifier gain used: set to 1, 3, or 5
K_AMP = 1;
% Amplifier type: set to 'UPM_1503', 'UPM_2405' or 'Q3', or 'VoltPaq'
AMP_TYPE = 'VoltPaq';
% Digital-to-Analog Maximum Voltage (V)
VMAX_DAC = 10;
%
%% Rotary Flexible Joint Configuration
% Rotary flexible joint option: either 'ROTFLEX' or 'ROTFLEX-E'.
ROTFLEX_OPTION = 'ROTFLEX-E';
% Spring type used in ROTFLEX, default is 2: set to 1, 2 or 3 based on
% stiffness. The lower the stiffness, the lower the number would be.
SPRING = 2;
% Position of Spring on HUB (Base): set to 'A', 'B' or 'C'
HUB_POSITION = 'B';
% Position of Spring Anchor on Arm, set to 1, 2 or 3
ARM_POSITION = 2;
% Location of the Short Arm on main arm. Set to 0 if not connected, 1, 2 or 3 for each other position.
% If the two holes near the short arm's end are used, set this value to 1.
% If the short arm is attached to the main arm such that a hole near the
% end is sticking out, set this value to 2. Otherwise, set it to 3.
ARM_LOAD = 0;
% Safety watchdog on the SRV02 arm angle: ON = 1, OFF = 0 
THETA_LIM_ENABLE = 1;       % safety watchdog turned ON
% THETA_LIM_ENABLE = 0;      % safety watchdog turned OFF
% Safety Limits on the SRV02 arm angle (deg)
%global THETA_MAX THETA_MIN
THETA_MAX = 90;            % pendulum angle maximum safety position (deg)
THETA_MIN = - THETA_MAX;   % pendulum angle minimum safety position (deg)
%
%% Lab Configuration
% Type of controller: set it to 'AUTO', 'MANUAL'
% CONTROL_TYPE = 'AUTO';
CONTROL_TYPE = 'MANUAL';
%
%% Control specifications
% SRV02 Position Control specifications
% Settling time (s)
ts = 1.5;
% Settling time percentage (%)
c_ts = 0.04;
% Percentage overshoot (%)
PO = 5.0;
%    
%% System Parameters
% Sets model variables according to the user-defined SRV02 configuration
[ Rm, kt, km, Kg, eta_g, Beq, Jm, Jeq, eta_m, K_POT, K_TACH, K_ENC, VMAX_AMP, IMAX_AMP ] = config_srv02( EXT_GEAR_CONFIG, ENCODER_TYPE, TACH_OPTION, AMP_TYPE, LOAD_TYPE );
% Sets model variables according to the user-defined ROTFLEX configuration
[ Jarm, Barm, K_Stiff, RtflxOp, K_POT_ROT ] = config_rotflex( SPRING, HUB_POSITION, ARM_POSITION, ARM_LOAD, ROTFLEX_OPTION );
% Set Open-loop state-space model of rotary single inverted pendulum
SRV02_ROTFLEX_ABCD_eqns;
% Initial state for simulation
X0 = zeros(1,4);
%
%% Filter Parameters
% SRV02 High-pass filter parameters used to compute the load gear angular
% velocity.
% Cutoff frequency (rad/s)
wcf_1 = 2 * pi * 23.8732;
% Damping ratio
zetaf_1 = 0.9;
% ROTFLEX High-pass filter parameters used to compute arm's deflection angular
% velocity.
% Cutoff frequency (rad/s)
wcf_2 = 2 * pi * 23.8732;
% Damping ratio
zetaf_2 = 0.9;
%
%% Calculate Control Parameters
if strcmp ( CONTROL_TYPE , 'MANUAL' )    
    % ROTFLEX control gain
    K = zeros(1,4);
    %
elseif strcmp ( CONTROL_TYPE , 'AUTO' )
    % ROTFLEX control gain calculated using d_rotflex_pp function.
    K = d_rotflex_pp(A,B);
end
%
%% Display
disp( ' ' );
disp( 'SRV02+ROTFLEX Control Gain: ' )
disp( [ '   K(1) = ' num2str( K(1) ) ' V/rad' ] )
disp( [ '   K(2) = ' num2str( K(2) ) ' V/rad' ] )
disp( [ '   K(3) = ' num2str( K(3) ) ' V/(rad/s)' ] )

disp( [ '   K(4) = ' num2str( K(4) ) ' V/(rad/s)' ] )

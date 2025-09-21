% Matlab equation file: "SRV02_FLEXGAGE_ABCD_eqns.m"
% Open-Loop State-Space Matrices: A, B, C, and D
% for the Quanser Flexible Link experiment.

A( 1, 1 ) = 0;
A( 1, 2 ) = 0;
A( 1, 3 ) = 1;
A( 1, 4 ) = 0;
A( 2, 1 ) = 0;
A( 2, 2 ) = 0;
A( 2, 3 ) = 0;
A( 2, 4 ) = 1;
A( 3, 1 ) = 0;
A( 3, 2 ) = K_Stiff/Jeq;
A( 3, 3 ) = -(eta_g*Kg^2*eta_m*kt*km+Beq*Rm)/Rm/Jeq;
A( 3, 4 ) = 0;
A( 4, 1 ) = 0;
A( 4, 2 ) = -K_Stiff*(Jeq+Jarm)/Jeq/Jarm;
A( 4, 3 ) = (eta_g*Kg^2*eta_m*kt*km+Beq*Rm)/Rm/Jeq;
A( 4, 4 ) = 0;

B( 1, 1 ) = 0;
B( 2, 1 ) = 0;
B( 3, 1 ) = eta_g*Kg*eta_m*kt/Rm/Jeq;
B( 4, 1 ) = -eta_g*Kg*eta_m*kt/Rm/Jeq;

C( 1, 1 ) = 1;
C( 1, 2 ) = 0;
C( 1, 3 ) = 0;
C( 1, 4 ) = 0;
C( 2, 1 ) = 0;
C( 2, 2 ) = 1;
C( 2, 3 ) = 0;
C( 2, 4 ) = 0;

D( 1, 1 ) = 0;
D( 2, 1 ) = 0;

Scalar const c0 = c[0] - 1;
Scalar const c1 = 0.5*atan2(c[1], c[0]);
Scalar const c2 = c1*c[1]/pow(c0, 2);
Scalar const c3 = pow(c[1], 2);
Scalar const c4 = 1.0/(c3 + pow(c[0], 2));
Scalar const c5 = c4*c[1];
Scalar const c6 = 0.5*t[1];
Scalar const c7 = c5*c6;
Scalar const c8 = 0.5*t[0];
Scalar const c9 = 1.0/c0;
Scalar const c10 = c3*c4*c9;
Scalar const c11 = c1*c9;
Scalar const c12 = c4*c[0];
Scalar const c13 = c5*c8;
Scalar const c14 = c9*c[0];
Scalar const c15 = -c11*c[1];
result[0] = c10*c8 + c2*t[0] - c7;
result[1] = -c11*t[0] + c12*c6 - c13*c14;
result[2] = c15;
result[3] = c1;
result[4] = c10*c6 + c13 + c2*t[1];
result[5] = -c11*t[1] - c12*c8 - c14*c7;
result[6] = -c1;
result[7] = c15;
result[8] = -c5;
result[9] = c12;
result[10] = 0;
result[11] = 0;

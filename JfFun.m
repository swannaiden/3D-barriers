function Jf = JfFun(in1)
%JFFUN
%    JF = JFFUN(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    07-Sep-2021 01:24:03

x4 = in1(4,:);
x5 = in1(5,:);
x6 = in1(6,:);
x7 = in1(7,:);
x11 = in1(11,:);
x12 = in1(12,:);
x13 = in1(13,:);
t2 = x4./2.0;
t3 = x5./2.0;
t4 = x6./2.0;
t5 = x7./2.0;
t6 = x11./2.0;
t7 = x12./2.0;
t8 = x13./2.0;
t9 = -t3;
t10 = -t4;
t11 = -t5;
t12 = -t6;
t13 = -t7;
t14 = -t8;
Jf = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t6,t7,t8,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t12,0.0,t14,t7,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t13,t8,0.0,t12,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t14,t13,t6,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t9,t2,t5,t10,0.0,0.0,0.0,-5.0e+1,0.0,0.0,0.0,0.0,0.0,t10,t11,t2,t3,0.0,0.0,0.0,0.0,-5.0e+1,0.0,0.0,0.0,0.0,t11,t4,t9,t2,0.0,0.0,0.0,0.0,0.0,-5.0e+1],[13,13]);
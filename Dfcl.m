function Dfcl = Dfcl(in1,in2)
%DFCL
%    DFCL = DFCL(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 8.3.
%    07-Sep-2021 01:24:03

u1 = in2(1,:);
x4 = in1(4,:);
x5 = in1(5,:);
x6 = in1(6,:);
x7 = in1(7,:);
x11 = in1(11,:);
x12 = in1(12,:);
x13 = in1(13,:);
t2 = x4.^2;
t3 = x5.^2;
t4 = x6.^2;
t5 = x7.^2;
t6 = x4./2.0;
t7 = x5./2.0;
t8 = x6./2.0;
t9 = x7./2.0;
t10 = x11./2.0;
t11 = x12./2.0;
t12 = x13./2.0;
t13 = -t7;
t14 = -t8;
t15 = -t9;
t16 = -t10;
t17 = -t11;
t18 = -t12;
t19 = t2+t3+t4+t5;
t20 = 1.0./t19;
t21 = t20.^2;
t22 = t20.*x4.*2.0;
t23 = t20.*x5.*2.0;
t24 = t20.*x6.*2.0;
t25 = t20.*x7.*2.0;
t26 = t21.*x4.*x5.*x6.*4.0;
t27 = t21.*x4.*x5.*x7.*4.0;
t28 = t21.*x4.*x6.*x7.*4.0;
t29 = t21.*x5.*x6.*x7.*4.0;
Dfcl = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t10,t11,t12,-u1.*(-t24+t27+t2.*t21.*x6.*4.0),-u1.*(t23+t28-t2.*t21.*x5.*4.0),u1.*(t22-t21.*x4.^3.*2.0+t3.*t21.*x4.*2.0+t4.*t21.*x4.*2.0-t5.*t21.*x4.*2.0),0.0,0.0,0.0,0.0,0.0,0.0,t16,0.0,t18,t11,-u1.*(-t25+t26+t3.*t21.*x7.*4.0),-u1.*(t22+t29-t3.*t21.*x4.*4.0),-u1.*(t23-t21.*x5.^3.*2.0+t2.*t21.*x5.*2.0-t4.*t21.*x5.*2.0+t5.*t21.*x5.*2.0),0.0,0.0,0.0,0.0,0.0,0.0,t17,t12,0.0,t16,-u1.*(-t22+t29+t4.*t21.*x4.*4.0),u1.*(t25+t26-t4.*t21.*x7.*4.0),-u1.*(t24-t21.*x6.^3.*2.0+t2.*t21.*x6.*2.0-t3.*t21.*x6.*2.0+t5.*t21.*x6.*2.0),0.0,0.0,0.0,0.0,0.0,0.0,t18,t17,t10,0.0,-u1.*(-t23+t28+t5.*t21.*x5.*4.0),u1.*(t24+t27-t5.*t21.*x6.*4.0),u1.*(t25-t21.*x7.^3.*2.0-t2.*t21.*x7.*2.0+t3.*t21.*x7.*2.0+t4.*t21.*x7.*2.0),0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t13,t6,t9,t14,0.0,0.0,0.0,-5.0e+1,0.0,0.0,0.0,0.0,0.0,t14,t15,t6,t7,0.0,0.0,0.0,0.0,-5.0e+1,0.0,0.0,0.0,0.0,t15,t8,t13,t6,0.0,0.0,0.0,0.0,0.0,-5.0e+1],[13,13]);
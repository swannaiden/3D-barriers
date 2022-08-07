quat = eul2quat([.3,1,.2])
quat_yaw = quatinv(eul2quat([.3,0,0]));
quat_yaw2 = eul2quat([-.3,0,0]);
quat_opti = eul2quat([.4,0,0]);
test = quatmultiply(quat_yaw2,quat);
test = quatmultiply(quat_opti,test);
quat2eul(test)


%%
 

pwm = 0:.1:1;
pwm = pwm.*(1811-172)+172;
thrust = (2.7508051e-05.*(pwm).^2 -0.0047313846.*pwm)./9.81;


plot(pwm, thrust);

%%

quatCur = eul2quat([.3,-.1,pi/2]);
eulOffset = [.3,-.1,pi];
quatTarget = eul2quat([eulOffset(1),eulOffset(2),pi]);

quatDif = quatmultiply(quatinv(quatCur), quatTarget);
quat2eul(quatDif)
quat_end = quatmultiply(quatCur, quatDif);
euler = quat2eul(quat_end)

% quatInv = eul2quat([.1,0,0]);
% %quatInv2 = eul2quat([0,-.1,0]);
% q1 = quatmultiply(quat,quatInv);
% %q2 = quatmultiply(q1, quatInv2);

quat2eul(q1);


%%
rot = quat2rotm(quat);
rot = roty(.1/(2*pi)*360)*rot*roty(.1/(2*pi)*360)';
rotm2eul(rot);


%%

quatCur = eul2quat([.1,-.1,pi/2]);

quatOffset = eul2quat([0, 0, -pi/2]);

quat_no_yaw = quatmultiply(quatCur, quatOffset)

quatYaw = eul2quat([0,0,pi]);
quat2eul(quatmultiply(quat_no_yaw, quatYaw))


function u = backup_controller_sim(x,hover_height,yaw0)
    quat = x(4:7)';
    tmp = quaternion_to_yaw(quat);
%     tmp = [1,2];
    R = [cos(tmp(1)), -sin(tmp(1)); sin(tmp(1)), cos(tmp(1))];
    tmp2 = transpose(R)*[x(8);x(9)];
    vbx = tmp2(1); vby = tmp2(2);
    vwdx = 0;
%     if (subs(x(1)) > 9)
%         vwdx = max(-1*(x(1)-9),-1);
%     end
    vwdy = 0;
    tmp2 = transpose(R)*[vwdx;vwdy];
    vdx = tmp2(1); vdy = tmp2(2);
    
    vxError = vdx-vbx;
    vyError = vdy-vby;
%     rolld = saturate(-vyError,-pi/3,pi/3);
%     pitchd = saturate(vxError,-pi/3,pi/3);
    rolld = -vyError;
    pitchd = -vxError;
    yawd = yaw0;
    
    Rd = eul_to_rotm([yawd,pitchd,rolld]);
    
%     R = quat_to_rotm(quat);
R = eye(3);
    
    eR = vee(1/2*(transpose(Rd)*R-transpose(R)*Rd));
    K = 10;
    Od = -K*eR;
    
    hoverThrust = .32;
    q = x(4:7);
    tmp = my_quatrotate(q', [0 0 1])';
        tmp(3) = max(tmp(3),.5);
    Kp = .1;
    Kd = .05;
%     if (hover_height < 1.5)
%         hover_height = 1.5;
%     end
    pterm = Kp*(hover_height-x(3));
    dterm = Kd*(0-x(10));
%     pterm = saturate(pterm,-.3,.3);
%     dterm = saturate(dterm,-.3,.3);
    u(1,1) = hoverThrust/tmp(3) + pterm + dterm;
%     u(1,1) = hoverThrust + pterm + dterm;
    u(2:4,1) = Od;
end

function out = vee(in)
    out(1,1) = in(3,2);
    out(2,1) = in(1,3);
    out(3,1) = in(2,1);
end

function [r] = my_quatrotate(q, w)
  % inverse of standard Matlab quatrotate
  r = quatmultiply(quatmultiply(q, [0 w]), quatinv(q));
  r = r(2:4);
end

function out = saturate(in,min_val,max_val)
    out = max(min(in,max_val),min_val);
end

function out = euler_to_quaternion(in)
    roll = in(1);
    pitch = in(2);
    yaw = in(3);
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2);
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2);
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2);
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2);
    out = [qw,qx,qy,qz];
end

function out = quaternion_to_yaw(in)
    w = in(1);
    x = in(2);
    y = in(3);
    z = in(4);
   
    t3 = +2.0 * (w * z + x * y);
    t4 = +1.0 - 2.0 * (y * y + z * z);
    yaw = atan2(t3, t4);
    out = yaw;
end

function out = eul_to_rotm(in)
theta(1) = in(3);
theta(2) = in(2);
theta(3) = in(1);
R_x = [1,       0,              0;
               0,       cos(theta(1)),   -sin(theta(1));

               0,       sin(theta(1)),   cos(theta(1))];

R_y = [ cos(theta(2)),    0,      sin(theta(2));

               0,               1,      0;

               -sin(theta(2)),   0,      cos(theta(2))];
           
R_z =[ cos(theta(3)),    -sin(theta(3)),      0;

               sin(theta(3)),    cos(theta(3)),       0;

               0,               0,                  1];
out = R_z*R_y*R_x;

end

% function out = quat_to_rotm(in)
%     q0 = in(1);
%     q1 = in(2);
%     q2 = in(3);
%     q3 = in(4);
%      
%     # First row of the rotation matrix
%     r00 = 2 * (q0 * q0 + q1 * q1) - 1
%     r01 = 2 * (q1 * q2 - q0 * q3)
%     r02 = 2 * (q1 * q3 + q0 * q2)
%      
%     # Second row of the rotation matrix
%     r10 = 2 * (q1 * q2 + q0 * q3)
%     r11 = 2 * (q0 * q0 + q2 * q2) - 1
%     r12 = 2 * (q2 * q3 - q0 * q1)
%      
%     # Third row of the rotation matrix
%     r20 = 2 * (q1 * q3 - q0 * q2)
%     r21 = 2 * (q2 * q3 + q0 * q1)
%     r22 = 2 * (q0 * q0 + q3 * q3) - 1
%      
%     # 3x3 rotation matrix
%     rot_matrix = np.array([[r00, r01, r02],
%                            [r10, r11, r12],
%                            [r20, r21, r22]])
% end
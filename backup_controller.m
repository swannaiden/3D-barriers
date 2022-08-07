function u = backup_controller(x,hover_height,yaw0)
    x_pos = 1;
    y_pos = 50;
    x_barrier = true;
    quat = x(4:7)';
    tmp = quat2eul(quat);
    R = [cos(tmp(1)), -sin(tmp(1)); sin(tmp(1)), cos(tmp(1))];
    tmp2 = transpose(R)*[x(8);x(9)];
    vbx = tmp2(1); vby = tmp2(2);
    vwdx = 0;
    vwdy = 0;
    if (x_barrier)
    if (x(1) > (x_pos-1))
        vwdx = saturate(-1*(x(1)-(x_pos-1)),-1,1);
    end
    if (x(2) > (y_pos-1))
        vwdy = saturate(-1*(x(2)-(y_pos-1)),-1,1);
    end
    end
    tmp2 = transpose(R)*[vwdx;vwdy];
    vdx = tmp2(1); vdy = tmp2(2);
    
    K = .15;
    vxError = vdx-vbx;
    vyError = vdy-vby;
%     rolld = saturate(-K*vyError,-pi/3,pi/3);
%     pitchd = saturate(K*vxError,-pi/3,pi/3);
    rolld = saturate(-K*vyError,-pi/2.1,pi/2.1);
    pitchd = saturate(K*vxError,-pi/2.1,pi/2.1);
%     rolld = saturate(-K*vyError,-pi/12,pi/12);
%     pitchd = saturate(K*vxError,-pi/12,pi/12);
    yawd = yaw0;
    
    if (abs(tmp(2)) > pi/2) || (abs(tmp(3)) > pi/2)
    	Rd = eye(3);
    else
        Rd = eul2rotm([yawd,pitchd,rolld]);
    end
%     Rd = eye(3);
    R = quat2rotm(quat);
    
    eR = vee(1/2*(transpose(Rd)*R-transpose(R)*Rd));
    K = 10;
    Od = -K*eR;
    
    hoverThrust = .32;
    q = x(4:7);
    tmp = my_quatrotate(q', [0 0 1])';
    if (tmp(3) < .5 && tmp(3) >= 0)
        tmp(3) = .5;
    end
    if (tmp(3) < 0)
        tmp(3) = 3;
    end
    u = zeros(4,1);
    Kp = .1;
    Kd = .05;
    if ~x_barrier
    if (hover_height < 1.5)
        hover_height = 1.5;
    end
    end
    pterm = Kp*(hover_height-x(3));
    dterm = Kd*(0-x(10));
    pterm = saturate(pterm,-.3,.3);
    dterm = saturate(dterm,-.3,.3);
    u(1,1) = saturate(hoverThrust/tmp(3) + pterm + dterm,0,1);
%     u(1,1) = hoverThrust + pterm + dterm;
    u(2:4,1) = Od;
end

function out = vee(in)
    out = zeros(3,1);
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
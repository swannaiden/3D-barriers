function u = controller(x,x_des,y_des,v_max)
    vwdx = saturate(x_des,-v_max,v_max);
    vwdy = saturate(y_des,-v_max,v_max);
    vwx = x(8);
    vwy = x(9);
    tmp = quat2eul(x(4:7)');
    R = [cos(tmp(1)), -sin(tmp(1)); sin(tmp(1)), cos(tmp(1))];
    tmp2 = transpose(R)*[x(8);x(9)];
    vbx = tmp2(1); vby = tmp2(2);
    tmp2 = transpose(R)*[vwdx;vwdy];
    vdx = tmp2(1); vdy = tmp2(2);
    vxError = vdx-vbx;
    vyError = vdy-vby;
%     rolld = saturate(-vyError,-pi/3,pi/3);
%     pitchd = saturate(vxError,-pi/3,pi/3);
    rolld = saturate(-vyError,-pi/12,pi/12);
    pitchd = saturate(vxError,-pi/12,pi/12);
    yawd = tmp(1);
    
    Rd = eul2rotm([yawd,pitchd,rolld]);
    R = quat2rotm(x(4:7)');
    
    eR = vee(1/2*(transpose(Rd)*R-transpose(R)*Rd));
    K = 10;
    Od = -K*eR;
    
    hoverThrust = .3;
    Ki = 1;
    Kp = 1;
    Kd = 1;
    q = x(4:7);
    tmp = my_quatrotate(q', [0 0 1])';
    if (tmp(3) < .5)
        tmp(3) = .5;
    end
    pterm = saturate(Kp*(0-x(3)),-.05,.05);
    dterm = saturate(Kd*(0-x(10)),-.05,.05);
    u(1,1) = hoverThrust/tmp(3) + pterm+dterm;
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
function [u,h,lambda] = barrier(x,u_des)
    x_barrier = true;
    x_pos = 1;
    y_pos = 50;
    
    T = 3;
    dt = T/100;
    h = zeros(T/dt+1,1);
    h(1) = SafetySet(x);
    hover_height = x(3);
    tmp = quat2eul(x(4:7)');
    yaw0 = tmp(1);
    x0 = x;
    ub = backup_controller(x,hover_height,yaw0);
    x_plot = zeros(13,T/dt+1);
    x_plot(:,1) = x0;
    for i = 1:T/dt
        u_tmp = backup_controller(x,hover_height,yaw0);
        xDot = cont_dynamics(0,x,u_tmp);
        x = x + xDot*dt;
        h(i+1) = SafetySet(x);
        x_plot(:,i+1) = x;
    end
    min_h = min(h); 
    min_h = min(min_h,BackupSet(x));
    if min_h < 0
        min_h = 0;
    end
    if (x_barrier)
        if abs(x(1)-x_pos) < abs(x(2)-y_pos)
            v_pos = saturate(abs(x0(8)),1,10); %x barrier
        else
            v_pos = saturate(abs(x0(9)),1,10); %y barrier
        end
    else
    v_pos = saturate(abs(x0(10)),2,10); %z barrier
    end
%     lambda = 1-exp(-3*(min_h^2/(100*v_pos))); %x barrier
    if (x_barrier)
    lambda = 1-exp(-3*(min_h/(2*v_pos))); %x barrier
    else
%         lambda = 1-exp(-3*(min_h/(2*v_pos))); %x barrier
        lambda = 1-exp(-3*(min_h^2/(100*v_pos))); %x barrier
    end
    u = lambda*u_des + (1-lambda)*ub;
    if min(h) < 0
        u = ub;
        lambda = 0;
    end
end

function h = SafetySet(x)
x_pos = 1;
y_pos = 1;
x_barrier = true;
circle_barrier = false; r = .5;
%     h = -(x(1))^2-(x(2))^2+1^2;
    h = zeros(1,1);
%     h = -(x(1))^2+10^2;
%     if x(1) < 0
%         h = (x(1))^2+10^2;
%     end
if (x_barrier)
    h = min(x_pos-x(1),y_pos-x(2));
else
    h = x(3)-.5;
end
if (circle_barrier)
    h = sqrt((x(1)-x_pos)^2+(x(2)-y_pos)^2)-r;
end
end

function hB = BackupSet(x)
    hB = 40*(-sqrt(x(8)^2+x(9)^2+x(10)^2)+1);
end

function out = saturate(in,min_val,max_val)
    out = max(min(in,max_val),min_val);
end
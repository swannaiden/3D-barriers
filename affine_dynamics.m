function [f,g] = affine_dynamics(x)
% f = zeros(13,1);
% g = zeros(13,4);
%states: x,y,z,qw,qx,qy,qz,vx,vy,vx,ox,oy,oz
vx = x(8); vy = x(9); vz = x(10);
f(1:3) = [vx;vy;vz];
ox = x(11); oy = x(12); oz = x(13);
% ox = u(2); oy = u(3); oz = u(4);
w = [0;ox;oy;oz];
q = [x(4);x(5);x(6);x(7)];

qDot = .5*quat_mult(q,w);
f(4:7) = qDot;
M = 1;
grav = 9.81;
f(8:10) = - [0; 0; grav];
g(8:10,1) = (1/M) * my_quatrotate(q', [0 0 1])';
f(11:13) = 50*-x(11:13);
g(11,2) = 50;
g(12,3) = 50;
g(13,4) = 50;
end

function out = quat_mult(q,p)
    out = [p(1)*q(1)-dot(p(2:4),q(2:4));q(1)*p(2:4)+p(1)*q(2:4)+cross(q(2:4),p(2:4))];
end

function [r] = my_quatrotate(q, w)
  % inverse of standard Matlab quatrotate
  r = quatmultiply(quatmultiply(q, [0 w]), quatinv(q));
  r = r(2:4);
end
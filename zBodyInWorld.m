function out = zBodyInWorld(in)
    tmp = my_quatrotate(in', [0 0 1])';
    out = tmp(3);
end

function [r] = my_quatrotate(q, w)
  % inverse of standard Matlab quatrotate
  r = quatmultiply(quatmultiply(q, [0 w]), quatinv(q));
  r = r(2:4);
end

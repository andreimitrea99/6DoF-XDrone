function Omega = omegaMat(omega)
%OMEGAMAT  Reorders elements of omega for quarterion update
%
%   omega : [3x1] vector containing rotation about body frame
%   Omega : [4x4] reorder
p=omega(1); q=omega(2); r=omega(3);
Omega = [ 0 -p -q -r;
          p  0  r -q;
          q -r  0  p;
          r  q -p  0 ];
end

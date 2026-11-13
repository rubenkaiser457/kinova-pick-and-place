function A = dhTransform(a, alpha, d, theta)
%DHTRANSFORM Homogeneous transform of one link (standard DH convention).
%   A = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)

    ct = cos(theta);  st = sin(theta);
    ca = cos(alpha);  sa = sin(alpha);

    A = [ ct, -st*ca,  st*sa, a*ct;
          st,  ct*ca, -ct*sa, a*st;
           0,     sa,     ca,    d;
           0,      0,      0,    1 ];
end

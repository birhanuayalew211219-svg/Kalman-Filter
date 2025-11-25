function Om = omega_matrix(w)
% 4x4 Omega matrix for quaternion kinematics
    Om = [-skew(w)  w;
          -w'       0];
end

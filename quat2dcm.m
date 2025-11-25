function R = quat2dcm(q)
% Quaternion (q1,q2,q3, q4) → DCM, scalar last
    q  = q / norm(q);
    q1 = q(1); 
    q2 = q(2); 
    q3 = q(3); 
    q4 = q(4);

    R = [2*(q4^2 + q1^2) - 1,   2*(q1*q2 + q4*q3),     2*(q3*q1 - q4*q2);
         2*(q1*q2 - q4*q3),     2*(q4^2 + q2^2) - 1,   2*(q2*q3 + q4*q1);
         2*(q3*q1 + q4*q2),     2*(q2*q3 - q4*q1),     2*(q4^2 + q3^2) - 1];
end

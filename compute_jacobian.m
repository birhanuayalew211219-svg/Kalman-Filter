function F = compute_jacobian(x)
% Linearized Jacobian of the 13-state system

    mu    = 398600;
    r     = x(1:3);
    v     = x(4:6);
    q     = x(7:10);
    omega = x(11:13);

    F = zeros(13,13);

    % Position wrt velocity
    F(1:3,4:6) = eye(3);

    % Gravity wrt position
    rnorm = norm(r);
    I3    = eye(3);
    F(4:6,1:3) = mu * (3*(r*r')/rnorm^5 - I3/rnorm^3);

    % Quaternion wrt quaternion
    Omega = [0      -omega';
             omega  -skew(omega)];
    F(7:10,7:10) = 0.5 * Omega;

    % Quaternion wrt angular rate (finite difference)
    eps = 1e-6;
    for i = 1:3
        domega      = zeros(3,1); 
        domega(i)   = eps;
        dq1         = 0.5 * omega_matrix(omega + domega) * q;
        dq2         = 0.5 * omega_matrix(omega - domega) * q;
        F(7:10,10+i) = (dq1 - dq2) / (2*eps);
    end
end

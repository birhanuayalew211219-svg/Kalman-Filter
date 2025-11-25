function dx = attitude_dynamics(x)
% Continuous-time orbit + attitude dynamics
    mu    = 398600;
    r     = x(1:3);
    v     = x(4:6);
    q     = x(7:10);
    omega = x(11:13);

    dr = v;
    dv = -mu * r / norm(r)^3;

    Omega = [0      -omega';
             omega  -skew(omega)];

    dq     = 0.5 * Omega * q;
    domega = zeros(3,1);

    dx = [dr; dv; dq; domega];
end

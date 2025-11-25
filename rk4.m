function x_next = rk4(f, x, h)
% 4th-order Runge–Kutta integrator
    k1 = f(x);
    k2 = f(x + 0.5*h*k1);
    k3 = f(x + 0.5*h*k2);
    k4 = f(x + h*k3);
    x_next = x + h/6*(k1 + 2*k2 + 2*k3 + k4);
end

function [dState_dt] = Motion_equations(t,state,g,m,Cd,A)
    vel = state(4:6);
    Fg = [0; -g*m; 0];
    vel_mag = sqrt((vel(1)^2) + (vel(2)^2) + (vel(3)^2));
    k = -0.5 * rho * A * Cd * vel_mag;
    Drag = [k * vel(1); k * vel(2); k * vel(3)];
    F_net = Fg + Drag;
    acc = F_net/m;
    dState_dt = [vel;acc];
end


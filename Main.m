clc
clear

% add row function

function Mat2 = append_row(Mat1,addition)
Mat2 = [Mat1;addition];
end

% Lug geometry
A = 10;
Cd = 0.2;
L = 0.3;
mass = 1;

% constants
dt = 0.1;
t_step = 0;

% Initial pos
pos = [];
X_release = 0;
Y_release = 0;
Z_release = 0;
pos = append_row(pos,[X_release,Y_release,Z_release]);

% Initial vel
vel = [];
U_release = 0;
V_release = 0;
W_release = 0;
vel = append_row(vel, [U_release,V_release,W_release]);

% Initial acc
acc = [];

% get drag
function drag = get_drag(Cd, A, vel, rho)
vel_mag = sqrt((vel(1)^2) + (vel(2)^2) + (vel(3)^2));
k = 0.5 * rho * A * Cd * vel_mag;
dragX = k * vel(1);
dragY = k * vel(2);
dragZ = k * vel(3);
drag = [dragX,dragY,dragZ];
end

function acc = get_acc(drag, g, m)
acc_x = (drag(1)/m);
acc_y = (drag(2)/m) + g;
acc_z = (drag(3)/m);
acc = [acc_x,acc_y,acc_z];
end

function [pos_f,vel_f,time_f] = update_pos_vel(pos_I, vel_I, acc_I, time_I, t_step)
[X_I,Y_I,Z_I] = pos_I(:);
[U_I,V_I,W_I] = vel_I(:);
[Uprime_I,Vprime_I,Wprime_I] = acc_I(:);

X_f = X_I + U_I * dt;
Y_f = Y_I + V_I * dt;
Z_f = Z_I + W_I * dt;

U_f = U_I + Uprime_I * dt;
V_f = V_I + Vprime_I * dt;
W_f = W_I + Wprime_I * dt;

t_step = t_step + 1;
time_f = time_I + dt;
end

% main sim
while pos(t_step,2) > 0

end 


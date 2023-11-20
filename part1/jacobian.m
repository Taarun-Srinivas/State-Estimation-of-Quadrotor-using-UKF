%Declare the states

P = sym('P', [1 3]);
q = sym('q', [1 3]);
linear_vel = sym('lv', [1 3]);
gyro_bias = sym('gb', [1 3]);
acc_bias = sym('ab', [1 3]);

%Declare sym variables for xd
Wm = sym('wm',[1 3]);
Am = sym('Am',[1 3]);

%Declare sym variables for noise
gyro_noise = sym('ng', [1 3]);
acc_noise = sym('na', [1 3]);
vel_noise = sym('nv', [1 3]);
gb_noise = sym('nbg',[1 3]);
ab_noise = sym('nba', [1 3]);

states = [transpose(P);
          transpose(q);
          transpose(linear_vel);
          transpose(gyro_bias);
          transpose(acc_bias);];

noise = [transpose(vel_noise);
         transpose(gyro_noise);
         transpose(acc_noise);
         transpose(gb_noise);
         transpose(ab_noise)]


R = [cos(q(2))*cos(q(3)) cos(q(3))*sin(q(1))*sin(q(2))-cos(q(1))*sin(q(3)) sin(q(1))*sin(q(3))+cos(q(1))*cos(q(3))*sin(q(2));
     cos(q(2))*sin(q(3)) cos(q(1))*cos(q(3))+sin(q(1))*sin(q(2))*sin(q(3)) cos(q(1))*sin(q(2))*sin(q(3))-cos(q(3))*sin(q(1));
     -sin(q(2)) cos(q(2))*sin(q(1)) cos(q(1))*cos(q(2))];

G = [0 -sin(q(3)) cos(q(2))*cos(q(3));
     0  cos(q(3)) cos(q(2))*sin(q(3));
     1 0 -sin(q(2))];

xd = [transpose(linear_vel) - transpose(vel_noise);
      R*inv(G)*(transpose(Wm) - transpose(gyro_bias) - transpose(gyro_noise));
      -9.81 + R*(transpose(Am) - transpose(acc_bias) - transpose(acc_noise));
      transpose(gb_noise);
      transpose(ab_noise)]

At = simplify(jacobian(xd,states));

Ut = simplify(jacobian(xd, noise));

size(At)
size(Ut)

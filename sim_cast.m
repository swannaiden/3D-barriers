clear all;clc;
speed_des = 3/2.23694;
x0 = [0,0,1,1,0,0,0,0,0,0,0,0,0]';
% x0 = [0,0,20,1,0,0,0,0,0,0,0,0,0]';
tmp = eul2quat([0,0,0]);
x0(4:7) = tmp;
vx0 = 0;
% vx0 = 0;
vy0 = 0;
vz0 = 0;
x0(8:10) = [vx0;vy0;vz0];
u = [0,0,0,0]';
u_vec{1} = u;
dt =.01;
T = 6;
[u,h,l] = barrier(x0,[.3,0,0,0])

x = x0;
x_vec{1} = x;
T_vec(1) = 0;
% delay_timesteps = 3;
% u_delay = zeros(4,delay_timesteps);
% u_delay(1,:) = .3;

for i = 1:T/dt
    des_u = controller(x,1,0,speed_des);
%     des_u = z_controller(x,100,speed_des);
    [u,h(:,i),lambda(i)] = barrier(x,des_u);
%     u = des_u;
    u_vec{i+1} = u;
%     u_delay(:,delay_timesteps) = u;
%     u = u_delay(:,1);
%     u_delay(:,1:delay_timesteps-1) = u_delay(:,2:delay_timesteps);
    xDot = cont_dynamics(0,x,u);
    x = x + xDot*dt;
    x_vec{i+1} = x;
    T_vec(i+1) = i*dt;
end
quat2eul(x(4:7)');


for i = 1:length(x_vec)
    posx(i) = x_vec{i}(1);
    posy(i) = x_vec{i}(2);
    posz(i) = x_vec{i}(3);
    velx(i) = x_vec{i}(8);
    vely(i) = x_vec{i}(9);
    velz(i) = x_vec{i}(10);
    wx(i) = x_vec{i}(11);
    wy(i) = x_vec{i}(12);
    wz(i) = x_vec{i}(13);
    ux(i) = u_vec{i}(2);
    uy(i) = u_vec{i}(3);
    uz(i) = u_vec{i}(4);
    tmp = quat2eul(x_vec{i}(4:7)');
    yaw(i) = tmp(1);
    pitch(i) = tmp(2);
    roll(i) = tmp(3);
end

figure(1)
subplot(2,2,1)
plot(T_vec,posx,'LineWidth',2)
hold on
plot(T_vec,1*ones(length(posx)),'linewidth',2,'color','black')
hold off
xlabel('t (s)','Interpreter','latex','FontSize',18)
ylabel('x (m)','Interpreter','latex','FontSize',18)
subplot(2,2,3)
plot(T_vec(1:end-1),lambda,'LineWidth',2)
xlabel('t (s)','Interpreter','latex','FontSize',18)
ylabel('$\lambda(h(x))$','Interpreter','latex','FontSize',18)
subplot(2,2,2)
plot(T_vec,3.6*abs(velx),'LineWidth',2)
xlabel('t (s)','Interpreter','latex','FontSize',18)
ylabel('v (km/h)','Interpreter','latex','FontSize',18)
subplot(2,2,4)
% plot(T_vec,wy,'LineWidth',2)
plot(posx,posy,'LineWidth',2)
xlabel('t (s)','Interpreter','latex','FontSize',18)
ylabel('$\omega_y$ (rad/s)','Interpreter','latex','FontSize',18)
clear all;clc;
speed_des = 65/2.23694;
%x0 = [-70,0,1,1,0,0,0,0,0,0,0,0,0]';
x0 = [-20,0,0,1,0,0,0,0,0,0,0,0,0]';
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
T = 15;
[u,h,l] = barrier(x0,[.3,0,0,0]);

x = x0;
x_vec{1} = x;
T_vec(1) = 0;
% delay_timesteps = 3;
% u_delay = zeros(4,delay_timesteps);
% u_delay(1,:) = .3;

for i = 1:T/dt
    des_u = controller(x,20,0,speed_des);
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

%%
figure(1)
subplot(2,2,1)
plot(T_vec,posx,'LineWidth',2)
hold on
plot(T_vec,1*ones(length(posx)),'linewidth',2,'color','black')
hold off
xlabel('t (s)','Interpreter','latex','FontSize',18)
ylabel('x (m)','Interpreter','latex','FontSize',18)
ylim([-70,12])
subplot(2,2,3)
plot(T_vec(1:end-1),lambda,'LineWidth',2)
xlabel('t (s)','Interpreter','latex','FontSize',18)
ylabel('$\lambda(h(x))$','Interpreter','latex','FontSize',18)
subplot(2,2,2)
plot(T_vec,3.6*abs(velx),'LineWidth',2)
ylim([0,110])
xlabel('t (s)','Interpreter','latex','FontSize',18)
ylabel('v (km/h)','Interpreter','latex','FontSize',18)
subplot(2,2,4)
plot(T_vec,wy,'LineWidth',2)
xlabel('t (s)','Interpreter','latex','FontSize',18)
ylabel('$\omega_y$ (rad/s)','Interpreter','latex','FontSize',18)

z = zeros(size(posx));
figure(2)
subplot(2,1,1)
scatter(posx, posy, [], T_vec)
c = colorbar;
c.Label.String = 'Time (s)';
hold on
plot(linspace(-25,1,length(posx)),1*ones(length(posx)),'linewidth',4,'color','red')
hold on
plot(1*ones(length(posx)), linspace(-10,1,length(posx)), 'linewidth',4,'color','red')
xlabel('x (m)','Interpreter','latex','FontSize',18)
ylabel('y (m)','Interpreter','latex','FontSize',18)
axis([-25, 2, -5 2])
title('Old Barrier')

%%
T_plot = linspace(min(T_vec),max(T_vec),7);
posx_plot = interp1(T_vec,posx,T_plot)
pitch_plot = interp1(T_vec,pitch,T_plot)
figure(2)
plot(posx(1:end-1),lambda)
hold on
[test1, test2,alpha] = imread('drone.png');
for i = 2:length(T_plot)-1
    l = (max(posx_plot)-min(posx_plot))/length(T_plot);
xim = [posx_plot(i-1)-l/2,posx_plot(i-1)+l/2];
yim = [0,1]/length(T_plot)+1;

angle = pitch_plot(i)*180/pi
image(xim,yim,imrotate(test1,180+angle),'AlphaData',imrotate(alpha,180+angle)*5)
colormap(test2)
end
hold off
% figure(2)
% subplot(5,1,1)
% plot(T_vec,velx)
% subplot(5,1,2)
% plot(T_vec,vely)
% subplot(5,1,3)
% plot(T_vec,velz)
% subplot(5,1,4)
% plot(T_vec(2:end),lambda)
% subplot(5,1,5)
% plot(T_vec(2:end),min(min(h),10))

% figure(3)
% plot(T_vec,uy)
% hold on
% plot(T_vec,wy)
%%
x0 = [0,0,10,1,0,0,0,25,0,0,0,0,0]';
[u,h,l] = barrier(x0,[.3,0,0,0])
min(h)
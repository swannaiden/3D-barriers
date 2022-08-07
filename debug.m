for i = 1:length(x_plot)
    tmp = quat2eul(x_plot(4:7,i)');
    roll(i) = tmp(3);
    rolldot(i) = x_plot(11,i);
end
subplot(2,1,1)
plot(roll)
subplot(2,1,2)
plot(rolldot)

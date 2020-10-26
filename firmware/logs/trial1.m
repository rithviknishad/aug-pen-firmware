[d,s,r] = xlsread('steady_accel_x_60s.csv');
t = d(:,1)/100;
v = d(:,2);
figure(1)
plot(t, v);
grid
xlabel('Time (s)')
ylabel('Acceleration (cm/s^2)')
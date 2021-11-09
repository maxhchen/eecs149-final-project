
data = testDataDecoder("data/q531_pi_control.txt");

f = figure('Renderer', 'painters', 'Position', [10 10 900 600]);
figure(f);

subplot(2, 1, 1);

plot(data.time, data.yaw, 'LineWidth', 2, 'DisplayName', 'Actual yaw');
hold on;
plot(data.time, data.control_yaw, 'LineWidth', 2, 'DisplayName', 'Control input');
title("Control input and reference signal versus actual yaw for PI controller");
ylabel("Yaw (deg)");
xlim([0, 70]);
grid;
legend;
hold off;

subplot(2, 1, 2);
plot(data.time, data.yaw, 'LineWidth', 2, 'DisplayName', 'Actual yaw');
hold on;
% 40 / pi * arcsin(sin(2 * pi * data.time/180))
plot(data.time, 360 / pi * asin(sin(2 * pi * data.time/20)), 'LineWidth', 2, 'DisplayName', 'Reference signal');

xlabel("Time (s)");
ylabel("Yaw (deg)");
xlim([0, 70]);
grid;
legend;
hold off;
saveas(f, "../figures/q531.png");


data = testDataDecoder("data/q532_dji_controller.txt");

f = figure('Renderer', 'painters', 'Position', [10 10 900 600]);
figure(f);

subplot(2, 1, 1);

plot(data.time, data.yaw, 'LineWidth', 2, 'DisplayName', 'Actual yaw');
hold on;
plot(data.time, data.control_yaw, 'LineWidth', 2, 'DisplayName', 'Control input');
title("Control input and reference signal versus actual yaw for DJI controller");
ylabel("Yaw (deg)");
xlim([0, 70]);
grid;
legend;
hold off;

subplot(2, 1, 2);
plot(data.time, data.yaw, 'LineWidth', 2, 'DisplayName', 'Actual yaw');
hold on;
% 40 / pi * arcsin(sin(2 * pi * data.time/180))
plot(data.time, 360 / pi * asin(sin(2 * pi * data.time/20)), 'LineWidth', 2, 'DisplayName', 'Reference signal');

xlabel("Time (s)");
ylabel("Yaw (deg)");
xlim([0, 70]);
grid;
legend;
hold off;
saveas(f, "../figures/q532.png");
return;






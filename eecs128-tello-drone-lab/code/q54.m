
% q54_test.txt is statedata.txt for question 5.4.
% That's where it should be saved in when ready.
data = testDataDecoder("data/q54_test.txt");

f = figure('Renderer', 'painters', 'Position', [10 10 900 600]);
figure(f);

subplot(3, 1, 1);

plot(data.time, data.yaw, 'LineWidth', 2, 'DisplayName', 'Actual yaw');
hold on;
plot(data.time, data.control_yaw, 'LineWidth', 2, 'DisplayName', 'Yaw control input');
title("Yaw, Updown Control input and reference signal versus actual yaw for disturbed controller");
ylabel("Yaw (deg)");
xlim([0, 70]);
grid;
legend;
hold off;

subplot(3, 1, 2);

plot(data.time, data.yaw, 'LineWidth', 2, 'DisplayName', 'Actual yaw');
hold on;
plot(data.time, data.control_upDown, 'LineWidth', 2, 'DisplayName', 'Up/down control input');
title("Yaw, Updown Control input and reference signal versus actual yaw for disturbed controller");
ylabel("Yaw (deg)");
xlim([0, 70]);
grid;
legend;
hold off;

subplot(3, 1, 3);
plot(data.time, data.yaw, 'LineWidth', 2, 'DisplayName', 'Actual yaw');
hold on;

plot(data.time, 360 / pi * asin(sin(2 * pi * data.time/20)), 'LineWidth', 2, 'DisplayName', 'Reference signal');

xlabel("Time (s)");
ylabel("Yaw (deg)");
xlim([0, 70]);
grid;
legend;
hold off;

saveas(f, "../figures/q54.png");

return;






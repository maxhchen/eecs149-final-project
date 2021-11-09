prelab_data = testDataDecoder("data/prelab5bTest.txt");

f = figure('Renderer', 'painters', 'Position', [10 10 900 600]);
figure(f);
plot(prelab_data.time, prelab_data.height, 'LineWidth', 2);
hold on;
title("Altitude plot of prelab flight");
xlabel("Time (s)");
ylabel("Height (cm)");
xlim([0 70]);
ylim([0 80]);
saveas(f, "../figures/q43_alt.png");
hold off;


f = figure('Renderer', 'painters', 'Position', [10 10 900 600]);
figure(f);
plot(prelab_data.time, prelab_data.velocity_x, 'LineWidth', 2, 'DisplayName', 'x');
hold on;
plot(prelab_data.time, prelab_data.velocity_y, 'LineWidth', 2, 'DisplayName', 'y');
plot(prelab_data.time, prelab_data.velocity_z, 'LineWidth', 2, 'DisplayName', 'z');
legend;
title("Velocity plot of prelab flight");
xlabel("Time (s)");
ylabel("Velocity (cm/s)");
xlim([0 70]);
%ylim([0 80]);
saveas(f, "../figures/q43_vel.png");
hold off;

f = figure('Renderer', 'painters', 'Position', [10 10 900 600]);
figure(f);
plot(prelab_data.time, prelab_data.yaw, 'LineWidth', 2, 'DisplayName', 'yaw');
hold on;
plot(prelab_data.time, prelab_data.pitch, 'LineWidth', 2, 'DisplayName', 'pitch');
plot(prelab_data.time, prelab_data.roll, 'LineWidth', 2, 'DisplayName', 'roll');
legend;
title("Rotation angles plot of prelab flight");
xlabel("Time (s)");
ylabel("Rotation (degrees)");
xlim([0 70]);
%ylim([0 80]);
saveas(f, "../figures/q43_rot.png");
hold off;
close all;


files = {'test_0.5.txt', 'test_0.75.txt', 'test_1.0.txt', 'test_2.0.txt', 'test_4.0.txt'};
names = {'0.5', '0.75', '1.0', '2.0', '4.0'};
in_freqs = [0.5 0.75 1.0 2.0 4.0];

% Calculated by inspecting time differences between peaks in the graphs.
time_shifts = -[(31.875 - 28.356) (21.203 - 18.959) (22.417 - 20.48) (14.377 - 13.357) (23.166 - 22.457)];

% times to clip the input data at
time_start = [15 10 12 11 11];
time_end = [43 39 39 38 39];

% to be computed below
gains = [0 0 0 0 0];
% phase shifts in radians
shifts = in_freqs .* time_shifts;

% change to 1 for time plots
draw_time_plots = 0;

for file_idx=1:length(files)
    %disp("data/" + files(file_idx));
    data = testDataDecoder("data/" + files(file_idx));

    if (draw_time_plots)
        f = figure('Renderer', 'painters', 'Position', [10 10 900 600]);
        figure(f);
        plot(data.time, data.yaw, 'LineWidth', 2);
        hold on;
        plot(data.time, data.control_yaw, 'LineWidth', 2);
        title("Altitude plot of freq. response flight for "+ names(file_idx) + " rad/s");
        xlabel("Time (s)");
        ylabel("Yaw(deg)");
        xlim([0 50]);
        grid;
        % uncomment to save
        saveas(f, "../figures/q51_time"+names(file_idx)+".png");
        hold off;
    end

    tstart = find(data.time > time_start(file_idx), 1);
    tend = find(data.time > time_end(file_idx), 1);
    if (mod(tend - tstart, 2) == 1) 
        tend = tend + 1;
    end
        
    Y_data = data.yaw(tstart:tend);
    U_data = data.control_yaw(tstart:tend);
    t_data = data.time(tstart:tend);
    
    freqs = [0:0.01:10];

    Y = fft(Y_data); 
    L = length(Y_data);
    
    % calc/plot magnitude of output
    P2 = abs(Y/L);
    P1 = P2(1:L/2+1);
    P1(2:end-1) = 2 *P1(2:end-1);
    f = figure;
    figure(f);
    
    subplot(2, 1, 1);
    semilogx(10*(0:(L/2))/L, mag2db(P1)); 
    title("Y(s) for " + string(1) + "rad/s oscillation");
    %xlabel("Frequency (rad/s)");
    grid;
    ylabel("|Y(s)| (dB)");

    % plot the phase...i think
    Ph2 = angle(Y/L);
    Ph1 = Ph2(1:L/2+1);
    Ph1(2:end-1) = 2 *Ph1(2:end-1);
    
    subplot(2, 1, 2);
    semilogx(10*(0:(L/2))/L, Ph1 / pi * 180); 
    grid;
    %title("|Y(s)| for " + string(1) + "rad/s oscillation");
    xlabel("Frequency (rad/s)");
    ylabel("angle(Y(s)) (degrees)");
    
    saveas(f, "../figures/q51_ft_"+names(file_idx)+".png");
    
    % get output Y for corresponding in frequency
    gains(file_idx) = interp1(10*(0:(L/2))/L, mag2db(P1), in_freqs(file_idx));
    
end

f = figure;
figure(f);


subplot(2, 1, 1);
semilogx(in_freqs, gains);
title("Bode plot of estimated system");
ylabel("|H(jw)| (dB)");
grid;

subplot(2, 1, 2);
semilogx(in_freqs, 180 / pi * (in_freqs .* time_shifts));
ylabel("angle(H(jw)) (degrees)");
xlabel("Frequency (rad/s)");
grid;
saveas(f, "../figures/q51_bode.png");

return;






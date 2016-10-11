data_ = load('data.csv');
path_length = []
time = []
epsilon = []
n = 200;
for i=0:length(data_)/n - 2
    path_length = [
        path_length; data_(i * n + 1:(i + 1) * n, 2)'];
        time = [time; data_(i * n + 1:(i + 1) * n, 3)'];
        epsilon = [epsilon; data_(i * n + 1,1)];
end
path_length = path_length';
time = time';
epsilon = epsilon';

path_mean = mean(path_length);
time_mean = mean(time);
figure(1);
plot(epsilon, path_mean,'DisplayName','mean'); hold on;
plot(epsilon, max(path_length),'DisplayName','max' ); 
plot(epsilon, min(path_length), 'DisplayName','min');
title('Path length vs epsilon');
ylabel('Path length (radians)');
xlabel('Epsilon (radians)');
axis([0 6 0 35]);

legend('show');
hold off;

figure(2);
plot(epsilon, time_mean,'DisplayName','mean'); hold on;
plot(epsilon, max(time),'DisplayName','max' ); 
plot(epsilon, min(time), 'DisplayName','min');
title('time vs epsilon');
ylabel('time (seconds)');
xlabel('Epsilon (radians)');
axis([0 6 0 1]);
legend('show');
hold off;

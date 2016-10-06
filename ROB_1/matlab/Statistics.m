data_ = load('output_file4.csv');
path_length = []
time = []
for i=0:4
    path_length = [path_length; data_(i * 1000 + 1:(i + 1) * 1000, 2)']
    time = [time; data_(i * 1000 + 1:(i + 1) * 1000, 3)']
end
path_length = path_length'
time = time'

fitdist =
%% Make model check, check if it is a normal distribution
for i = 1:5
    r_path = chi2gof(path_length(:,i));
    r_time = chi2gof(time(:,i));
    [h,p] = chi2gof(path_length(:,i),'cdf',chi2pdf(path_length(:,i)))
    figure(1)
    subplot(3,3,i);
    histogram(path_length(:,i));
    figure(2)
    subplot(3,3,i);
    histogram(time(:,i));
    [h_p,p_p] = kstest((path_length(:,i) - mean(path_length(:,i)))/std(path_length(:,i)), 'Alpha', 0.05);
    [h_t,p_t] = kstest((time(:,i) - mean(time(:,i)))/std(time(:,i)), 'Alpha', 0.05);
    

    if(r_path)
        fprintf('path set %d is not a normal distribution! %f %f\n', i,h_p,p_p);
    end
    if(r_time)
        fprintf('time set %d is not a normal distribution! %f %f\n', i,h_t,p_t);
    end
end

chi2data_ = random('chi2',999,10000);

figure(3);
qqplot(path_length(:,1), chi2data_)

%%
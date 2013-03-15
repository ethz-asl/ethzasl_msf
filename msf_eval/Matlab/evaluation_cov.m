
clear all
close all

binSize=10.0; % [m]

% get the data

SLAM_vicon_msf_abs_cov;
data_EVAL{1} = data; %method A

SLAM_vicon_msf_rel_cov;
data_EVAL{2} = data; %method B

nameMethodA = 'A';
nameMethodB = 'B';

mk{1}='.';
mk{2}='o';

clr{1}=[1,0,0];
clr{2}=[0.0,0.75,0];

scales=[1.25,.4,.7];

for k=1:2

    subplot(1,2,k);
    plot(data_EVAL{k}(:, 3),'b'); %translation error
    hold on;   
    
    cov = (sqrt(data_EVAL{k}(:, 6)) + sqrt(data_EVAL{k}(:, 7)) + sqrt(data_EVAL{k}(:, 8))) * 3;
    
    plot(cov,'r-'); % trace 3sigma
    
end

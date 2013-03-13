% evaluate aslam
% written by lestefan
% December 2012
% =========================================================================


clear all
close all

% configuration
%aslam_results='/home/lestefan/datasets/aslam/stereo/test.bag';
%aslam_results_no_imu='/workspace/aslam/aslam_node/no_imu.bag';
%gt='/home/lestefan/datasets/aslam/stereo/Vicon/19122012_Vicon_P0_d02_SY.bag';
%doRun=0;
%
% run evaluation - TODO: Why the f**k does this fail? Because you dont have
% the environment for ROS loaded.
%if doRun
%    system(['rosrun aslam_eval aslam_eval_run ' aslam_results ' ' gt]);
%end

binSize=10.0; % [m]

% get the data

SLAM_sensor_vicon_msf_abs_noscale;
data_EVAL{1} = data; %method A

SLAM_sensor_vicon_msf_rel_noscale;
data_EVAL{2} = data; %method B

SLAM_sensor_vicon_msf_rel_4KF; 
data_EVAL{3} = data; %method C

nameMethodA = 'A';
nameMethodB = 'B';
nameMethodC = 'C';

mk{1}='.';
mk{2}='o';
mk{3}='x';

clr{1}=[1,0,0];
clr{2}=[0.0,0.75,0];
clr{3}=[0,0,1];

scales=[1.25,.4,.7];

for k=1:3

numBins = floor(max(data_EVAL{k}(:, 2)) / binSize);

translationLow = zeros(numBins, 1);
translationHigh = zeros(numBins, 1);
translationMean = zeros(numBins, 1);

rotationLow = zeros(numBins, 1);
rotationHigh = zeros(numBins, 1);
rotationMean = zeros(numBins, 1);

rotationZLow = zeros(numBins, 1);
rotationZHigh = zeros(numBins, 1);
rotationZMean = zeros(numBins, 1);

dsVector = zeros(1,numBins);
i=1;

for ds = binSize / 2:binSize:max(data_EVAL{k}(:, 2) - binSize / 2)
    box=data_EVAL{k}(data_EVAL{k}(:, 2) > ds - binSize / 2 & data_EVAL{k}(:, 2) < ds + binSize / 2, :) / ds;
    
    %translation
    translationLow(i, 1) = prctile(box(:, 3), .05);
    translationHigh(i, 1) = prctile(box(:, 3), .95);
    translationMean(i, 1) = mean(box(:, 3));
    
    rotationLow(i, 1) = prctile(box(:,4) / pi * 180, .05);
    rotationHigh(i, 1) = prctile(box(:,4) / pi * 180, .95);
    rotationMean(i, 1) = mean(box(:,4) / pi * 180);
    
    if size(box, 2) > 4
        rotationZLow(i, 1) = prctile(box(:, 5) / pi * 180, .05);
        rotationZHigh(i, 1) = prctile(box(:, 5) / pi * 180, .95);
        rotationZMean(i, 1) = mean(box(:, 5) / pi * 180);
    end
    
    dsVector(1, i) = ds + (k - 2) * 2;
    i = i + 1;
end
figure(1)

%Translation ERROR
subplot(3,1,1)
errorbar(dsVector, translationMean * 100, translationLow * 100, translationHigh * 100, mk{k}, 'MarkerSize', ...
    12 * scales(k), 'LineWidth', 0.5,'Color',clr{k})
ylabel('Translation error [%]')
xlabel('Distance travelled [m]')
axis([0 90 0 2.5])
set(gca,'xTick', binSize / 2:binSize:160)
set(gca,'YGrid', 'on');

hold on;

%Orientation ERROR
subplot(3,1,2)
errorbar(dsVector, rotationMean, rotationLow, rotationHigh, mk{k},'MarkerSize',12*scales(k),'LineWidth',0.5,'Color',clr{k})
ylabel('Orient. err. [{}^\circ/m]')
xlabel('Distance travelled [m]')
hold on;
axis([0 90 0 0.25])
set(gca,'xTick',binSize/2:binSize:160)
set(gca,'YGrid','on');

%Gravity align ERROR
subplot(3,1,3)
errorbar(dsVector, rotationZMean, rotationZLow, rotationZHigh, mk{k},'MarkerSize',12*scales(k),'LineWidth',0.5,'Color',clr{k})
ylabel('World z-dir. err. [{}^\circ/m]')
xlabel('Distance travelled [m]')
hold on;
axis([0 90 0 0.25])
set(gca,'xTick',binSize/2:binSize:160)
set(gca,'YGrid','on');

end

subplot(3,1,2)
legend(nameMethodA, nameMethodB, nameMethodC,'Orientation','horizontal')

% save as pretty plot
%set(gca,'FontSize',3)
%matfig2pgf('filename', 'ViconComp.pgf', 'fignr', 1,'converttexttolatex',true,'figwidth', 8,'texticklabelsize','scriptsize','textextlabelsize','scriptsize')
%matlabfrag('ViconComp')
%matlab2tikz('ViconComp.tikz', 'height', '\figureheight', 'width', '\figurewidth');

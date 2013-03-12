% evaluate aslam
% written by lestefan
% December 2012
% =========================================================================


clear all
close all

% configuration
aslam_results='/home/lestefan/datasets/aslam/stereo/test.bag';
aslam_results_no_imu='/workspace/aslam/aslam_node/no_imu.bag';
gt='/home/lestefan/datasets/aslam/stereo/Vicon/19122012_Vicon_P0_d02_SY.bag';
doRun=0;

binSize=10.0; % [m]

% run evaluation - TODO: Why the f**k does this fail?
if doRun
    system(['rosrun aslam_eval aslam_eval_run ' aslam_results ' ' gt]);
end

% get the data
matlab_data_Vicon_aslam2;
data_aslam{3}=data;
matlab_data_Vicon_msf3;
data_aslam{2}=data; % msf
matlab_data_Vicon_visodo2;
data_aslam{1}=data; % no imu
mk{3}='x';
mk{2}='o';
mk{1}='.';
clr{3}=[0,0,1];
clr{2}=[0.0,0.75,0];
clr{1}=[1,0,0];
scales=[1.25,.4,.7];

for k=1:3

% make bins
%numBins=floor(max(data(:,2))/binSize);
%translationMatrix=NaN(size(data,1),numBins);
%rotationMatrix=NaN(size(data,1),numBins);
%dsVector=NaN(1,numBins);
numBins=floor(max(data_aslam{k}(:,2))/binSize);
translationLow=zeros(numBins,1);
translationHigh=zeros(numBins,1);
translationMean=zeros(numBins,1);
rotationLow=zeros(numBins,1);
rotationHigh=zeros(numBins,1);
rotationMean=zeros(numBins,1);
rotationZLow=zeros(numBins,1);
rotationZHigh=zeros(numBins,1);
rotationZMean=zeros(numBins,1);
dsVector=zeros(1,numBins);
i=1;
for ds=binSize/2:binSize:max(data_aslam{k}(:,2)-binSize/2)
    box=data_aslam{k}(data_aslam{k}(:,2)>ds-binSize/2&data_aslam{k}(:,2)<ds+binSize/2,:)/ds;
    %translation
    translationLow(i,1)=prctile(box(:,3),.05);
    translationHigh(i,1)=prctile(box(:,3),.95);
    translationMean(i,1)=mean(box(:,3));
    rotationLow(i,1)=prctile(box(:,4)/pi*180,.05);
    rotationHigh(i,1)=prctile(box(:,4)/pi*180,.95);
    rotationMean(i,1)=mean(box(:,4)/pi*180);
    if size(box,2)>4
        rotationZLow(i,1)=prctile(box(:,5)/pi*180,.05);
        rotationZHigh(i,1)=prctile(box(:,5)/pi*180,.95);
        rotationZMean(i,1)=mean(box(:,5)/pi*180);
    end
    %translationMatrix(1:size(box,1),i)=box(:,3);
    %rotationMatrix(1:size(box,1),i)=box(:,4);
    %mean(box(:,3))
    %std(box(:,3))
    dsVector(1,i)=ds+(k-2)*2;
    i=i+1;
end
figure(1)
%set(gca,'FontSize',8)
%title('Vicon ASLam Prototype v0 Evaluation')
subplot(3,1,1)
%boxplot(translationMatrix,dsVector,'outliersize',2)
errorbar(dsVector, translationMean*100, translationLow*100, translationHigh*100, mk{k},'MarkerSize',12*scales(k),'LineWidth',0.5,'Color',clr{k})
ylabel('Translation error [%]')
xlabel('Distance travelled [m]')
axis([0 90 0 2.5])
set(gca,'xTick',binSize/2:binSize:160)
set(gca,'YGrid','on');
hold on;
subplot(3,1,2)
%boxplot(rotationMatrix/pi*180,dsVector,'outliersize',2)
errorbar(dsVector, rotationMean, rotationLow, rotationHigh, mk{k},'MarkerSize',12*scales(k),'LineWidth',0.5,'Color',clr{k})
ylabel('Orient. err. [{}^\circ/m]')
xlabel('Distance travelled [m]')
hold on;
%legend('Vision-only', 'Loosely-coubled', 'Visiual-inertial')
axis([0 90 0 0.25])
set(gca,'xTick',binSize/2:binSize:160)
set(gca,'YGrid','on');
subplot(3,1,3)
%boxplot(rotationMatrix/pi*180,dsVector,'outliersize',2)
errorbar(dsVector, rotationZMean, rotationZLow, rotationZHigh, mk{k},'MarkerSize',12*scales(k),'LineWidth',0.5,'Color',clr{k})
ylabel('World z-dir. err. [{}^\circ/m]')
xlabel('Distance travelled [m]')
hold on;
%legend('Vision-only', 'Loosely-coubled', 'Visiual-inertial')
axis([0 90 0 0.25])
set(gca,'xTick',binSize/2:binSize:160)
set(gca,'YGrid','on');
end
subplot(3,1,2)
legend('Vision-only', 'Loosely-coupled', 'Tightly-coupled','Orientation','horizontal')

% save as pretty plot
%set(gca,'FontSize',3)
%matfig2pgf('filename', 'ViconComp.pgf', 'fignr', 1,'converttexttolatex',true,'figwidth', 8,'texticklabelsize','scriptsize','textextlabelsize','scriptsize')
%matlabfrag('ViconComp')
%matlab2tikz('ViconComp.tikz', 'height', '\figureheight', 'width', '\figurewidth');

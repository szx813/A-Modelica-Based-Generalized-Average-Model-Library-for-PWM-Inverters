clear;clc;
data1 = load("Detail_Model.mat");
data2 = load("Average_Model.mat");
data3 = load("GAM_1.mat");
data4 = load("GAM_2.mat");
data5 = load("GAM_3.mat");


fontSize = 14; %% 坐标轴字体大小

ax1 = subplot(2,1,1);
plot(data1.data(:,1),data1.data(:,2),'black', ...
    'LineWidth',1);
hold on;
plot(data2.data(:,1),data2.data(:,2),'--', ...
    'Color','red','LineWidth',1,'MarkerIndices',1:25:length(data2.data(:,2)));
hold on;
plot(data3.data(:,1),data3.data(:,2),'-square', ...
    'Color','#0072BD','LineWidth',1,'MarkerIndices',1:25:length(data3.data(:,2)));
hold on;
plot(data4.data(:,1),data4.data(:,2),'-v', ...
    'Color','#7E2F8E','LineWidth',1,'MarkerIndices',1:25:length(data4.data(:,2)));
hold on;
plot(data5.data(:,1),data5.data(:,2),'-^', ...
    'LineWidth',1,'MarkerIndices',1:25:length(data5.data(:,2)));
hold off;

xlabel("Time(s)");
ylabel("Inductor current(A)");
legend('Detailed model','Average model','GAM1','GAM2','GAM3');
xlim([0, 0.1]);
set(gca,'FontSize',fontSize);

ax2 = subplot(2,1,2);
plot(data1.data(:,1),data1.data(:,2),'black', ...
    'LineWidth',1);
hold on;
plot(data2.data(:,1),data2.data(:,2),'--', ...
    'Color','red','LineWidth',1,'MarkerIndices',1:25:length(data2.data(:,2)));
hold on;
plot(data3.data(:,1),data3.data(:,2),'-square', ...
    'Color','#0072BD','LineWidth',1,'MarkerIndices',1:25:length(data3.data(:,2)));
hold on;
plot(data4.data(:,1),data4.data(:,2),'-v', ...
    'Color','#7E2F8E','LineWidth',1,'MarkerIndices',1:25:length(data4.data(:,2)));
hold on;
plot(data5.data(:,1),data5.data(:,2),'-^', ...
    'LineWidth',1,'MarkerIndices',1:25:length(data5.data(:,2)));
hold off;

xlabel("Time(s)");
ylabel("Inductor current(A)");
legend('Detailed model','Average model','GAM1','GAM2','GAM3');
xlim([0, 0.1]);
set(gca,'FontSize',fontSize);

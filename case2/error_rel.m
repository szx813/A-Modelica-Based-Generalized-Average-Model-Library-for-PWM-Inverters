clear;clc;
data1 = load("Detail_Model.mat");
data2 = load("Average_Model.mat");
data3 = load("GAM_1.mat");
data4 = load("GAM_2.mat");
data5 = load("GAM_3.mat");

error_ia_ave = sum((data2.data(:,2) - data1.data(:,2)).^2)/sum(data1.data(:,2).^2);
error_ib_ave = sum((data2.data(:,3) - data1.data(:,3)).^2)/sum(data1.data(:,3).^2);
error_ic_ave = sum((data2.data(:,4) - data1.data(:,4)).^2)/sum(data1.data(:,4).^2);

error_ia_GAM1 = sum((data3.data(:,2) - data1.data(:,2)).^2)/sum(data1.data(:,2).^2);
error_ib_GAM1 = sum((data3.data(:,3) - data1.data(:,3)).^2)/sum(data1.data(:,3).^2);
error_ic_GAM1 = sum((data3.data(:,4) - data1.data(:,4)).^2)/sum(data1.data(:,4).^2);

error_ia_GAM2 = sum((data4.data(:,2) - data1.data(:,2)).^2)/sum(data1.data(:,2).^2);
error_ib_GAM2 = sum((data4.data(:,3) - data1.data(:,3)).^2)/sum(data1.data(:,3).^2);
error_ic_GAM2 = sum((data4.data(:,4) - data1.data(:,4)).^2)/sum(data1.data(:,4).^2);

error_ia_GAM3 = sum((data5.data(:,2) - data1.data(:,2)).^2)/sum(data1.data(:,2).^2);
error_ib_GAM3 = sum((data5.data(:,3) - data1.data(:,3)).^2)/sum(data1.data(:,3).^2);
error_ic_GAM3 = sum((data5.data(:,4) - data1.data(:,4)).^2)/sum(data1.data(:,4).^2);


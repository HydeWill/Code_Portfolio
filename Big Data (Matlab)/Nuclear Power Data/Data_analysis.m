clear, close all
%%Section 1
%Reads in the data
np = readcell('nuclear_plants_small_dataset.csv');
np_mat = readmatrix('nuclear_plants_small_dataset.csv');
%% Task 1
%Searches for missing values
% numel(find(isnan(np_mat)))
% numel(find(ismissing(np(:,1))))

%% Task 2
%Initialises new arrays with pre-allocation
np_nor = cell(498,13);
np_ab = cell(498,13);
%Counters to apply positions of row of numbers during sort
count_nor = 1;
count_ab = 1;
%Sorts array on whether the first column contains Normal or Abnormal
for row = 2:size(np, 1)
   if strcmpi(np(row,1),'Normal')
       np_nor(count_nor,:) =  np(row,:);
       count_nor = count_nor + 1;
   else
       np_ab(count_ab,:) = np(row,:);
       count_ab = count_ab + 1;
   end
end

%Removes string column
np_nor(:,1) = [];
np_ab(:,1) = [];
%Convers into a matrix
np_nor = cell2mat(np_nor);
np_ab = cell2mat(np_ab);

stats_nor = zeros(6,12);
stats_ab = zeros(6,12);
tmp = [];
%Loops through columns and finds the Maximum for each
for col = 1:size(np_nor,2)
    stats_nor(1,col) =  max(np_nor(:,col));
end

for col = 1:size(np_ab,2)
    stats_ab(1,col) = max(np_ab(:,col));
end

%Loops through columns and finds the Minimum for each
for col = 1:size(np_nor,2)
    stats_nor(2,col) = min(np_nor(:,col));
end
for col = 1:size(np_ab,2)
    stats_ab(2,col) = min(np_ab(:,col));
end

%Loops through columns and finds the mean for each
for col = 1:size(np_nor,2)
    stats_nor(3,col) = mean(np_nor(:,col));
end
for col = 1:size(np_ab,2)
    stats_ab(3,col) = mean(np_ab(:,col));
end

%Loops through columns and finds the median for each
for col = 1:size(np_nor,2)
    stats_nor(4,col) = median(np_nor(:,col));
end
for col = 1:size(np_ab,2)
    stats_ab(4,col) = median(np_ab(:,col));
end

%Loops through columns and finds the mode for each
for col = 1:size(np_nor,2)
    stats_nor(5,col) = mode(np_nor(:,col));
end
for col = 1:size(np_ab,2)
    stats_ab(5,col) = mode(np_ab(:,col));
end

%Loops through columns and finds the mean for each
for col = 1:size(np_nor,2)
    stats_nor(6,col) = var(np_nor(:,col));
end
for col = 1:size(np_ab,2)
    stats_ab(6,col) = var(np_ab(:,col));
end
%Creates a table with the feature names and statistic row names applied to 
%each group
S_nor = array2table(stats_nor,'VariableNames',{'Power_range_sensor_1','Power_range_sensor_2', 'Power_range_sensor_3', 'Power_range_sensor_4','Pressure _sensor_1','Pressure _sensor_2','Pressure _sensor_3','Pressure _sensor_4','Vibration_sensor_1','Vibration_sensor_2','Vibration_sensor_3','Vibration_sensor_4'},'RowNames',{'Max','Min','Mean','Median','Mode','Var'});
S_ab = array2table(stats_ab,'VariableNames',{'Power_range_sensor_1','Power_range_sensor_2', 'Power_range_sensor_3', 'Power_range_sensor_4','Pressure _sensor_1','Pressure _sensor_2','Pressure _sensor_3','Pressure _sensor_4','Vibration_sensor_1','Vibration_sensor_2','Vibration_sensor_3','Vibration_sensor_4'},'RowNames',{'Max','Min','Mean','Median','Mode','Var'});

%Box Plots

%Normal group boxplots for the 12 features
%Seperated into a 3 by 4 subplot for power, pressure and vibrations
figure,
subplot(3,4,1)
boxplot(np_nor(1,:))
title('Group Normal - Power range sensor 1')

subplot(3,4,2)
boxplot(np_nor(2,:))
title('Group Normal - Power range sensor 2')

subplot(3,4,3)
boxplot(np_nor(3,:))
title('Group Normal - Power range sensor 3')

subplot(3,4,4)
boxplot(np_nor(4,:))
title('Group Normal - Power range sensor 4')

subplot(3,4,5)
boxplot(np_nor(5,:))
title('Group Normal - Pressure sensor 1')

subplot(3,4,6)
boxplot(np_nor(6,:))
title('Group Normal - Pressure sensor 2')

subplot(3,4,7)
boxplot(np_nor(7,:))
title('Group Normal - Pressure sensor 3')

subplot(3,4,8)
boxplot(np_nor(8,:))
title('Group Normal - Pressure sensor 4')

subplot(3,4,9)
boxplot(np_nor(9,:))
title('Group Normal - Vibration sensor 1')

subplot(3,4,10)
boxplot(np_nor(10,:))
title('Group Normal - Vibration sensor 2')

subplot(3,4,11)
boxplot(np_nor(11,:))
title('Group Normal - Vibration sensor 3')

subplot(3,4,12)
boxplot(np_nor(12,:))
title('Group Normal - Vibration sensor 4')

%Abnomral group boxplots for the 12 features
%Seperated into a 3 by 4 subplot for power, pressure and vibrations
figure,
subplot(3,4,1)
boxplot(np_ab(1,:))
title('Group Abnormal - Power range sensor 1')

subplot(3,4,2)
boxplot(np_ab(2,:))
title('Group Abnormal - Power range sensor 2')

subplot(3,4,3)
boxplot(np_ab(3,:))
title('Group Abnormal - Power range sensor 3')

subplot(3,4,4)
boxplot(np_ab(4,:))
title('Group Abnormal - Power range sensor 4')

subplot(3,4,5)
boxplot(np_ab(5,:))
title('Group Abnormal - Pressure sensor 1')

subplot(3,4,6)
boxplot(np_ab(6,:))
title('Group Abnormal - Pressure sensor 2')

subplot(3,4,7)
boxplot(np_ab(7,:))
title('Group Abnormal - Pressure sensor 3')

subplot(3,4,8)
boxplot(np_nor(8,:))
title('Group Normal - Pressure sensor 4')

subplot(3,4,9)
boxplot(np_ab(9,:))
title('Group Abnormal - Vibration sensor 1')

subplot(3,4,10)
boxplot(np_ab(10,:))
title('Group Abnormal - Vibration sensor 2')

subplot(3,4,11)
boxplot(np_ab(11,:))
title('Group Abnormal - Vibration sensor 3')

subplot(3,4,12)
boxplot(np_ab(12,:))
title('Group Abnormal - Vibration sensor 4')

%%Task 3
%Correlation matrix
%corr_mat = corrcoef(np_nor);
Corr_Mat = zeros(12,12);
for x = 1:12
    for y =1:12
        Corr_Mat(x,y) = Correlation(np_nor(:,x),np_nor(:,y));
    end
end

%Checks correlation matrix for moderate or greater relationships 
for row = 1:12
    for col = 1:12
        if 0.3 < Corr_Mat(row,col) || Corr_Mat(row,col) < -0.3
            disp(Corr_Mat(row,col));
        end
    end
end

%%Section 2

%%Task 4
%Date shuffling

%Numeric data
x = np_mat(:,2:end);
%Row labels
y = np(2:end,1);

%Vector of randomly sorted numbers
n = numel(np_mat(:,1)) ; 
rnd_row = randperm(n); 

%Training data allocation
train_x = x(rnd_row(1:round(0.7*n)),:);
train_y = y(rnd_row(1:round(0.7*n)),:);
%Testing data allocation
test_x = x(rnd_row(round(0.7*n)+1:end),:);
test_y = y(rnd_row(round(0.7*n)+1:end),:);

%Train_y binary conversion for ANN
t_train = zeros(numel(train_y),1);
for i = 1:size(train_y,1)
    if strcmpi(train_y(i,1),'Normal')
        t_train(i) = 1;
    end
end
%Test_y binary conversion for ANN
t_test = zeros(numel(test_y),1);
for i = 1:size(test_y,1)
    if strcmpi(test_y(i,1),'Normal')
        t_test(i) = 1;
    end
end
%Transposes for ANN input
x_tt = train_x';
t = t_train';
%% Task 5 Decision tree, SVM and ANN

%Training a decision tree and then predicting 
tree = fitctree(train_x, train_y,'MinParentSize',1);
%Tree labels
pred_tree = predict(tree,test_x);

%Decision tree confusion matrix plot 
figure,
confusionchart(test_y,pred_tree)
%view(tree,'mode','graph'); - Views tree structure

%Training SVM classification model
lin_model = fitcsvm(train_x, train_y);

%Model predict with confusion chart
pred_class = predict(lin_model,test_x); %Linear Labels 
%SVM confusion matrix
figure, confusionchart(test_y,pred_class);

%Creates ANN network model
trainFcn = 'trainscg';
net_model = patternnet(15, trainFcn);

%Training, Testing, Validation split
%Applies for full training
% net_model.divideParam.trainRatio = 100/100;
% net_model.divideParam.valRatio = 0/100;
% net_model.divideParam.testRatio = 0/100;

%Trains the training data using the network model
net = train(net_model,x_tt,t);
%Predicts test set targets
y = net(test_x');
%ANN Labels
classes = vec2ind(y);

%ANN confusion matrix 
figure, 
plotconfusion(t_test',y)
xlabel('Predicted Class')
ylabel('True Class')

view(net)

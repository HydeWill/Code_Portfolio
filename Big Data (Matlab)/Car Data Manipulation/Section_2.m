%% Section 2 

%Data managment
%Reads in file as a matrix to store numeric values
CD = readmatrix('car_data.xls');
%Formats command window to more readable numbers
format long g

%Deletes column 9, all values are missing
CD(:,9) = [];

%Creates new matrix without any missing values
% CD=CD(~any(ismissing(CD),2),:); <-alternative 
CD = Deletion(CD);

%Column vector variable definitions
Mpg = CD(:,1);
Cy = CD(:,2);
Dis = CD(:,3);
HP = CD(:,4);
W = CD(:,5);
Acc = CD(:,6);
MY = CD(:,7);
Or = CD(:,8);

%% Task 3 -Statistical analysis
%Functions commented for ease of use

% Mean for MPG, Hp, Weight and Acc

% Mean(Mpg)
% Mean(HP)
% Mean(CD(:,5))
% Mean(CD(:,6))

%Median for MPG, Hp, Weight and Acc

% Median(Mpg)
% Median(HP)
% Median(W)
% Median(Acc)

%Minimun for MPG, Hp, Weight and Acc

% Min(Mpg)
% Min(HP)
% Min(W)
% Min(Acc)

%Maximun for MPG, Hp, Weight and Acc

% Max(Mpg)
% Max(HP)
% Max(W)
% Max(Acc)

%Standard Deviation

% StdDev(Mpg)
% StdDev(HP)
% StdDev(W)
% StdDev(Acc)

%% Task 4 - Graphs
% Box plots
figure,
subplot(2,2,1)
boxplot(Mpg)
ylabel('Miles per Gallon')
title('Box Plot for Mpg')

subplot(2,2,2)
boxplot(Acc)
ylabel('Acceleration')
title('Box Plot for Acceleration')

subplot(2,2,3)
boxplot(HP)
ylabel('Horse Power')
title('Box Plot for Horsepower')
 
subplot(2,2,4)
boxplot(W)
ylabel('Weight')
title('Box Plot for Weight')

% Scatter plots
figure,
subplot(2,2,1)
scatter(Acc,Mpg)
xlabel('Acceleration')
ylabel('Mpg')
title('Scattter graph plotting Mpg against Acceleration')

subplot(2,2,2)
scatter(HP,Mpg)
xlabel('Horsepower')
ylabel('Mpg')
title('Scattter graph plotting Horsepower against Mpg')

subplot(2,2,3)
scatter(W,HP)
xlabel('Weight')
ylabel('Horsepower')
title('Scattter graph plotting Weight against Horsepower')

%Density plots
figure,
subplot(2,2,1)
[x,f]= ksdensity(Mpg);
plot(f,x,'LineWidth', 2)
title('Graph showing density plot for Mpg')
xlabel('MPG')

subplot(2,2,2)
[x,f]= ksdensity(Acc);
plot(f,x,'LineWidth', 2)
title('Graph showing density plot for Acceleration')
xlabel('Acceleration')

subplot(2,2,3)
[x,f]= ksdensity(HP);
plot(f,x,'LineWidth', 2)
title('Graph showing density plot for Horsepower')
xlabel('Horsepower')
 
subplot(2,2,4)
[x,f]= ksdensity(W);
plot(f,x,'LineWidth', 2)
title('Graph showing density plot for Weight')
xlabel('Weight')

%% Task 5 - Correlation matrix

% Corr_Mat = zeros(8,8);
% for x = 1:8
%     for y =1:8
%         Corr_Mat(x,y) = Corr(CD(:,x),CD(:,y));
%     end
% end
% 
% figure, scatter(HP,Dis)
% figure, scatter(HP,Mpg)
% figure, scatter(Cy,Dis)


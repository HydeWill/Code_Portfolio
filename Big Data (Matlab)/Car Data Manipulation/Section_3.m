%% Section 3

%Data management
CD = readmatrix('car_data.xls');
%Formats command window to more readable numbers
format long g

%Deletes column 9, all values are missing
CD(:,9) = [];

%Creates new matrix without any missing values
% CD=CD(~any(ismissing(CD),2),:); <-alternative 
CD = Deletion(CD);

%% Training and Tesing 
n = numel(CD(:,1)) ; 
rnd_row = randperm(n);  
CD_train = CD(rnd_row(1:round(0.7*n)),:) ; 
CD_test = CD(rnd_row(round(0.7*n)+1:end),:);

Mpg_train = CD_train(:,1);
HP_train = CD_train(:,4);
W_train = CD_train(:,5);
Acc_train = CD_train(:,6);

Mpg_test = CD_test(:,1);
HP_test = CD_test(:,4);
W_test = CD_test(:,5);
Acc_test = CD_test(:,6);

%% Acceleration vs Mpg
%Defines x and y that is being plotted
x = Acc_train;
y = Mpg_train;

%Creates row of ones
one = ones(1,numel(x));
%Matrix of x values
X = [one' x];

% Trains x and y to find weights of the regression line
Weights = Train(x,y);
%Calculates predicted y by applying the weights to its x value
Y_predict = X*Weights;

%Plots training set data against the regression line
figure,
scatter(x,y)
hold on
plot(x,Y_predict)
xlabel('Acceleration')
ylabel('Mpg')
title('Linear Regression Plotting Training for Acceleration vs Mpg')
grid on

%Calculates error, sums it and then takes the average of it
err = (y - Y_predict);
SSE = dot(err,err);
MSE = SSE/numel(x);
MSE
% Alternative - MSE = dot(dev_y,dev_y) - (dot(Sxy,Sxy)/Sxx);

%Assigns testing set values for the same features as the train
x = Acc_test;
y = Mpg_test;

%Creates new length of ones for the test set and calculates predicted y
one = ones(1,numel(x));
X = [one',x];
Y_predict = X*Weights;

%Plots testing set data against the regression line
figure,
scatter(x,y)
hold on
plot(x,Y_predict)
xlabel('Acceleration')
ylabel('Mpg')
title('Linear Regression Plotting Testing for Acceleration vs Mpg')
grid on

err = (y - Y_predict);
SSE = dot(err,err);
MSE = SSE/numel(x);
MSE


%% Horsepower vs Mpg
%Repeats regression training and testing process for different features
x = HP_train;
y = Mpg_train;

one = ones(1,numel(x));
X = [one' x];

Weights = Train(x,y);
Y_predict = X*Weights;

figure,
scatter(x,y)
hold on
plot(x,Y_predict)
xlabel('Horsepower')
ylabel('Mpg')
title('Linear Regression Plotting Training for Horsepower vs Mpg')
grid on

err = (y - Y_predict);
SSE = dot(err,err);
MSE = SSE/numel(x);
MSE

x = HP_test;
y = Mpg_test;

one = ones(1,numel(x));
X = [one',x];
Y_predict = X*Weights;

figure,
scatter(x,y)
hold on
plot(x,Y_predict)
xlabel('Horsepower')
ylabel('Mpg')
title('Linear Regression Plotting Testing for Horsepower vs Mpg')
grid on

err = (y - Y_predict);
SSE = dot(err,err);
MSE = SSE/numel(x);
MSE

%% Weight vs Horsepower
%Repeats regression training and testing process for different features
x = W_train;
y = HP_train;

one = ones(1,numel(x));
X = [one' x];

Weights = Train(x,y);
Y_predict = X*Weights;

figure,
scatter(x,y)
hold on
plot(x,Y_predict)
xlabel('Weight')
ylabel('Horsepower')
title('Linear Regression Plotting Training for Weight vs Horsepower')
grid on

err = (y - Y_predict);
SSE = dot(err,err);
MSE = SSE/numel(x);
MSE
% Alternative - MSE = dot(dev_y,dev_y) - (dot(Sxy,Sxy)/Sxx);

x = W_test;
y = HP_test;

one = ones(1,numel(x));
X = [one',x];
Y_predict = X*Weights;

figure,
scatter(x,y)
hold on
plot(x,Y_predict)
xlabel('Weight')
ylabel('Horsepower')
title('Linear Regression Plotting Testing for Weight vs Horsepower')
grid on

err = (y - Y_predict);
SSE = dot(err,err);
MSE = SSE/numel(x);
MSE

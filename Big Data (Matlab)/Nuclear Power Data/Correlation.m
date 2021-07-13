%Returns Pearsons correlation of two inputs
function r =Correlation(x,y)
num_total = 0;
X_sum = 0;
Y_sum = 0;
for i = 1:numel(x)
    num_total = num_total + (x(i)-mean(x))*(y(i)-mean(y));
    X_sum = X_sum + (x(i)-mean(x))^2;
    Y_sum = Y_sum + (y(i)-mean(y))^2;
    
end
r = num_total/sqrt(X_sum*Y_sum);


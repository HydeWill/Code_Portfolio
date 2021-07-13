function s = StdDev(i)
n = numel(i);
sum = 0;
for x = 1:n
    sum = sum + (i(x) - Mean(i))^2;
end
var = sum/n;
s = sqrt(var);

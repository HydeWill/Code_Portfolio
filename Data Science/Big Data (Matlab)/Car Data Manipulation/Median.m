function t = Median(i)
%Sorts vector
si = Quick_Sort(i);
%Finds the amount of values in vector
x = numel(si);
Mod = mod(x,2);
%Odd number of values gives a direct value
if Mod == 1
    n = ceil(x/2);
    t = si(n);
end
%Even number of values gives mean of two middle numbers
if Mod == 0
    n = x/2;
    t = (si(n)+si(n+1))/2;
end
function t = Quick_Sort(x)

n = numel(x);
if n<2
    t = x;
    return;
end

x1 = [];
x2 = [];

for i = 1:n-1
    if x(i) < x(n)
        x1 = [x1 x(i)];
    else
        x2 = [x2 x(i)];
    end
end

t = [Quick_Sort(x1) x(n) Quick_Sort(x2)];
    

%Cycles through vector and replaces temp with any value higher
function temp = Max(i)
n = numel(i);
temp = 0;
for t = 1:n
    if temp < i(t)
        temp = i(t);
    end
end

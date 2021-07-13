%Cycles through vector and replaces temp if lower than previous temp
function temp = Min(i)
n = numel(i);
temp = inf;
for t = 1:n
    if temp > i(t)
        temp = i(t);
    end
end

    
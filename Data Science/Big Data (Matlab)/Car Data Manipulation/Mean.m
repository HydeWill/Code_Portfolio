%Sums vector and divides by the amount of value in the vector
function mean = Mean(i)
total = 0;
for row = 1:numel(i)
    total = total + i(row);
end
mean = total/numel(i);
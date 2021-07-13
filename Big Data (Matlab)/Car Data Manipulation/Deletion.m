%Cycles through array and deletes row if a value is missing 
function i = Deletion(i)
n = numel(i(:,1));
for row = n:-1:1
    for col = 1:numel(i(row,:))
        if ismissing(i(row,col)) == 1
            i(row,:) = [];
        end
    end
end

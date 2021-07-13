clear, close all
%% Task 8 - MapReduce

%Reads in data
ds=tabularTextDatastore('nuclear_plants_big_dataset.csv', 'TreatAsMissing','NA','MissingValue',0);
ds.SelectedVariableNames='Power_range_sensor_1';

%Functions calls
fun1a=@mapsum_mean;
fun1b=@redsum_mean;

%Get sum and mean values
outds1=mapreduce(ds, fun1a, fun1b);
result1=readall(outds1);
result1=result1{:,2};

sum_val=result1{1}
mean_val=result1{2}

%% Functions 

% Mean mapper
function mapsum_mean(data, info, intermKVStore)
PRS=data.Power_range_sensor_1;
sumlen=[sum(PRS) length(PRS)];
add(intermKVStore, 'sumlen',sumlen);
end
% Mean reducer
function redsum_mean(intermKey, intermValIter, outKVStore)
sumlen=[0 0];
while hasnext(intermValIter)
    sumlen=sumlen+getnext(intermValIter);
end
add(outKVStore, 'sum',sumlen(1));
add(outKVStore, 'mean', sumlen(1)/sumlen(2));
end



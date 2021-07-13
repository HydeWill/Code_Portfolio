%% Section 1

% Task 1 - File read
%Reads in file as a table to store both numeric and non-numeric values
CD = readtable('car_data.xls');


%% Task 2 - Missing Values

%Calculations to comment on missing values
%Finds the amount of missing values in the table
s  = size(find(ismissing(CD)));

%Calculates the percentage of values that are missing in a column
%MPG column
Miss_MPG = find(ismissing(CD(:,1)));
per_MPG = size(Miss_MPG)/size(CD(:,1))*100;
%Horsepower column
Miss_Horse = find(ismissing(CD(:,4)));
per_Horse = size(Miss_Horse)/size(CD(:,4))*100;

%Creates new table without any missing values
%CD=CD(~any(ismissing(CD),2),:);
CD = Deletion(CD);





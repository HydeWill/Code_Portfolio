format long g
fid = fopen('car_numeric_data.txt', 'r+');
tmpChar = fscanf(fid, '%c', 1);
ch_arr = ('');
while ~isempty(tmpChar)
    while  ~any(tmpChar == sprintf('\r\n'))
        if isempty(tmpChar)
            break
        end
        fprintf(num2str(tmpChar));
        ch_arr = append(ch_arr, tmpChar);
        tmpChar = fscanf(fid,'%c',1);
    end
    tmpChar = fscanf(fid,'%c',1);
    ch_arr = append(ch_arr, ' ');
    fprintf(' ')
end
ch_arr
fprintf('\n');
fclose(fid);
CD = split(ch_arr);
CD(end) = [];
size(CD)


CD =reshape(CD,[8,406]);
CD = CD';
% CD =str2double(CD)

% f = readmatrix('car_numeric_data.txt');
% for i = 1:size(CD,1)
%     for o = 1:size(CD,2)
%         if f(i,o) ~= CD(i,o)
%             fprintf('No');
%         end
%     end
% end

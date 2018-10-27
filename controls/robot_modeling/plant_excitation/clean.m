num_entries_per_row = 10;
filename = 'gyro_out_vely_4';

fid = fopen(filename);
fid_out = fopen([filename, '_cleaned'], 'w');

tline = fgetl(fid);
while ischar(tline)
    % Hacky way to remove debug lines of output in dataset
    if (length(strfind(tline, ',')) == num_entries_per_row - 1)
        fprintf(fid_out, '%s\n', tline);
    end
    
    tline = fgetl(fid);
end
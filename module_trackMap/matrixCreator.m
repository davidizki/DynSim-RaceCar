function M = matrixCreator(filename)

fileID = fopen(filename);
M = textscan(fileID,'%s','delimiter',',');
M = M{1}; % everything stored by default in a "big" single cell
pattern=";";

j = 0;
Mtemp = zeros(length(M),1);
for i = 1:length(M)
    if contains(M{i},pattern) % ignore the entries containing ";" (at the end of each line)
    else
        j = j+1;
        Mtemp(j) = str2double(M{i}); % store variables in a temporal 
    end
end

while mod(length(Mtemp),3) ~= 0
    Mtemp(length(Mtemp)+1) = 0;
end

M = reshape(Mtemp,3,[]).';
M(~any(M,2),:) = []; % remove empty rows
M(end,:) = M(1,:);

end

% clc
clear

obj = udp('192.168.2.97',4555,'Terminator','');
% obj.Terminator = '';
fopen(obj);

formatSpecRead = '%s';

% feedback = 	(obj,formatSpecRead);
formatSpecIR = '$K4DRV,REQ,IR-P';

for ii = 1:1
    fprintf(obj,formatSpecIR);
%     for jj = 1:5
    feedbackIR = fscanf(obj,formatSpecRead);
%     end
end

[~,pos] = textscan(feedbackIR,'%s',3,'Delimiter',',');
C = cell2mat(textscan(feedbackIR(16+1:end),'%f',12,'Delimiter',',')).'

fclose(obj);


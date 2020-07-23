function [] = NinetyDeg_test()
%90DEG_TEST Summary of this function goes here
%   Detailed explanation goes here

fileID = fopen('input_90deg_test.txt','r');
formatSpec = '%f';
A = fscanf(fileID,formatSpec);
fclose(fileID);

fileID = fopen('output_90deg_test.txt','w');
 for i=1:length(A)
 	B(i) = tan(A(i));
    C(i) = sensitive90zone(A(i));
    D(i) = tan(C(i));
    fprintf(fileID,'%12.8f %20.10f %12.8f %20.10f\n',A(i),B(i),C(i),D(i));
 end

%P = [A ; B' ; C' ; D'];
%P = [A;B']; 


%fprintf(fileID,'%12.8f %12.8f %12.8f %12.8f\n',P);

fclose(fileID);

end


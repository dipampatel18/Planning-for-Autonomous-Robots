function [ID]= IDPoint(newnode)

% This function is used to generate a unique ID for each node. 

P = floor(newnode(1));
Q = floor(newnode(2));

TempID = strcat(num2str(P),num2str(Q));
ID = str2num(TempID);

end
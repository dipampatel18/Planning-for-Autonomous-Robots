function [DLNew] = MoveDownLeft(CurrentNode)

x = CurrentNode(1);
y = CurrentNode(2);

if x > 0 && y > 0
   
   XNew = x - 1;
   YNew = y - 1;
    
   DLNew = [XNew, YNew];

else
    
   DLNew = [];    
   
end
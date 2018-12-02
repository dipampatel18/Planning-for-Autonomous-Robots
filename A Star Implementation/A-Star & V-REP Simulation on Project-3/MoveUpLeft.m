function [ULNew] = MoveUpLeft(CurrentNode)
x = CurrentNode(1);
y = CurrentNode(2);

if x > 0 && y < 150
    
    XNew = x - 1;
    YNew = y + 1;
  
    ULNew = [XNew, YNew];
else
    
    ULNew = [];
    
end
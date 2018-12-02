function [URNew] = MoveUpRight(CurrentNode)

x = CurrentNode(1);
y = CurrentNode(2);

if x < 100 && y < 150
    
   XNew = x + 1;
   YNew = y + 1;
   
   URNew = [XNew, YNew];

else
    
   URNew = [];
    
end
function [DRNew] = MoveDownRight(CurrentNode)

x = CurrentNode(1);
y = CurrentNode(2);

if x < 250 && y > 0
    
   XNew = x+1;
   YNew = y-1;
    
   DRNew = [XNew, YNew];
   
else
    
    DRNew = [];
    
end
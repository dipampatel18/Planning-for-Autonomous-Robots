function [DNew] = MoveDown(CurrentNode)

y = CurrentNode(2);

if y > 0
   
   XNew = CurrentNode(1);
   YNew = y-1;
    
   DNew = [XNew, YNew];
    
else
    
   DNew = [];
        
end
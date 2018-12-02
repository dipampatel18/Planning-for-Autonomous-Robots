function [UNew] = MoveUp(CurrentNode)

y = CurrentNode(2);

if y < 150

    XNew = CurrentNode(1);
    YNew = y + 1;

    UNew = [XNew, YNew];

else
    
    UNew = [];
    
end
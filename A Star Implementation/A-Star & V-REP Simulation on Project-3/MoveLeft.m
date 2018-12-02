function [LNew] = MoveLeft(CurrentNode)

x = CurrentNode(1);

if x > 0
    
    XNew = x - 1;
    YNew = CurrentNode(2);

    LNew = [XNew, YNew];

else
    
    LNew = [];

end
function [RNew] = MoveRight(CurrentNode)

x = CurrentNode(1);

    if x < 100
        
    XNew = x + 1;
    YNew = CurrentNode(2);
    
    RNew = [XNew, YNew];
    
    else
        
    RNew = [];
        
end
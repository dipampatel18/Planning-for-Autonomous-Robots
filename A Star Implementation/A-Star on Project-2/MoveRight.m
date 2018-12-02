function [RNew] = MoveRight(CurrentNode)

x = CurrentNode(1);

    if x < 250
        
    XNew = x+1;
    YNew = CurrentNode(2);
    
    RNew = [XNew, YNew];
    
    else
        
    RNew = [];
        
end
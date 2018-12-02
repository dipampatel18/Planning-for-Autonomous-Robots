%% Project-3- A* in Project-2

clearvars
close all
clc

A = 1;                     % Used for denoting the New Node
B = 0;                     % Used for denoting the Parent Node

%% Starting and End Point Inputs from User

StartX(A) = input('Enter the X co-ordinate of Starting Point (0 - 250): ');
StartY(A) = input('Enter the Y co-ordinate of Starting Point (0 - 150): ');

StartPoint = [StartX(A), StartY(A)];

% Since the value of starting point can be any of the co-ordinates,
% their values are stored in the form of loop run by the variable A

% A = A;

ParentNode(:,A) = B;                    % Storing the Value in a Parent Node starting from 0
B = 1;                                  % Assigning the Value of B as 1 to start the number of Parent Nodes

EndX = input('Enter the X co-ordinate of End Point (0 - 250): ');
EndY = input('Enter the Y co-ordinate of End Point (0 - 150): ');

EndPoint = [EndX, EndY];

tic

%% Restrictions for User Input Points

[HP] = HalfPlane (StartX(A), StartY(A));     % Checking if the Entered Co-ordinate doesn't lie inside the Obstacle Space 
P = HP;

[HP] = HalfPlane (EndX, EndY);           % Checking if the Entered Co-ordinate doesn't lie inside the Obstacle Space 
Q = HP;

%% Plotting of Graph and Objects

if ((P == 0) && (Q == 0))           % To ensure that the user has Entered Correct Co-ordinates of Start and End Points
    
% Graph

plot (0, 0, 250, 150, '-s')             % Plotting the X and Y axis of the Graph
axis equal
grid on                                 % Used to depict Gridlines in the Graph

% Rectangle

rectangle('Position', [55 67.5 50 45], 'EdgeColor', 'red', 'LineWidth', 1.0)    % Co-ordinates of One Vertex and the Length of Adjacent Sides
hold on

% Circle

C = [180, 120];                         % Co-ordinates of Center of the Circle
R = 15;                                 % Radius of the Circle

viscircles (C, R, 'Color', 'red', 'LineWidth', 1.0)
axis equal
hold on

% Polygon

line ([145; 168; 188; 165; 158; 120; 145], [14; 14; 51; 89; 51; 55; 14],...
    'Color', 'red', 'LineWidth', 1.0);      % Plotting Line-segments wrt x and y co-ordinates of the Polygon
axis equal
hold on

%% 

% Initialize the variables.
Nodes = [];
NodesInfo = [];

% Initialize the start node.
Nodes(:, :, 1) = StartPoint;

% Get id of the start Node
    
ID = IDPoint(StartPoint);    

% Initialize NodeInfo for start node
% NodesInfo = [Node#, ParentNode#, cost, CP, H, ID];

NodesInfo(:,:,1) = [1,0,0,0,0,ID]; 
    
ClosedNodes = [];
ClosedNodesInfo = [];
    
% Initialize the child and parent node number variables.

A = 2; % Child node number variable
B = 1; % Parent Node Number Variable
C = 1;
    
while (StartX ~= EndX || StartY ~= EndY)  % To check whether the Starting Point is equivalent to the End Point
        
        % Initialize the parent node in each loop
        CurrentNode = Nodes(:,:,B);
        ClosedNodes(:,:,C) = CurrentNode;
        
        ID = IDPoint(CurrentNode);
        ClosedNodesInfo(:,:,C) = ID;

        plot(CurrentNode(1),CurrentNode(2),'.','color','yellow')

        
%% Move Left

[LNew] = MoveLeft(CurrentNode);
[HP] = HalfPlane(LNew(1), LNew(2));
IDCheck = IDPoint(LNew);
       
        if HP == 0
               
            if (~any(IDCheck == NodesInfo(1,6,:))) 
             
                TempCost = NodesInfo(:,3,B);
                CP = TempCost + 1;
                H = sqrt(((LNew(1) - EndPoint(1)) ^ 2) + ((LNew(2) - EndPoint(2)) ^ 2));
                F = CP + H;
                    
                Nodes(:,:,A) = LNew;
                NodesInfo(:,:,A) = [A,B,CP,H,F,IDCheck];
                A = A + 1;
                    
            elseif  (~any(IDCheck == ClosedNodesInfo(1,1,:)))
     
                N = find(IDCheck == NodesInfo(1,6,:));
                cost = NodesInfo(1, 5, N);
                TempCost = NodesInfo(:,3,B);
                CP = TempCost + 1;
                H = sqrt(((LNew(1) - EndPoint(1)) ^ 2) + ((LNew(2) - EndPoint(2)) ^ 2));
                F = CP + H;
                   
                if cost > F
                    NodesInfo(:,:,N) = [A, B, CP, H, F, IDCheck];
                end
                    
            end
                
                if LNew(1) == EndPoint(1) && LNew(2) == EndPoint(2)
                    break
                end 
                
        end
            
        
%% Move Down        
        
[DNew] = MoveDown(CurrentNode);
[HP] = HalfPlane(DNew(1), DNew(2));
IDCheck = IDPoint(DNew);
       
        if HP == 0
               
            if (~any(IDCheck == NodesInfo(1,6,:))) 
                    TempCost = NodesInfo(:,3,B);
                    CP = TempCost + 1;
                    H = (sqrt(((DNew(1) - EndPoint(1)) ^ 2)+((DNew(2) - EndPoint(2)) ^ 2)));
                    F = CP + H;
                    
                    Nodes(:,:,A) = DNew;
                    NodesInfo(:,:,A) = [A, B, CP, H, F, IDCheck];
                    A = A + 1;                    
                    
                elseif  (~any(IDCheck == ClosedNodesInfo(1,1,:)))
                    N = find(IDCheck == NodesInfo(1,6,:));
                    cost = NodesInfo(1,5,N);
                    TempCost = NodesInfo(:,3,B);
                    CP = TempCost + 1;
                    H = sqrt(((DNew(1) - EndPoint(1)) ^ 2)+((DNew(2) - EndPoint(2)) ^ 2));
                    F = CP+H;
                
                    if cost>F
                        NodesInfo(:,:,N) = [A, B, CP, H, F, IDCheck];
                    end
                end
                
                if DNew(1) == EndPoint(1) && DNew(2) == EndPoint(2)
                    break
                end
                
            end
                
            
%% Move Right            

[RNew] = MoveRight(CurrentNode);
[HP] = HalfPlane(RNew(1), RNew(2));
IDCheck = IDPoint(RNew);
       
        if HP == 0
               
            if (~any(IDCheck == NodesInfo(1,6,:))) 
                      
            
                    TempCost = NodesInfo(:,3,B);
                    CP = TempCost + 1;
                    H = (sqrt(((RNew(1) - EndPoint(1))^2) + ((RNew(2) - EndPoint(2))^2)));
                    F = CP + H;
                    
                    Nodes(:,:,A) = RNew;
                    NodesInfo(:,:,A) = [A,B,CP,H,F,IDCheck];
                    A = A + 1;                    
                    
            elseif  (~any(IDCheck == ClosedNodesInfo(1,1,:)))
                    N = find(IDCheck == NodesInfo(1,6,:));
                    cost = NodesInfo(1,5,N);
                    TempCost = NodesInfo(:,3,B);
                    CP = TempCost + 1;
                    H = sqrt(((RNew(1)- EndPoint(1)) ^ 2) + ((RNew(2) - EndPoint(2)) ^ 2));
                    F = CP + H;
            
                    if cost>F
                        NodesInfo(:,:,N) = [A,B,CP,H,F,IDCheck];
                    end
            end
                
                if RNew(1) == EndPoint(1) && RNew(2) == EndPoint(2)
                    break
                end   
                
        end
            

        
%% Move Up

[UNew] = MoveUp(CurrentNode);
[HP] = HalfPlane(UNew(1), UNew(2));
IDCheck = IDPoint(UNew);
       
        if HP == 0
               
            if (~any(IDCheck == NodesInfo(1,6,:))) 
                      
                    TempCost = NodesInfo(:,3,B);
                    CP = TempCost + 1;
                    H = (sqrt(((UNew(1)-EndPoint(1))^2)+((UNew(2)-EndPoint(2))^2)));
                    F = CP+H;
                    Nodes(:,:,A) = UNew;
                    NodesInfo(:,:,A) = [A,B,CP,H,F,IDCheck];
                    A = A + 1;                    
                    
                elseif  (~any(IDCheck == ClosedNodesInfo(1,1,:)))
                    N = find(IDCheck == NodesInfo(1,6,:));
                    cost = NodesInfo(1,5,N);
                    TempCost = NodesInfo(:,3,B);
                    CP = TempCost + 1;
                    H = sqrt(((UNew(1) - EndPoint(1)) ^ 2) + ((UNew(2) - EndPoint(2)) ^ 2));
                    F = CP + H;
                   
                    if cost>F
                        NodesInfo(:,:,N) = [A,B,CP,H,F,IDCheck];
                    end
                end
                
                if UNew(1) == EndPoint(1) && UNew(2) == EndPoint(2)
                    break
                end   
            end
        
        
       
        
%% Move Down Left        

[DLNew] = MoveDownLeft(CurrentNode);
[HP] = HalfPlane(DLNew(1), DLNew(2));
IDCheck = IDPoint(DLNew);
       
        if HP == 0
               
            if (~any(IDCheck == NodesInfo(1,6,:))) 
                      
                    TempCost = NodesInfo(:,3,B);
                    CP = TempCost + 1;
                    H = (sqrt(((DLNew(1) - EndPoint(1)) ^ 2)+((DLNew(2) - EndPoint(2)) ^ 2)));
                    F = CP+H;
                    Nodes(:,:,A) = DLNew;
                    NodesInfo(:,:,A) = [A, B, CP, H, F, IDCheck];
                    A = A+1;                    
                
                elseif  (~any(IDCheck == ClosedNodesInfo(1,1,:)))
                    N = find(IDCheck == NodesInfo(1,6,:));
                    cost = NodesInfo(1,5,N);
                    TempCost = NodesInfo(:,3,B);
                    CP = TempCost + 1;
                    H = sqrt(((DLNew(1) - EndPoint(1)) ^ 2)+((DLNew(2) - EndPoint(2)) ^ 2));
                    F = CP+H;
                    
                    if cost>F
                        NodesInfo(:,:,N) = [A, B, CP, H, F, IDCheck];
                    end
                    
                end
                
                if DLNew(1) == EndPoint(1) && DLNew(2) == EndPoint(2)
                    break
                end
                
        end
        
        
 %% Move Down Right
 

[DRNew] = MoveDownRight(CurrentNode);
[HP] = HalfPlane(DRNew(1), DRNew(2));
IDCheck = IDPoint(DRNew);
       
        if HP == 0
               
            if (~any(IDCheck == NodesInfo(1,6,:))) 
                      
                    TempCost = NodesInfo(:,3,B);
                    CP = TempCost + 1;
                    H = (sqrt(((DRNew(1) - EndPoint(1)) ^ 2)+((DRNew(2) - EndPoint(2)) ^ 2)));
                    F = CP + H;
                    Nodes(:,:,A) = DRNew;
                    NodesInfo(:,:,A) = [A, B, CP, H, F, IDCheck];
                    A = A + 1;                    
                    
                elseif  (~any(IDCheck == ClosedNodesInfo(1,1,:)))
                    N = find(IDCheck == NodesInfo(1,6,:));
                    cost = NodesInfo(1,5,N);
                    TempCost = NodesInfo(:,3,B);
                    CP = TempCost + 1;
                    H = sqrt(((DRNew(1) - EndPoint(1)) ^ 2)+((DRNew(2) - EndPoint(2)) ^ 2));
                    F = CP + H;
                
                    if cost>F
                        NodesInfo(:,:,N) = [A, B, CP, H, F, IDCheck];
                    end
                    
                end
                
                if DRNew(1) == EndPoint(1) && DRNew(2) == EndPoint(2)
                    break
                end
                
            end

        
        
%% Move Up Right

[URNew] = MoveUpRight(CurrentNode);
[HP] = HalfPlane(URNew(1), URNew(2));
IDCheck = IDPoint(URNew);
       
        if HP == 0
               
            if (~any(IDCheck == NodesInfo(1,6,:))) 
                                  TempCost = NodesInfo(:,3,B);
                    CP = TempCost + 1;
                    H = (sqrt(((URNew(1) - EndPoint(1)) ^ 2)+((URNew(2) - EndPoint(2)) ^ 2)));
                    F = CP + H;
                    Nodes(:,:,A) = URNew;
                    NodesInfo(:,:,A) = [A, B, CP, H, F, IDCheck];
                    A = A + 1;                    
                    
                elseif  (~any(IDCheck == ClosedNodesInfo(1,1,:)))
                    N = find(IDCheck == NodesInfo(1,6,:));
                    cost = NodesInfo(1,5,N);
                    TempCost = NodesInfo(:,3,B);
                    CP = TempCost + 1;
                    H = sqrt(((URNew(1) - EndPoint(1)) ^ 2)+((URNew(2) - EndPoint(2)) ^ 2));
                    F = CP + H;
                
                    if cost>F
                        NodesInfo(:,:,N) = [A, B, CP, H, F, IDCheck];
                    end
                    
                end
                
                if URNew(1) == EndPoint(1) && URNew(2) == EndPoint(2)
                    break
                end
                
            end
            

%% Move Up Left
[ULNew] = MoveUpLeft(CurrentNode);
[HP] = HalfPlane(ULNew(1), ULNew(2));
IDCheck = IDPoint(ULNew);
       
        if HP == 0
               
            if (~any(IDCheck == NodesInfo(1,6,:))) 
                                    TempCost = NodesInfo(:,3,B);
                    CP = TempCost + 1;
                    H = (sqrt(((ULNew(1) - EndPoint(1)) ^ 2)+((ULNew(2) - EndPoint(2)) ^ 2)));
                    F = CP + H;
                    Nodes(:,:,A) = ULNew;
                    NodesInfo(:,:,A) = [A, B, CP, H, F, IDCheck];
                    A = A + 1;                    
                    
                elseif  (~any(IDCheck == ClosedNodesInfo(1,1,:)))
                    N = find(IDCheck == NodesInfo(1,6,:));
                    cost = NodesInfo(1,5,N);
                    TempCost = NodesInfo(:,3,B);
                    CP = TempCost + 1;
                    H = sqrt(((ULNew(1) - EndPoint(1)) ^ 2)+((ULNew(2) - EndPoint(2)) ^ 2));
                    F = CP + H;
                
                    if cost>F
                        NodesInfo(:,:,N) = [A, B, CP, H, F, IDCheck];
                    end
                    
                end
                
                if ULNew(1) == EndPoint(1) && ULNew(2) == EndPoint(2)
                    break
                end
                
            end
        
        
%% Finding Minimum Value 

        MinValue = [inf,0];
        
        for y = 1:A-1
        IDCheck = IDPoint(Nodes(:,:,y));
        
            if (~any(IDCheck == ClosedNodesInfo(1,1,:)))
                cost = NodesInfo(1,5,y);
            
                if cost<MinValue(1,1)
                    MinValue = [cost , y];
                end
                
            end
            
        end

        B = MinValue(1,2);
        C = C+1;
                
end

    text(StartX(1), StartY(1), 'Start')                          % To display Start and End at the designated Points in the Graph
    plot(StartX(1), StartY(1), 'x', 'MarkerEdgeColor', 'red')    % To display a Marker besides the Start and End Text
    
    text(EndX, EndY, 'End')
    plot(EndX, EndY, 'x', 'MarkerEdgeColor', 'red')
    
TempIndex = 0;
M = A - 1;

while M ~= 1      
    
    NodesInfo(:, :, M);
    P = Nodes(1, 1, M);
    Q = Nodes(1, 2, M); 
    
    info = NodesInfo(1,2,M);
    M = info;
    
    TempIndex = TempIndex + 1;
    scatter(P,Q, 2, 'blue')   
    pause(0.5)

end
    
else
    disp('Invalid Input. Please try again!')
end

toc
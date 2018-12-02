%% Project-3- A* in RRL Lab for TurtleBot2

clearvars
close all
clc

A = 1;                     % Used for denoting the New Node
B = 0;                     % Used for denoting the Parent Node
%% Starting and End Point Inputs from User

StartX(A) = input('Enter the X co-ordinate of Starting Point (0 - 100): ');
StartY(A) = input('Enter the Y co-ordinate of Starting Point (0 - 150): ');

StartPoint = [StartX(A), StartY(A)];

ParentNode(:,A) = B;                    % Storing the Value in a Parent Node starting from 0
B = 1;                                  % Assigning the Value of B as 1 to start the number of Parent Nodes

EndX = input('Enter the X co-ordinate of End Point (0 - 100): ');
EndY = input('Enter the Y co-ordinate of End Point (0 - 150): ');
EndPoint = [EndX, EndY];

% Since the value of starting point can be any of the co-ordinates,
% their values are stored in the form of loop run by the variable A

tic

%% Restrictions for User Input Points

[HP] = HalfPlaneTTB (StartX(1), StartY(1));     % Checking if the Entered Co-ordinate doesn't lie inside the Obstacle Space 
P = HP;

[HP] = HalfPlaneTTB (EndX, EndY);           % Checking if the Entered Co-ordinate doesn't lie inside the Obstacle Space 
Q = HP;

if ((P == 0) && (Q == 0))               % To ensure that the user has Entered Correct Co-ordinates of Start and End Points

    
%% Plotting of Graph and Objects

% Graph

plot (0, 0, 100, 150, '-s')           % Plotting the X and Y axis of the Graph
axis equal
grid on                                 % Used to depict Gridlines in the Graph


%% Floor Plan / Wall

rectangle('Position', [1.5 1.5 97 147], 'FaceColor', 'white', 'LineWidth', 1.0)    % Co-ordinates of One Vertex and the Length of Adjacent Sides
hold on


%% Customizable Tables

% Table-1

rectangle('Position', [20 1.5 20 8], 'FaceColor', 'red', 'LineWidth', 1.0)    % Co-ordinates of One Vertex and the Length of Adjacent Sides
hold on

% Table-2

rectangle('Position', [40 1.5 20 8], 'FaceColor', 'red', 'LineWidth', 1.0)    % Co-ordinates of One Vertex and the Length of Adjacent Sides
hold on

% Table-3

rectangle('Position', [72 113 20 8], 'FaceColor', 'red', 'LineWidth', 1.0)    % Co-ordinates of One Vertex and the Length of Adjacent Sides
hold on

% Table-4
rectangle('Position', [72 124.5 20 8], 'FaceColor', 'red', 'LineWidth', 1.0)    % Co-ordinates of One Vertex and the Length of Adjacent Sides
hold on
 

%% Dining Table

rectangle('Position', [84 15 11 16], 'FaceColor', 'yellow', 'LineWidth', 1.0)      % Co-ordinates of One Vertex and the Length of Adjacent Sides
hold on


%% Conference Tables

% Table-1

rectangle('Position', [32.5 40 32 16], 'Curvature', [0.5, 1], 'FaceColor', 'm', 'LineWidth', 1.0)
hold on

% Table-1

rectangle('Position', [32.5 80 32 16], 'Curvature', [0.5, 1], 'FaceColor', 'm', 'LineWidth', 1.0)
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
[HP] = HalfPlaneTTB(LNew(1), LNew(2));
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
[HP] = HalfPlaneTTB(DNew(1), DNew(2));
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
[HP] = HalfPlaneTTB(RNew(1), RNew(2));
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
[HP] = HalfPlaneTTB(UNew(1), UNew(2));
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
[HP] = HalfPlaneTTB(DLNew(1), DLNew(2));
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
[HP] = HalfPlaneTTB(DRNew(1), DRNew(2));
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
[HP] = HalfPlaneTTB(URNew(1), URNew(2));
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
[HP] = HalfPlaneTTB(ULNew(1), ULNew(2));
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
        
        
%% Finding Minimum Value out of all the Possible Positions

        MinValue = [inf,0];
        
        for y = 1:A-1
        IDCheck = IDPoint(Nodes(:,:,y));
        
            if (~any(IDCheck == ClosedNodesInfo(1,1,:)))
                cost = NodesInfo(1,5,y);
            
                if cost < MinValue(1,1)
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
O = 1;

while M ~= 1      
    
    NodesInfo(:, :, M);
    P = Nodes(1, 1, M);
    Q = Nodes(1, 2, M); 
    
    OptPath(:,:,O) = [P; Q];    % Creating Matrix about the Parent Nodes Numbers for Plotting in V-REP
    
    info = NodesInfo(1,2,M);
    M = info;
    
    TempIndex = TempIndex + 1;
    scatter(P, Q, 2, 'blue')   % Plotting in Graph
    pause(0.5)
    
    O = O + 1;
    
end
   
else
    disp('Invalid Input. Please try again!')
end

%%
%%
%%

%% Simulating the Path in V-REP

vrep = remApi('remoteApi');
vrep.simxFinish(-1);
clientID = vrep.simxStart('127.0.0.1', 19999, true, true, 5000, 5);
 
 
if (clientID>-1)
    disp('Connected to remote API server');
        

%% Transforming Co-ordinates according to the Robot's Orientation in V-REP

TT = [-45; -75];                % Translation Matrix

RR = [cosd(270) -sind(270)      % Rotation Matrix
      sind(270) cosd(270)];


  
[returnCode, Turtle] = vrep.simxGetObjectHandle (clientID, 'Turtlebot2', vrep.simx_opmode_blocking);
[returnCode, LeftWheel] = vrep.simxGetObjectHandle (clientID, 'wheel_left_joint', vrep.simx_opmode_blocking);
[returnCode, RightWheel] = vrep.simxGetObjectHandle (clientID, 'wheel_right_joint', vrep.simx_opmode_blocking);


fileID1 = fopen('Linear Velocities.txt','w');
fileID2 = fopen('Angular Velocities.txt','w');
fprintf(fileID1,'%5s %10s %15s \r\n','linear.x','linear.y', 'linear.z');
fprintf(fileID2,'%5s %10s %15s \r\n','angular.x','angular.y', 'angular.z');


for r = 0:(size(OptPath,3)-1)

    TempPath(:, :, r + 1) = [OptPath(1, 1, size(OptPath, 3)-r); OptPath(2, 1, size(OptPath, 3)-r)];
    TempPath2(:, :, r + 1) = TempPath(:, :, r + 1) + TT;
    TempPath3(:, :, r + 1) = RR * TempPath2(:, :, r + 1);
    
end

PathSize = length(TempPath3);
t =1;

for PS = 1: (PathSize)

    [returnCode, OrientationTheta] = vrep.simxGetObjectOrientation(clientID, Turtle, -1, vrep.simx_opmode_blocking);
    [returnCode, PositionXY] = vrep.simxGetObjectPosition(clientID, Turtle, -1, vrep.simx_opmode_blocking);
     
    if TempPath3(1, 1, PS) ~= PositionXY(1)
       PathPoints = ((TempPath3(2, 1, PS) - (PositionXY(2)*10)) / (TempPath3(1, 1, PS) - (PositionXY(1)*10)));
       Theta = atan(PathPoints);
    end
    
    if TempPath3(1,1,PS) == (PositionXY(1) *10)
       Theta = (pi)/2;
    end
     
    if Theta < 0
       Theta = Theta + pi;
    end
    
    if Theta ~= (OrientationTheta(2)+(pi/2))
        if (Theta - (OrientationTheta(2)+(pi/2))) > 0
            ThetaDiff = (Theta - (OrientationTheta(2)+(pi/2)));
            
            while ThetaDiff > .1
            
            [returnCode] = vrep.simxSetJointTargetVelocity(clientID, LeftWheel, -.5, vrep.simx_opmode_blocking);
            [returnCode] = vrep.simxSetJointTargetVelocity(clientID, RightWheel, .5, vrep.simx_opmode_blocking);
            
            [returnCode,OrientationTheta] = vrep.simxGetObjectOrientation(clientID, Turtle, -1, vrep.simx_opmode_blocking);
            ThetaDiff = (Theta - (OrientationTheta(2) + (pi/2)));
            
%% Initializing Velocities            

            [returnCode, LinearVelocity, AngularVelocity] = vrep.simxGetObjectVelocity(clientID, Turtle, vrep.simx_opmode_blocking)
            
            LinearX (:,t) = LinearVelocity(1);
            LinearY (:,t) = LinearVelocity(2);
            LinearZ (:,t) = LinearVelocity(3);
            
            AngularX (:,t) = AngularVelocity(1);
            AngularY (:,t) = AngularVelocity(2);
            AngularZ (:,t) = AngularVelocity(3);
            
            t = t+1;
            
            end 
            [returnCode] = vrep.simxSetJointTargetVelocity(clientID, LeftWheel, 0, vrep.simx_opmode_blocking);
            [returnCode] = vrep.simxSetJointTargetVelocity(clientID, RightWheel, 0, vrep.simx_opmode_blocking);
        
        end
        
        if (Theta - (OrientationTheta(2)+(pi/2))) < 0
            ThetaDiff = (Theta - (OrientationTheta(2)+(pi/2)));
            
             while ThetaDiff < -.1          % For finding the Orientation of the Bot
            
             [returnCode] = vrep.simxSetJointTargetVelocity(clientID, LeftWheel, .5, vrep.simx_opmode_blocking);
             [returnCode] = vrep.simxSetJointTargetVelocity(clientID, RightWheel, -.5, vrep.simx_opmode_blocking);
             [returnCode,OrientationTheta] = vrep.simxGetObjectOrientation(clientID, Turtle, -1, vrep.simx_opmode_blocking);
             ThetaDiff = (Theta - (OrientationTheta(2)+(pi/2)));
             
            [returnCode, LinearVelocity, AngularVelocity] = vrep.simxGetObjectVelocity(clientID, Turtle, vrep.simx_opmode_blocking);
            
            LinearX (:,t) = LinearVelocity(1);
            LinearY (:,t) = LinearVelocity(2);
            LinearZ (:,t) = LinearVelocity(3);
            
            AngularX (:,t) = AngularVelocity(1);
            AngularY (:,t) = AngularVelocity(2);
            AngularZ (:,t) = AngularVelocity(3);
             
             t = t+1;
             end
             
             [returnCode] = vrep.simxSetJointTargetVelocity(clientID, LeftWheel, 0, vrep.simx_opmode_blocking);
             [returnCode] = vrep.simxSetJointTargetVelocity(clientID, RightWheel, 0, vrep.simx_opmode_blocking);
        
        end
        
    end
    
    [returnCode, PositionXY]=vrep.simxGetObjectPosition(clientID, Turtle, -1, vrep.simx_opmode_blocking);
     distance = sqrt(((TempPath3(2,1,PS)) - (PositionXY(2)*10))^2 + ((TempPath3(1,1,PS) - (PositionXY(1)*10))^2));
     
     while distance > 8         % Threshold Value for stopping the bot
         
         [returnCode] = vrep.simxSetJointTargetVelocity(clientID, LeftWheel, 10, vrep.simx_opmode_blocking);
         [returnCode] = vrep.simxSetJointTargetVelocity(clientID, RightWheel, 10, vrep.simx_opmode_blocking);
         [returnCode, PositionXY] = vrep.simxGetObjectPosition(clientID, Turtle, -1, vrep.simx_opmode_blocking);
         distance = sqrt(((TempPath3(2,1,PS)) - (PositionXY(2)*10))^2 + ((TempPath3(1,1,PS) - (PositionXY(1)*10))^2));
         
         [returnCode,LinearVelocity,AngularVelocity] = vrep.simxGetObjectVelocity(clientID, Turtle, vrep.simx_opmode_blocking);
         
         LinearX (:,t) = LinearVelocity(1);
         LinearY (:,t) = LinearVelocity(2);
         LinearZ (:,t) = LinearVelocity(3);
            
         AngularX (:,t) = AngularVelocity(1);
         AngularY (:,t) = AngularVelocity(2);
         AngularZ (:,t) = AngularVelocity(3);
         
         t = t+1;
         
     end
      
     [returnCode] = vrep.simxSetJointTargetVelocity(clientID, LeftWheel, 0, vrep.simx_opmode_blocking);     % Stopping the Turtlebot by assigning Zero velocity
     [returnCode] = vrep.simxSetJointTargetVelocity(clientID, RightWheel, 0, vrep.simx_opmode_blocking);
end

fprintf(fileID1,'%5.5f %10.5f %15.5f \r\n', LinearX, LinearY, LinearZ);     % Creating Text file of Velocities
fprintf(fileID2,'%5.5f %10.5f %15.5f \r\n', AngularX, AngularY, AngularZ);


vrep.simxFinish(-1);

end
 
vrep.delete();

toc
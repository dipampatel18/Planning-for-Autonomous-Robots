function [HP] = HalfPlaneTTB (x, y)

% If the values of x & y satisfy any of the below conditions, then the
% point lies inside the Obstacle Space and is to be removed from the BFS Calculation
% This method is known as Half-Plane.

HP = 0;

%% Half-Plane for Floor

if [(x <= 1.5) && (x >= 98.5) && (y <= 1.5) && (y >= 148.5)]
   HP = HP + 1;
end

% Since the Robot here is not a point robot but has considerable dimensions, its radius
% is added to the dimensions of the obstacles, in order to consider the robot as point robot.
% Thus, 177m (17.7 cm) would be added to the dimensions of all the obstacles.

%% Half-Plane for Customizable Tables

% Tables-1 & 2

if [(x >= 18.23) && (x <= 61.77) && (y >= -.27) && (y <= 11.27)]
   HP = HP + 1;
end

% Table-3
    
if [(x >= 70.23) && (x <= 93.77) && (y >= 111.23) && (y <= 122.77)]
   HP = HP + 1;
end

% Table-4

if [(x >= 70.23) && (x <= 93.77) && (y >= 122.73) && (y <= 134.27)]
   HP = HP + 1;    
end


%% Half-Plane for Dining Table

if [(x >= 82.23) && (x <= 96.77) && (y >= 14.23) && (y <= 32.77)]
   HP = HP + 1;    
end

%% Half-Plane for Conference Tables

% Table-1

if [(x >= 37.73) && (x <= 57.27) && (y >= 38.23) && (y <= 57.77)]
   HP = HP + 1;    
end

if [((x-39.5)^2 + (y-48)^2) <= 95.4529]
   HP = HP + 1;
end 

if [((x-55.5)^2 + (y-48)^2) <= 95.4529]
   HP = HP + 1;
end 

% Table-2

if [(x >= 37.73) && (x <= 57.27) && (y >= 78.23) && (y <= 97.77)]
   HP = HP + 1;    
end

if [((x-55.5)^2 + (y-88)^2) <= 95.4529]
   HP = HP + 1;
end

if [((x-39.5)^2 + (y-88)^2) <= 95.4529]
   HP = HP + 1;
end

end

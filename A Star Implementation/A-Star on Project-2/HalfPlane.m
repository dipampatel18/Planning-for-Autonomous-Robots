function [HP] = HalfPlane (x, y)

% If the values of x & y satisfy any of the below conditions, then the
% point lies inside the Obstacle Space and is to be removed from the BFS Calculation
% This method is known as Half-Plane.

HP = 0;

%% Half-Plane for Rectangle

if [(x >= 55) && (x <= 105) && (y >= 67.5) && (y <= 112.5)]

   HP = HP + 1;

end

%% Half-Plane for Circle

if [((x-180)^2 + (y-120)^2) <= 225]

   HP = HP + 1;
    
end

%% Half-Plane for Polygon

% Dividing the Polygon into Two Quadrilaterals for Easier Calculations

% Quadrilateral-1

if [(y >= 4) && ((41*x) + (25*y) >= 6295) && ((4*x) + (38*y) <= 2570)...
        && ((37*x) + (10*y) <= 6356)]
    
   HP = HP + 1;    
    
end

% Quadrilateral-2

if [((37*x) + (10*y) >= 6356) && ((37*x) - (20*y) <= 5936) && ((38*x)...
        + (23*y) <= 8317) && ((38*x) - (7*y) >= 5647)]

   HP = HP + 1;

end

end
% Extracts mass, geometric, and inertia properties of a robotic arm
% Author: <Preksha Krishnan>

clc; clear all;
% Load the STL file
stlData = stlread('link1.STL');  

% Get the triangulated mesh
TR = triangulation(stlData.ConnectivityList, stlData.Points);

% Compute the volume
[volume, ~] = stlVolume(TR);  

density = 2700;  % aluminum
mass = volume * density;
fprintf('Estimated Mass: %.4f kg\n', mass);

function [vol, area] = stlVolume(TR)
    % Compute volume and surface area of STL mesh
    vol = 0;
    area = 0;
    faces = TR.ConnectivityList;
    vertices = TR.Points;

    for i = 1:size(faces,1)
        p1 = vertices(faces(i,1),:);
        p2 = vertices(faces(i,2),:);
        p3 = vertices(faces(i,3),:);
        
        % Calculate volume using divergence theorem
        v321 = p3(1)*p2(2)*p1(3);
        v231 = p2(1)*p3(2)*p1(3);
        v312 = p3(1)*p1(2)*p2(3);
        v132 = p1(1)*p3(2)*p2(3);
        v213 = p2(1)*p1(2)*p3(3);
        v123 = p1(1)*p2(2)*p3(3);
        vol = vol + (1.0/6.0)*(-v321 + v231 + v312 - v132 - v213 + v123);
        
        % Surface area (optional)
        area = area + 0.5 * norm(cross(p2 - p1, p3 - p1));
    end
    vol = abs(vol);  % ensure volume is positive
end

function [vertices,faces] = draw_cube(A)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
vertices = A.*[0 0 0; 0 1 0; 1 1 0; 1 0 0; 0 0 1; 0 1 1; 1 1 1; 1 0 1] - [A/2 A/2 0];
faces = [1 2 3 4; 2 6 7 3; 4 3 7 8; 1 5 8 4; 1 2 6 5; 5 6 7 8];

vertices = cart2hom(vertices);
end


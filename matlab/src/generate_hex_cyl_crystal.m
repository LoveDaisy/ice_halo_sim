function crst = generate_hex_cyl_crystal(axis_ori, roll, ratio)
% This function generates a crystal given input parameters
% INPUT
%   axis_ori:   [lon,lat], in degree
%   roll:       roll angle, in degree
%   ratio:      height / radii
% OUTPUT
%   crst:

assert(size(axis_ori,1) == size(roll,1));
assert(length(ratio) == 1);

% Vertex index:
%             3   2
% upper:   4         1
%             5   6
% 
%            9   8
% lower:  10        7
%           11   12

vtx = [cosd(0:60:359)', sind(0:60:359)', ones(6,1)*ratio/2;
    cosd(0:60:359)', sind(0:60:359)', -ones(6,1)*ratio/2];
faces = [1, 2, 3; 1, 3, 4; 4, 5, 6; 4, 6, 1;
    1, 7, 8; 1, 8, 2; 2, 8, 9; 2, 9, 3; 3, 9, 10; 3, 10, 4;
    4, 10, 11; 4, 11, 5; 5, 11, 12; 5, 12, 6; 6, 12, 7; 6, 7, 1;
    7, 9, 8; 7, 10, 9; 10, 7, 11; 11, 7, 12];
crst = generate_crystal(vtx, faces, axis_ori, roll);

end


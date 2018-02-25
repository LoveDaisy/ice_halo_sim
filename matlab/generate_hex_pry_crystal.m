function crst = generate_hex_pry_crystal(axis_ori, roll, ratio)
% This function generates a crystal given input parameters
% INPUT
%   axis_ori:   [lon,lat], in degree
%   roll:       roll angle, in degree
%   ratio:      [height1, height2, height3] / radii, from top to bottom
% OUTPUT
%   crst:

assert(size(axis_ori,1) == size(roll,1));
assert(length(ratio) == 3);
ratio([1,3]) = min(ratio([1,3]), 1.6288);

% Vertex index:
%            3   2
% top:    4         1
%            5   6
% 
%            9   8
% upper:  10        7
%           11   12
%
%            15   14
% lower:  16         13
%            17   18
%
%            21  20
% bottom: 22         19
%            23  24

q = 31.55;
vtx = [cosd(0:60:359)'*(1-ratio(1)*tand(q)), sind(0:60:359)'*(1-ratio(1)*tand(q)), ones(6,1)*(ratio(2)/2+ratio(1));
    cosd(0:60:359)', sind(0:60:359)', ones(6,1)*ratio(2)/2;
    cosd(0:60:359)', sind(0:60:359)', -ones(6,1)*ratio(2)/2;
    cosd(0:60:359)'*(1-ratio(3)*tand(q)), sind(0:60:359)'*(1-ratio(3)*tand(q)), -ones(6,1)*(ratio(2)/2+ratio(3))];
f1 = [1, 2, 3; 1, 3, 4; 4, 5, 6; 4, 6, 1];
f2 = [1, 7, 8; 1, 8, 2; 2, 8, 9; 2, 9, 3; 3, 9, 10; 3, 10, 4;
    4, 10, 11; 4, 11, 5; 5, 11, 12; 5, 12, 6; 6, 12, 7; 6, 7, 1];
faces = [f1; f2; f2+6; f2+12; f1+18];
crst = generate_crystal(vtx, faces, axis_ori, roll);

end


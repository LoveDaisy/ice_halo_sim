function crst = generate_crystal(axis_ori, roll, ratio)
% This function generates a crystal given input parameters
% INPUT
%   axis_ori:   [lon,lat], in degree
%   roll:       roll angle, in degree
%   ratio:      height / diameter
%   n:          fractive index
% OUTPUT
%   crst:       .local_axis, echo row is a local axes
%               .normals
%               .areas
%               .axis_ori
%               .roll
%               .ratio

crst.axis_ori = axis_ori;
crst.roll = roll;
crst.ratio = ratio;

z = [cosd(axis_ori(2)) * cosd(axis_ori(1)), ...
    cosd(axis_ori(2)) * sind(axis_ori(1)), ...
    sind(axis_ori(2))];
zz = sqrt(1 - z(3)^2);
x = [-z(2), z(1), 0] / zz;
y = [-z(1) * z(3), -z(2) * z(3), 1 - z(3)^2] / zz;
local_axis = [x; y; z];

matR = axis_angle_to_matrix(z, roll);

crst.local_axis = local_axis * matR';
crst.normals = [sqrt(3)/2, 0.5, 0; 0, 1, 0; -sqrt(3)/2, 0.5, 0;
    -sqrt(3)/2, -0.5, 0; 0, -1, 0; sqrt(3)/2, -0.5, 0; 0, 0, 1; 0, 0, -1;] * matR';
crst.areas = [ratio*ones(6, 1); 3*sqrt(3); 3*sqrt(3)];
end


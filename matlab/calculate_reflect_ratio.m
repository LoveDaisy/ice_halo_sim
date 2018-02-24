function [Rs, Rp] = calculate_reflect_ratio(inc_angle, n1, n2)
c = cosd(inc_angle);
s = sind(inc_angle);

d = max(1 - ((n1 ./ n2) .* s).^2, 0);

Rs = ((n1 .* c - n2 .* sqrt(d)) ./ (n1 .* c + n2 .* sqrt(d))).^2;
Rp = ((n1 .* sqrt(d) - n2 .* c) ./ (n1 .* sqrt(d) + n2 .* c)).^2;
end
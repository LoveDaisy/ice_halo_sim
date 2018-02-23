function matR = axis_angle_to_matrix(u, theta)
c = cosd(theta); s = sind(theta);
matR = [c + u(1)^2*(1-c), u(1)*u(2)*(1-c) - u(3)*s, u(1)*u(3)*(1-c) + u(2)*s;
        u(2)*u(1)*(1-c) + u(3)*s, c + u(2)^2*(1-c), u(2)*u(3)*(1-c) - u(1)*s;
        u(3)*u(1)*(1-c) - u(2)*s, u(3)*u(2)*(1-c) + u(1)*s, c + u(3)^2*(1-c)];
end
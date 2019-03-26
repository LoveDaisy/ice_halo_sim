function n = shellmeier2(coef, wavelength)
% The two-term Sellmeier equation:
% n^2 = 1 + B1 * lambda^2 / (lambda^2 - C1)
%         + B2 * lambda^2 / (lambda^2 - C2)
%
% INPUT
%  coef:        [B1, C1*1e-2; B2, C2*1e2]
%  wavelength:  1xn row, in nm
% OUTPUT
%  n:           1xn row

wavelength = wavelength(:)' / 1000;
coef = coef .* [1, 1e-2; 1, 1e2];
n = sqrt(sum(bsxfun(@times, coef(:,1), ...
    1./(1 - bsxfun(@times, coef(:,2), 1./wavelength.^2)))) + 1);
end
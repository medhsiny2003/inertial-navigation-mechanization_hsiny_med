% ============================================================
% INS_Mechanization - Analysis Scripts
% Author: Mohammed Hsiny
% Based on the original project by GYH-WHU
% Year: 2025
% ============================================================

function r = rmse(estimated, truth)
    if nargin == 1
        % Single argument: root mean square of error array
        r = sqrt(mean(estimated.^2));
    else
        % Two arguments: root mean square error between estimated and truth
        r = sqrt(mean((estimated - truth).^2));
    end
end

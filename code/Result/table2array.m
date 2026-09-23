% ============================================================
% INS_Mechanization - Analysis Scripts
% Author: Mohammed Hsiny
% Based on the original project by GYH-WHU
% Year: 2025
% ============================================================

function A = table2array(X)
    if ismatrix(X) && isnumeric(X)
        A = X;
    elseif iscell(X)
        A = cell2mat(X);
    else
        A = double(X);
    end
end

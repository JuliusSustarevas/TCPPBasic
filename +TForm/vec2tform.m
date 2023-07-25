function T = vec2tform(vec,T)
%This function converts from nx7 (xyz wxyz) to tform 4x4xn
if ~exist('T')
    T = zeros(4, 4, size(vec, 1));
end 
    T = quat2tform(vec(:, 4:end));    
    T(1:3, 4, :) = vec(:, 1:3)';
end

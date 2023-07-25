function T = tform2inv(T)
    %This inverses all Tforms in a 4d array
    for ii = 1:size(T, 3)
        %         R = T(1:3,1:3,ii)';
        %             p = -R*T(1:3,4);
        %             Tinv = [R,    p; ...
        %                 [0 0 0 1]];
        T(:, :, ii) = [T(1:3, 1:3, ii)', -T(1:3, 1:3, ii)' * T(1:3, 4, ii); T(4, :, ii)];
    end

end

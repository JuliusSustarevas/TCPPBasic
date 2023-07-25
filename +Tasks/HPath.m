classdef HPath < Tasks.Path

    methods

        function self = HPath(order, nn)
            % order - order of self repetition
            % nn - num points

            A = zeros(1, 3);
            B = zeros(1, 3);
            C = zeros(1, 3);
            D = zeros(1, 3);

            north = [0 1 0];
            east = [1 0 0];
            south = [0 -1 0];
            west = [-1 0 0];

            for n = 1:order
                AA = [B; north; A; east; A; south; C];
                BB = [A; east; B; north; B; west; D];
                CC = [D; west; C; south; C; east; A];
                DD = [C; south; D; west; D; north; B];

                A = AA;
                B = BB;
                C = CC;
                D = DD;
            end

            A = [0 0 0; cumsum(A)];
            curve = 1 * A ./ max(A);
            curve(isnan(curve)) = 0;
            curve = interp1(linspace(0, 1, size(curve, 1)), curve, linspace(0, 1, nn), 'pchip');
            self = self@Tasks.Path(curve);
        end

    end

end

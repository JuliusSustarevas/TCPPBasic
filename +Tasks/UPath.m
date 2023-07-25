classdef UPath < Tasks.Path

    methods

        function self = UPath(sep_ratio)
            x1 = (1 - sep_ratio) / 2;
            x2 = ((1 - sep_ratio) / 2) + sep_ratio;
            s=sep_ratio/2;
            pattern = [0 -1 0; x1 -1 0; x1 0 0; s 0 0; s 1 0; 1-s 1 0; 1-s 0 0; x2 0 0; x2 -1 0; 1 -1 0];
            pattern = interp1(linspace(0, 1, length(pattern)), pattern, linspace(0, 1, 50));
            self = self@Tasks.Path(pattern);
        end

    end

end

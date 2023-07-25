classdef XHatch2Path < Tasks.Path

    methods

        function self = XHatch2Path(path_width,nozzle_width)            
            pattern = [0 -1 0; 1 1 0; 2 1 0; 1 -1 0; 2 -1 0].*[nozzle_width,path_width 0];
            pattern = interp1(linspace(0, 1, length(pattern)), pattern, linspace(0, 1, 100));
            self = self@Tasks.Path(pattern);
            self.smooth(0.01);
        end

    end

end

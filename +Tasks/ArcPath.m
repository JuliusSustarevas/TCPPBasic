classdef ArcPath < Tasks.Path

    methods

        function self = ArcPath(angle)
            t = linspace(0, 1, 100)';
            path = [cos(angle * t) sin(angle * t) zeros(100, 1)];         
            self = self@Tasks.Path(path);
        end

    end

end

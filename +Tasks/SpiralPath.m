classdef SpiralPath < Tasks.Path

    methods

        function self = SpiralPath(rotations, layer_height)
            t = linspace(0, rotations * 2 * pi, rotations * 100)';
            path = [cos(t) sin(t) ((t ./ t(end)) * layer_height * rotations)];
            self = self@Tasks.Path(path);
        end

    end

end

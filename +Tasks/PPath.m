classdef PPath < Tasks.Path

    methods

        function self = PPath()
            angle=pi;
            t = linspace(0, 1, 100)';
            path = [cos(angle * t) sin(angle * t) zeros(100, 1)];         
            path =[path; -1 -.08 0 ; 5 -.08 0 ] ;            
            self = self@Tasks.Path(path);
        end

    end

end

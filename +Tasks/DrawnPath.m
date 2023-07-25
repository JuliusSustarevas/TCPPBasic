classdef DrawnPath < Tasks.Path

    methods

        function self = DrawnPath()            
            [x,y]=ginput;
            n=size(x,1);
            z=zeros(n,1);            
            path=interp1(1:n,[x,y,z],linspace(1,n,100));
            self = self@Tasks.Path(path);
            self.smooth(0.075);
            %
            pattern = Tasks.UPath(.1);
            pattern.scale([0.2 0.05 1]);
            self.resample(0.001);         
            self.superimpose(pattern) 
            self.resample(0.01);
            % end
            self.traj=TForm.tformX(self.toTForm(self),TForm.DOWN());
            self.s=self.gett(1);
        end

    end

end

classdef MultiLayerDrawnTask < Tasks.Path

    methods

        function self = MultiLayerDrawnTask(layers,layer_dz)            
            [x,y]=ginput;
            n=size(x,1);
            z=zeros(n,1);            
            path=interp1(1:n,[x,y,z],linspace(1,n,100));
            self = self@Tasks.Path(path);
            self.smooth(0.075);
            %
            pattern = Tasks.UPath(.015);
            pattern.scale([0.1 0.01 1]);
%             pattern.scale([0.2 0.05 1]);
            self.resample(0.001);         
            self.superimpose(pattern) 
            self.resample(0.005);
            % end            
            new_path=self.path;
            for ii=1:layers-1
                self.path=flipud(self.path);
                self.path(:,3)=self.path(:,3)+layer_dz;
                new_path=[new_path; self.path];
            end
            self.path=new_path;
            
            self.traj=TForm.tformX(self.toTForm(self),TForm.DOWN());
            td=TForm.DOWN();
            for ii=size(self.traj,3)
               self.traj(1:3,1:3,ii)=td(1:3,1:3); 
            end            
            self.s=self.gett(1);
        end

    end

end

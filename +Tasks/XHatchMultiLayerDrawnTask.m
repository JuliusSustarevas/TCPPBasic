classdef XHatchMultiLayerDrawnTask < Tasks.Path

    methods

        function self = XHatchMultiLayerDrawnTask(layers,layer_dz,width,nozzle_width)            
            [x,y]=ginput;
            n=size(x,1);
            z=zeros(n,1);            
            path=interp1(1:n,[x,y,z],linspace(1,n,100));
            
            
            path1 = Tasks.GenericPath(path);            
            path1.smooth(0.075);
            %
            pattern1 = Tasks.XHatch1Path(width,nozzle_width);            
            path1.resample(0.00005);            
            path1.superimpose(pattern1)             
            path1.resample(0.0025);            
            path2 = Tasks.GenericPath(flipud(path));           
            path2.smooth(0.075);
            %
            pattern2 = Tasks.XHatch2Path(width,nozzle_width);            
            path2.resample(0.00005);         
            path2.superimpose(pattern2)             
            path2.resample(0.0025);
            
            paths=[path1; path2];
            my_path=[];
            z=0;
            for ii=1:layers
                my_path=[my_path; (paths(1+mod(ii,2)).path + [0 0 z])];
                z=z+layer_dz;                
            end
            
            self = self@Tasks.Path(my_path);          
            self.traj=TForm.tformX(self.toTForm(self),TForm.DOWN());
            td=TForm.DOWN();
            for ii=size(self.traj,3)
               self.traj(1:3,1:3,ii)=td(1:3,1:3); 
            end            
            self.s=self.gett(1);
        end

    end

end

classdef AsRobot < handle
    
    % This just generates some points inside Armstone Bounding box. These
    % points will be checked in an occupancy grid. 
    
    properties                
        rbbr% robot bounding box radius
        rcp% robot collision points 16x2. wrt base link        
    end    

    methods

        function self = AsRobot()
            bb=[0.8+0.15 0.62+0.15]./2;%Added plus to push it away bit more 
            res=0.03;
            n1=round(bb(1)*2/res);
            pts1=[linspace(-bb(1),bb(1),n1)', -ones(n1,1)*bb(2)];
            pts2=[linspace(-bb(1),bb(1),n1)', ones(n1,1)*bb(2)];
            n2=round(bb(2)*2/res)-2;
            pts3=[ -ones(n2,1)*bb(1) , linspace(-bb(2)+res,bb(2)-res,n2)' ];            
            pts4=[ ones(n2,1)*bb(1) , linspace(-bb(2)+res,bb(2)-res,n2)' ]; 
            self.rcp=[pts1;pts2;pts3;pts4];
            self.rcp=[self.rcp zeros(size(self.rcp,1),1) ones(size(self.rcp,1),1)];            
            self.rbbr=norm(bb);
            
        end
        
        function plot(self)
            plot(self.rcp(:,1),self.rcp(:,2),'.',"MarkerSize",15);
            axis equal
        end

    end

end

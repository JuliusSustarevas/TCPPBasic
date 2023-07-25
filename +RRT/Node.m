classdef Node < handle
    
    %This class defines thee statespace where the underlying RRT of TCPP
    %takes place. The fun bit about this node is that it is node in BOTH
    %RRT tree as well as KDTree. This allows us todo quite nice NN search
    %without duplicating data. 

    properties
        pose % x y cos th, sin th, s
        %
        rrt_parent
        costs
        %
        KDT_Lchild
        KDT_Rchild
        KDT_linked_list
        % IRM
        bools
        %
        edge_to_parent
        node_arrow
    end

    properties (Constant)
        weights = [1 1 0.3 0.3 0.1];
    end

    methods

        function self = Node(x, y, th, s)

            if nargin == 0
                return
            end

            self.pose = [x, y, cos(th), sin(th), s];
        end

        function d = dist(self, other)           
            %from other to self. like a minus
            %distance = self.cart_weight* sqrt(sum((self.pose7(1:3)-other.pose7(:,1:3)).^2,2)) + self.rot_weight*acos(abs(self.pose7(4:7)*other.pose7(:,4:7)'));
            m = self.pose - other.pose;
%             global s_w
%             weightss=[self.weights(1:end-1) s_w];
            if m(5) <= 0
                d = inf;
            else
%                 d = sqrt(sum((m.^2) .* weightss, 2));
                d = sqrt(sum((m.^2) .* self.weights, 2));
            end

        end

        function T = toTform(self)
            c = self.pose(3); s = self.pose(4);
            T = [c -s 0 self.pose(1); s c 0 self.pose(2); 0 0 1 0; 0 0 0 1];
        end
        
        function fake_nodes=to6FakesAround(self)
            c=0.03;%size of increment
            xyths=self.toVec4()';
            xyths=xyths(1:3);
            s=self.pose(5);            
            rotm=eul2rotm([xyths(3) 0 0]);
            fwdxy=xyths+c*rotm*[1 0 0]';
            bcky=xyths+c*rotm*[-1 0 0]';
            leftxy=xyths+c*rotm*[0 1 0]';
            rightxy=xyths+c*rotm*[0 -1 0]';
            fake_nodes=[
                RRT.Node(fwdxy(1), fwdxy(2),xyths(3), s);
                RRT.Node(bcky(1), bcky(2),xyths(3), s);
                RRT.Node(leftxy(1), leftxy(2),xyths(3), s);
                RRT.Node(rightxy(1), rightxy(2),xyths(3), s);
                RRT.Node(xyths(1), xyths(2),xyths(3)+c, s);
                RRT.Node(xyths(1), xyths(2),xyths(3)-c, s)];
        end

        function a = ang(self)
            a = atan2(self.pose(4), self.pose(3));
        end

        function vec = toVec4(self)
            vec = [self.pose(1:2) self.ang() self.pose(5)];
        end

        function draw(self)

            if isempty(self.node_arrow)                
                self.node_arrow = quiver(self.pose(1), self.pose(2), self.pose(3), self.pose(4), 0.1,'b');
                drawnow
            end

        end

        function nn=to_kdlist(self)

            num_retrieved=0;
            nnc=self.KDT_linked_list;
            while ~isempty(nnc)
                num_retrieved=num_retrieved+1;
                nnc=nnc.KDT_linked_list;
            end
            
            if num_retrieved==0
                nn=[];
                return
            end
            nn(num_retrieved)=RRT.Node();
            nnc=self.KDT_linked_list;
            for ii=1:num_retrieved
                temp=nnc;
                nn(ii)=temp;                
                nnc=temp.KDT_linked_list;
                temp.KDT_linked_list=[];
            end
           self.KDT_linked_list=[];

        end

    end

end

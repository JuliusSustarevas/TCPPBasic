classdef RRT < handle

    properties
        kd_tree
        %
        scene
        %
        max_s_node
        %% Default constants:
        e_inc = 0.03;
        e_reach = 0.2;
        e_goal = 1;
        start_rad = 0.5;
        s_var = 0.1
        num_start_nodes = 50;
        draw = false;
    end

    methods

        function self = RRT(scene)
            self.scene = scene;
            
        end

        function [goal, reached] = is_goal_reached(self, node)            
            gn = RRT.Node(node.pose(1), node.pose(2), node.ang(), self.scene.task_smax);            
            if self.scene.isValid(gn)
                if node.pose(5)~=self.scene.task_smax                    
                    [goal, reached] = self.extend(node, gn);
                else
                    goal=node; reached=true;
                end
            else
                goal=[];reached= false;
            end            
        end

        function s = sample_s(self)
            smax = self.scene.task_smax;
            s = self.max_s_node.pose(5) + randn(1) * self.s_var * smax;
            s = min(max(s, 0), smax);
        end

        function [n_new, reach] = extend(self, n_near, n_rand)
            % n_new - new node or empty. empty if any extend has failed
            % reach true if  n_rand is reached
            %entry tests:
            assert(self.scene.isValid(n_rand))
            %
            reach = false;            
            diff_vec4 = [n_rand.pose(1:2) - n_near.pose(1:2) angdiff(n_near.ang(), n_rand.ang()) n_rand.pose(5) - n_near.pose(5)];

            assert(diff_vec4(4) > 0);
            e_vec4 = self.e_inc * diff_vec4 ./ norm(diff_vec4);
            vec_last = n_near.toVec4();
            n_last = RRT.Node(vec_last(1), vec_last(2), vec_last(3), vec_last(4));

            while n_last.dist(n_near) < self.e_reach || all(abs(n_last.pose - n_near.pose)<0.001) %allow nlast-nnear

                vec_new = vec_last + e_vec4;
                n_new = RRT.Node(vec_new(1), vec_new(2), vec_new(3), vec_new(4));

%                 if ~self.scene.isValid(n_new)
%                     break
%                 end
                
                if ~self.scene.isValid7(n_new)
                    break
                end

                if isinf(n_rand.dist(n_new)) %went past n_rand. means n_last has essentially reached n_rand
                    n_last = n_rand;
                    reach = true;
                    break
                end

                n_last = n_new;
                vec_last = vec_new;
            end

            %case n_new = n_near
            if all(abs(n_last.pose - n_near.pose)<0.001)
                n_new = [];
                reach=false;
            else
                n_new = n_last;
                assert(~isempty(n_new.bools))
            end

        end

        function rewire(self, n_new)
            temp=self.e_reach;
            self.e_reach=self.start_rad+0.01;

            [ns, d] = self.kd_tree.NNRad(n_new,self.start_rad);

            for ii = 1:length(ns)                
%                 assert((ns(ii).dist(n_new) == d(ii)) && d(ii) < self.start_rad)
                disp("rewire_000")
                dd = d(ii) + n_new.costs;
                if dd < ns(ii).costs
                    [~, reached] = self.extend(n_new, ns(ii));
                else
                    continue
                end               
                
                if reached

                    if self.draw
                        delete(ns(ii).edge_to_parent);
                    end

                    self.add_edge(n_new, ns(ii));
                    disp("rewired_111 ---")
                end

            end
            self.e_reach=temp;
        end

        function path = trace_path(self, end_node)
            node = end_node;
            path = [];
            

            while ~isempty(node.rrt_parent)
                if any(node==path)
                    display("tracing path has a loop!!")
                    return
                end
                path = [path; node];
                node = node.rrt_parent;
            end
            path = [path; node];
            path=flipud(path);
            
        end
        
        function plot_path(rrt,path,varargin)            
            if ~isempty(varargin)
                c=varargin{1};
            else
                c=[0.1, 0.1,0.9];
            end
            for ii=2:length(path)
                line([path(ii).rrt_parent.pose(1) path(ii).pose(1)], [path(ii).rrt_parent.pose(2) path(ii).pose(2)],'LineWidth',3,'Color',c);
            end
        end
        
        

        function add_edge(self, parent, child)
            child.rrt_parent = parent;
            child.costs = parent.costs + child.dist(parent);

            if self.draw
                if ~isempty(child.edge_to_parent)
                    delete(child.edge_to_parent)
                end
                child.draw();
                child.edge_to_parent = line([parent.pose(1) child.pose(1)], [parent.pose(2) child.pose(2)]);
            end

        end

        function path = rrtstar(self)            

            self.kd_tree = RRT.KDTree();

            start_nodes = self.scene.sample_valid_irm(0, self.num_start_nodes);

            for ii = 1:self.num_start_nodes
                start_nodes(ii).costs=0;
                self.kd_tree.insert(start_nodes(ii));
                if self.draw
                    start_nodes(ii).draw();
                end
            end

            goal_reached = false;
            self.max_s_node = start_nodes(1);

            while ~goal_reached
                %% initial sampling and extend
                display(self.max_s_node.pose(5)/self.scene.task_smax)
                if rand()<0.05 
                    n_rand=self.scene.sample_from_node(self.max_s_node);%propage last node. 
                    if isempty(n_rand)
                        continue
                    end
                else    
                    s = self.sample_s();
                    n_rand = self.scene.sample_valid_irm(s, 1);
                end
                [n_near,~] = self.kd_tree.NN(n_rand);
                
                if isempty(n_near)
                    assert(~n_rand.pose(5))
                    continue
                end
                [n_new, reach] = self.extend(n_near, n_rand);                

                if isempty(n_new)
                    continue
                end
                
              
                %% add edge
                self.add_edge(n_near, n_new);               
                self.kd_tree.insert(n_new);               
                %% update smax
                if self.max_s_node.pose(5)<n_new.pose(5)
                    self.max_s_node=n_new;
                end
                %% goal check
                if abs(self.scene.task_smax - n_new.pose(5)) < self.e_goal
                    [goal, reached] = self.is_goal_reached(n_new);

                    if reached
                        if ~all(goal.pose==n_new.pose)                        
                            goal.rrt_parent = n_new;
                            goal.costs = n_new.costs + goal.dist(n_new);
                            self.kd_tree.insert(goal);
                        end                        
                        break
                    end

                end

                %%  rewire               
                self.rewire(n_new);
                
            end    
            
            path = self.trace_path(goal);           
        end
        
        
        function extra_samples(self,m)            
                nn=self.kd_tree.size()*m;
                
            for ii=1:nn
                %% initial sampling and extend
                display(round(ii/nn,2))    
                
                s = rand()*self.scene.task.s(end);
                n_rand = self.scene.sample_valid_irm(s, 1);   
                
                [n_near,~] = self.kd_tree.NN(n_rand);  
                if isempty(n_near)
                    assert(~n_rand.pose(5))
                    continue
                end
                [n_new, reach] = self.extend(n_near, n_rand);                

                if isempty(n_new)
                    continue
                end
                
              
                %% add edge
                self.add_edge(n_near, n_new);               
                self.kd_tree.insert(n_new);                       
                self.rewire(n_new);
                
            end            
                
        end
        
        function elipses = basepoints2elipses_local(self,basepoints)
            elipses=zeros(length(basepoints),3);

            
            for kk =1:length(basepoints)
                basepoint=[basepoints(kk,1:2) 0  basepoints(kk,3)];
                s=basepoints(kk,4);                
                taskpoint = self.scene.task.path(self.scene.task.s2index(s), :); 


                lb = [(basepoint(1:3) - taskpoint) basepoint(4)];            
                axes_limits = zeros(3, 1);
                axes_vecs=[1 0 0; 0 1 0]';                
                axes_vecs=eul2rotm([basepoint(4) 0 0])*axes_vecs;
                axes_vecs=[[axes_vecs' zeros(2,1)]; 0 0 0 1];

                for ii = 1:3

                    for jj = 1:10                       
                        testpoint = lb;
                        testpoint = testpoint + axes_vecs(ii,:)*0.025* jj;
                        fake_node= RRT.Node(testpoint(1)+taskpoint(1), testpoint(2)+taskpoint(2), testpoint(4), s);
                        

                        if ~self.scene.isValid(fake_node)
                            break
                        end

                        testpoint = lb;
                        testpoint = testpoint - axes_vecs(ii,:)*0.025* jj;
                        fake_node= RRT.Node(testpoint(1)+taskpoint(1), testpoint(2)+taskpoint(2), testpoint(4), s);
                        
                        if ~self.scene.isValid(fake_node)
                            break
                        end

                    end
                    if jj==1
                        disp("this means there is no leeway - should not happen");
                    end
                    axes_limits(ii) = 0.025 * (jj - 1);
                end
                elipses(kk,:)=axes_limits;                
                if self.draw && rand()<0.2 %every fith
                    qs=Visualization.draw_full_elipse(basepoint,axes_limits);
                    drawnow
                end
                
            end


        end
        
        function elipses = basepoints2elipses_global(self,basepoints)
            elipses=zeros(length(basepoints),3);

            
            for kk =1:length(basepoints)
                basepoint=[basepoints(kk,1:2) 0  basepoints(kk,3)];
                s=basepoints(kk,4);                
                taskpoint = self.scene.task.path(self.scene.task.s2index(s), :); 


                lb = [(basepoint(1:3) - taskpoint) basepoint(4)];            
                axes_limits = zeros(3, 1);
                axes_vecs=[1 0 0 0; 0 1 0 0; 0 0 0 1 ];                

                for ii = 1:3

                    for jj = 1:10                       
                        testpoint = lb;
                        testpoint = testpoint + axes_vecs(ii,:)*0.025* jj;
                        fake_node= RRT.Node(testpoint(1)+taskpoint(1), testpoint(2)+taskpoint(2), testpoint(4), s);
                        

                        if ~self.scene.isValid(fake_node)
                            break
                        end

                        testpoint = lb;
                        testpoint = testpoint - axes_vecs(ii,:)*0.025* jj;
                        fake_node= RRT.Node(testpoint(1)+taskpoint(1), testpoint(2)+taskpoint(2), testpoint(4), s);
                        
                        if ~self.scene.isValid(fake_node)
                            break
                        end

                    end
                    if jj==1
                        disp("this means there is no leeway - should not happen");
                    end
                    axes_limits(ii) = 0.025 * (jj - 1);
                end
                elipses(kk,:)=axes_limits;                
                if self.draw && rand()<0.9 %every fith
                    qs=Visualization.draw_full_elipse_global(basepoint,axes_limits);
                    drawnow
                end
                
            end


        end

    end

end

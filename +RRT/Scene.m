classdef Scene < handle
    
    % This class is what the RRT uses to Validate and Sample from
    % environment. All thee validity/sampling includes RM/IRM. Note that an
    % occupancy grid is used for collisions. 

    properties
        robot % robot params
        IRM
        map
        task % tasl
        task_smax
        TaskSaturation
        % properties to set. 
        iriThreshold = 0.33;
        num_layers
    end    

    methods

        function self = Scene(task_fn, IRM, robot)
            %% Map
            task_=load(task_fn).task;    
            self.num_layers=task_.num_layers;
            env_data = Imports.ReadYaml(strcat(task_.map_path, task_.map_fn));
            image = imread(strcat(task_.map_path, env_data.image));
            imageNorm = double(image) / 255;
            imageOccupancy = 1 - imageNorm;
            

            imageOccupancy(imageOccupancy < env_data.free_thresh) = 0;
            imageOccupancy(imageOccupancy > env_data.free_thresh) = 1;
            imageOccupancy=imdilate(imageOccupancy,ones(3));%Dialation
            self.TaskSaturation = [0.1 0.9];
            self.map = occupancyMap(imageOccupancy, (1 / env_data.resolution));
            self.map.FreeThreshold = 0;
            self.map.OccupiedThreshold = 1;
            self.map.GridLocationInWorld=env_data.origin(1:2);

            %% task
            task=task_.obj;
            tn = size(task.path, 1);
            sat_vec=self.TaskSaturation(1) + (1 - (1:tn) ./ tn) .* (self.TaskSaturation(2) - self.TaskSaturation(1));
            for ii=1:size(task.path,1)
                if  self.map.getOccupancy(squeeze(task.path(ii, 1:2)))<sat_vec(ii)
                    self.map.setOccupancy(squeeze(task.path(ii, 1:2)), sat_vec(ii)); % set task occupancy
                end
            end            
            self.task = task;
            hold off
            show(self.map)
            hold on
            self.task.plot();            
            self.task_smax = self.task.s(end);
            %% IRM
            self.IRM = IRM;
            %% Robot
            self.robot = robot;

        end
        
        function valid = isValid7(self, node)
%             is valid if +- 0.025 in 3 directions is also valid
%               This means 7 validity tests
            fakes=node.to6FakesAround();
            valid=self.isValid(node);
            for ii=1:6
                if ~valid
                    return
                end
                valid=valid && ~self.isInCollision(fakes(ii));
%                 valid=valid && self.isValid(fakes(ii));
            end         

        end
        

        function valid = isValid(self, node)
            valid = false;         
            if self.isInCollision(node)
%                 display("c")
                return
            end
            %% Check IRM
            if ~self.isIRM(node)
%                 display("i")
                return
            end

            valid = true;

        end

        function inirm = isIRM(self, node)
            inirm = false;
            ground_level=0;
            s=node.pose(5);
            %% Check IRM
            ii = self.task.s2index(s);            
            vec = self.task.traj(1:3, 3, ii);
            if isempty(node.bools)                
                vec4=node.toVec4(); 
                task_point = self.task.path(ii, :); 
                node.bools=self.IRM.scenePoint32Bools(task_point,ground_level, vec4(1:3));                                
            end
            iri = self.IRM.validate_basepoint([0 0 0  node.ang()], node.bools, vec);% this function doesnt use the  xyz since 

            if iri < self.iriThreshold
                return
            end

            inirm = true;

        end

        function inCollision = isInCollision(self, node)
            inCollision = true;
            %% check Collision
            cpoints = (node.toTform() * (self.robot.rcp'))';
            freeThres= self.TaskSaturation(1) + (1 - (node.pose(5) / self.task_smax)) .* (self.TaskSaturation(2) - self.TaskSaturation(1));            
            
            occ = self.map.getOccupancy(cpoints(:,1:2));

            if any(occ > freeThres)
                return
            end

            inCollision = false;

        end

        function samples = sample_irm(self, s, n)
            ground_level = 0;
            ii = self.task.s2index(s);
            point = self.task.path(ii, :);            
            vec = self.task.traj(1:3, 3,ii);
            nn = n * 2;% A way to increse chance that we get valid samples in the directed graph
            %TODO remove
            nn=1;
            [points4, bools, ~] = self.IRM.sample_irm_at(point, ground_level, nn);
            iris = self.IRM.validate_basepoint(points4, bools, vec);
            [~, k] = max(iris);
            ps4 = points4(k(1:n), :);
            b=bools(k(1:n), :);
            samples(n)= RRT.Node();
            for ii=1:n
                samples(ii)=RRT.Node(ps4(1),ps4(2),ps4(4),s);
                samples(ii).bools=b(ii,:);
            end
        end
        
        function sample = sample_from_node(self, node)                        
            xyth=[node.pose(1), node.pose(2),atan2(node.pose(4),node.pose(3))];            
            sample=[];
            c=0;
            while c<100                
                c=c+1;
                ssample=RRT.Node(xyth(1)+randn()*0.1,xyth(2)+randn()*0.1,xyth(3)+randn()*0.1,min(node.pose(5)+0.1,self.task_smax));
                if self.isValid7(ssample)   
                    sample=ssample;
                    break
                end
            end
            
        end


        function samples = sample_valid_irm(self, s, n)
            samples(n) = RRT.Node();

            for ii = 1:n

                while ~any(samples(ii).pose) %all zero. all zero is just not possible normally
                    sample = self.sample_irm(s, 1);

                    if self.isValid7(sample)
                        samples(ii) = sample;
                    end

                end

            end

        end

    end

end

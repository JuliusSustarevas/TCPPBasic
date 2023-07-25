classdef KDTree < handle
    % This is a customee KDTree implementation that is suitable for the
    % specific domain used in tcpp (See Node class) and specific calls made
    % by TCPP planner. The class is based on  Douglas' KD tree https://github.com/douglasdotc/RRT.KDTree_MATLAB/blob/main/CLS_RRT.KDTree.m
    properties
        %% Description://///////////////////////////////////////////////////////////////////////////
        % CLS_RRT.KDTree expects nodes defined with Superclass "handle" (classdef nodes < handle)
        % And expect the nodes defined with properties along with Data:
        % node --- Data_1
        %       |- ...
        %       |- Data_n
        %       |- KDT_Lchild
        %       -- KDT_Rchild
        % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
        root
        n_dim

    end

    methods

        function self = KDTree()
        end

        function f = branching_factor(self, varargin)

            if isempty(varargin)
                node = self.root;
            else
                node = varargin{1};
            end

            f = 0;

            if ~isempty(node.KDT_Lchild)
                f = f + 1 + self.branching_factor(node.KDT_Lchild);
            end

            if ~isempty(node.KDT_Rchild)
                f = f + 1 + self.branching_factor(node.KDT_Rchild);
            end

            if node == self.root
                f = f / self.outtree_size();
            end

        end

        function s = size(self, varargin)

            if isempty(varargin)
                node = self.root;
            else
                node = varargin{1};
            end

            s = 1;

            if ~isempty(node.KDT_Lchild)
                s = s + self.size(node.KDT_Lchild);
            end

            if ~isempty(node.KDT_Rchild)
                s = s + self.size(node.KDT_Rchild);
            end

        end

        function s = outtree_size(self, varargin)

            if isempty(varargin)
                node = self.root;
            else
                node = varargin{1};
            end

            s = double(~isempty(node.KDT_Lchild) || ~isempty(node.KDT_Rchild));

            if ~isempty(node.KDT_Lchild)
                s = s + self.outtree_size(node.KDT_Lchild);
            end

            if ~isempty(node.KDT_Rchild)
                s = s + self.outtree_size(node.KDT_Rchild);
            end

        end

        function [isReached, isInserted] = insert(self, node, varargin)
            %% Description://///////////////////////////////////////////////////////////////////////////
            % Recurrsively search for the the leaf in the KD tree where the new point (pt_new) fits.
            % When the place is found, define:
            %   node.KDT_Lchild OR node.KDT_Rchild = pt_new (*)
            %
            % Inputs:
            % 1. pt_new:    the node to be insert
            % 2. node:      the node to be examined [init: node(1)]
            % 3. varargin
            %    - current_dim: The dimension that the function is looking at
            %    - isReached:   Flag for founding the insert location
            %    - isInserted:  Flag for inserting pt_new (*)
            % Outputs:
            % 1. isReached:     --
            % 2. isInserted:    --
            % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\

            if isempty(self.root)
                self.root = node;
                isReached = true;
                isInserted = true;
                self.n_dim=size(node.pose, 2);
                return
            end

            if isempty(varargin)
                current_dim = 0;
                isReached = false;
                isInserted = false;
                current_node = self.root;
            else
                current_dim = varargin{1};
                isReached = varargin{2};
                isInserted = varargin{3};
                current_node = varargin{4};
            end

            

            if isempty(current_node)
                isReached = true;

            elseif node.pose == current_node.pose
                % Duplicated, do nothing                
                display("Attemped to insert duplicate node to KD. insert failed")
                isReached=false; 
                isInserted=false; 
                return 

            elseif node.pose(current_dim + 1) < current_node.pose(current_dim + 1)
                % Go left side
                current_dim = mod((current_dim + 1), self.n_dim);
                [isReached, isInserted] = self.insert(node, current_dim, isReached, isInserted, current_node.KDT_Lchild);

                if isReached && ~isInserted
                    current_node.KDT_Lchild = node;
                    isInserted = true;
                end

            else
                % Go right side
                current_dim = mod((current_dim + 1), self.n_dim);
                [isReached, isInserted] = self.insert(node, current_dim, isReached, isInserted, current_node.KDT_Rchild);

                if isReached && ~isInserted
                    current_node.KDT_Rchild = node;
                    isInserted = true;
                end

            end

        end

        function [nn, min_dist] = NN(self, pt, varargin)
            %% Description://///////////////////////////////////////////////////////////////////////////
            % Recurrsively find the nearest neighbour with KD tree structured
            % node
            % Input:
            % 1. pt:    Point of interest
            % 2. node:  the node to be examined [init: node(1)]
            % 3. varargin:
            %    - current_dim: Current dimension
            %    - min_dist:    Minimum distance
            %    - NN:          Nearest neighbour found at the moment
            % Outputs:
            % 2. NN:        --
            % 1. min_dist:  --
            % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
            if isempty(varargin)
                current_dim = 1;
                min_dist = Inf;
                nn = [];
                current_node = self.root;
            else
                current_dim = varargin{1};
                nn = varargin{2};
                min_dist = varargin{3};
                current_node = varargin{4};
            end        

            % Capture leaf node
            if isempty(current_node)
                return
            end

            % Found point nearer than the nearest:
            dist = pt.dist(current_node);

            if dist < min_dist
                nn = current_node;
                min_dist = dist;
            end

            % Calculate next dimension
            next_dim = mod(current_dim, self.n_dim) + 1;

            % Visit subtree
            if pt.pose(current_dim) < current_node.pose(current_dim)
                % Go left side
                [nn, min_dist] = self.NN(pt, next_dim, nn, min_dist, current_node.KDT_Lchild);

                if min_dist >= pt.weights(current_dim) * abs(pt.pose(current_dim) - current_node.pose(current_dim)) && current_dim ~= self.n_dim
                    [nn, min_dist] = self.NN(pt, next_dim, nn, min_dist, current_node.KDT_Rchild);
                end

            else
                % Go right side
                [nn, min_dist] = self.NN(pt, next_dim, nn, min_dist, current_node.KDT_Rchild);

                if min_dist >= pt.weights(current_dim) * abs(pt.pose(current_dim) - current_node.pose(current_dim))
                    [nn, min_dist] = self.NN(pt, next_dim, nn, min_dist, current_node.KDT_Lchild);
                end

            end

        end

        function [nn, min_dist] = NNRad(self, pt, rad)            
            %
            current_dim = 1;
            nn = [];
            min_dist = [];
            current_node = self.root;
            n_last=pt;
            % find
            
            n_last=self.NNRad_internal(pt, rad,n_last, current_dim, current_node);
            nn=pt.to_kdlist();
            min_dist=zeros(length(nn),1);
            for ii=1:length(nn)
                min_dist(ii)=nn(ii).dist(pt);
%                 min_dist(ii)=pt.dist(nn(ii));
            end
            %%sort
            [min_dist, jj] = sort(min_dist);
            nn = nn(jj);
        end

        function n_last = NNRad_internal(self, pt, rad, n_last, current_dim, current_node)            
            %% Description://///////////////////////////////////////////////////////////////////////////
            % Recurrsively find the nearest neighbour with KD tree structured
            % node
            % Input:
            % 1. pt:    Point of interest
            % 2. node:  the node to be examined [init: node(1)]
            % 3. varargin:
            %    - current_dim: Current dimension
            %    - min_dist:    Minimum distance
            %    - NN:          Nearest neighbour found at the moment
            % Outputs:
            % 2. NN:        --
            % 1. min_dist:  --
            % \\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\\
          
            % Capture leaf node
            if isempty(current_node)
                return
            end

            % Found point nearer than the nearest:
            dist = current_node.dist(pt);
%           dist = pt.dist(current_node);

            if dist < rad                
                temp=n_last;
                n_last=current_node;
                temp.KDT_linked_list=current_node;                     
            end

            % Calculate next dimension
            next_dim = mod(current_dim, self.n_dim) + 1;

            % Visit subtree
            if pt.pose(current_dim) < current_node.pose(current_dim)
                % Go left side
                n_last = self.NNRad_internal(pt, rad, n_last,next_dim, current_node.KDT_Lchild);

                if rad >= pt.weights(current_dim) * abs(pt.pose(current_dim) - current_node.pose(current_dim)) 
                    n_last = self.NNRad_internal(pt, rad, n_last,next_dim, current_node.KDT_Rchild);
                end

            else
                % Go right side
                n_last = self.NNRad_internal(pt, rad, n_last,next_dim, current_node.KDT_Rchild);

                if rad >= pt.weights(current_dim) * abs(pt.pose(current_dim) - current_node.pose(current_dim)) && current_dim ~= self.n_dim
                    n_last = self.NNRad_internal(pt, rad, n_last,next_dim, current_node.KDT_Lchild);
                end

            end

        end

    end

end

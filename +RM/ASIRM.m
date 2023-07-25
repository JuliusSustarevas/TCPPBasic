classdef ASIRM < handle

    
    %   ASIRM (ArmStoneInverseReachabilityMap). Basic idea is that we have a lot of
    %   data stored in files  +RM/data/... see xxx_fn descriptions. 
    %   This data might be redundant. Also I used both sparse and dense
    %   arrays to store this data. I believe sparse array functionality is
    %   no longer used in this class as dense array proved to be faster.
    
    % Another basic idea is that this class stores a 4D grid of robot base
    % X,Y,Z,TH. And inside these grid cells we store booleans of IK
    % solutions for putting the end-effector at x=0,y=0,z=0 but at
    % different orientations. 
    
    %   !!!
    %   Anyway, the basic problem this class is trying to solve is that we
    %   want a function that takes X Y Theta of a robot base in the world 
    %   frame and Z of a task point in the world frame. (This is expresed
    %   as tuple x y z th) Dont confuse what the Z means. This function
    %   should also take in desired end-effector orientation as unit
    %   vector. And given these things it should tell us the IRI of that
    %   robot base pose to reach point at x=0,y=0,z=Z and specified
    %   orientation. So this is done in point2iri function. 
    
    properties %(Access = private)
        %Files
        rm_fn = "+RM/data/as_rm.mat"
        irm_fn = "+RM/data/as_irm_data.mat"
        %Privates
        asrm
        poses % test poses
        zvecs
        xs
        ys
        zs
        ths
        n % num elem of of squeezed data
        nn %num elem of sparse arrays
        m %num elem of poses
        sparse_index % sparse nnx1 logical array. For easier non-zero element find
        sparse_bools % sparse nnxm logical array.  representing rm
        sparse_grid % sparse nnx4 double array. for storing points
        compact_bools % only non zero compact
        compact_grid % only non zero compact
        dx % the vox difference
        dy % the vox difference
        dz % the vox difference
        dth % the vox difference
        angtoll=15;
        % sampling
        z_layers % celllist of cumsums and other z stuffs  first index corresponds to z of base according to zs. care
        vectorised_angle_minus = @(to, from) atan2(sin(to - from), cos(to - from))
        angle_minus = @(to, from) angdiff(from, to)

    end

    methods
        % Note that constructor will attempt to load from file first. This
        % might take 30s or so. If file is not found it will attempt to
        % build all the dense data. However!!! this would take foreever. So
        % dont do it. Find the file.
        function self = ASIRM()
            % General
            self.asrm = load(self.rm_fn).asrm;
            % indexing
            radius = max([abs(self.asrm.xs); abs(self.asrm.ys); abs(self.asrm.zs)]);
            span = linspace(-radius, radius, round((radius * 2) / self.asrm.d));
            temp = exp((1:72) * 1i * 2 * pi / 72);

            self.xs = span;
            self.ys = span;
            self.zs = span;
            self.ths = atan2(imag(temp), real(temp));

            self.dx = unique(abs(diff(self.xs)));
            self.dy = unique(abs(diff(self.ys)));
            self.dz = unique(abs(diff(self.zs)));
            self.dx = self.dx(1); self.dy = self.dy(1); self.dz = self.dz(1);
            self.dth = 2 * pi / 72;

            [X, Y, Z, TH] = ndgrid(span, span, span, self.ths);
            x = X(:); y = Y(:); z = Z(:); th = TH(:);

            self.nn = length(x);
            self.m = self.asrm.m;

            self.sparse_index = false(self.nn, 1);
            self.sparse_bools = false(self.nn, self.m);
            self.sparse_grid = zeros(self.nn, 4);

            self.zvecs = self.asrm.zvecs;
            self.poses = self.asrm.poses;

            try % Try loading from file
                disp("Loading from file...")
                data = load(self.irm_fn).data;

                self.sparse_bools = data.sparse_bools;
                self.sparse_index = data.sparse_index;
                self.sparse_grid = data.sparse_grid;

                self.compact_bools = data.compact_bools;
                self.compact_grid = data.compact_grid;
                self.z_layers = data.z_layers;

                assert(all(size(self.sparse_index) == [self.nn, 1]))
                assert(all(size(self.sparse_bools) == [self.nn, self.m]))
                assert(all(size(self.sparse_grid) == [self.nn, 4]))
                assert(all(size(self.z_layers) == [length(self.zs), 1]))

            catch
                disp("Loading from file failed. Computing...")
                assert(false)

                for ii = 1:length(x)

                    [xi, yi, zi, thi] = ind2sub([length(self.xs), length(self.ys), length(self.zs), length(self.ths)], ii);
                    assert(all([self.xs(xi) self.ys(yi) self.zs(zi) self.ths(thi)] == [x(ii), y(ii), z(ii), th(ii)]))
                    assert(ii == self.indexfun(xi, yi, zi, thi))

                    T = eul2tform([th(ii) 0 0]);
                    T(1:3, 4) = [x(ii) y(ii) z(ii)]';
                    Tinv = TForm.tform2inv(T);
                    bools = self.asrm.point2bools(Tinv(1:3, 4)');

                    if any(bools)
                        self.sparse_index(ii) = true;
                        self.sparse_bools(ii, :) = bools;
                        self.sparse_grid(ii, :) = [x(ii) y(ii) z(ii), th(ii)];
                    end

                    display(round(ii / length(x), 4));
                end

                self.sparse_index = sparse(self.sparse_index);
                self.sparse_bools = sparse(self.sparse_bools);
                self.sparse_grid = sparse(self.sparse_grid);

                disp("IRM data gathered.")
                disp("Converting sparse to full")
                self.compact_bools = full(self.sparse_bools(self.sparse_index, :));
                self.compact_grid = full(self.sparse_grid(self.sparse_index, :));
                disp("Converting done")
                %compute cumsums
                disp("Retriecing iri z plane cumsums")
                self.z_layers = cell(length(self.zs), 1);

                for ii = 1:length(self.zs)
                    zindex = abs(self.compact_grid(:, 3) - self.zs(ii)) <= self.dz / 2;
                    assert(size(zindex, 2) == 1)
                    iris = self.comp_iris(self.compact_bools(zindex, :));
                    layer.z = self.zs(ii);
                    layer.zindex = zindex;
                    layer.grid = self.compact_grid(zindex, :);
                    layer.bools = self.compact_bools(zindex, :);
                    layer.iris = iris;
                    layer.cumsumiris = cumsum(iris);
                    self.z_layers{ii} = layer;
                    ii
                end

            end

        end

        function index = indexfun(self, xi, yi, zi, thi)
            index = sub2ind([length(self.xs), length(self.ys), length(self.zs), length(self.ths)], xi, yi, zi, thi);
        end

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %% Point2_
        function bools = point2bools(self, points)
            bools = false(size(points, 1), self.m);
            index = self.point2index(points);

            bools(index < 0, :) = false(sum(index < 0), self.m);
            bools(sum(index > 0), :) = self.sparse_bools(sum(index > 0), :);
        end

        function iri = point2iri(self, points, varargin)
            [vec, tol] = self.parse_orientation_args(varargin{:});
            assert(size(points, 1) == 1);

            if length(varargin) == 3
                index = varargin{3};
            else
                index = self.point2index(points);
            end

            if index < 0
                iri = 0;
                return
            end

            bools = self.sparse_bools(index, :);

            if isempty(vec)
                iri = self.comp_iris(bools);
            else
                cs = cos(-points(4));
                sn = sin(-points(4));
                vec = [vec(1) * cs - sn * vec(2); vec(1) * sn + cs * vec(2); vec(3)]; %rotation bout inverse of point 4 z
                iri = self.comp_iri(bools, vec, tol);
            end

        end

        function index = point2index(self, points)
            %This should be in base footprint frame
            index = zeros(size(points, 1), 1);

            for ii = 1:size(index, 1)
                idx = find(abs(self.xs - points(ii, 1)) <= self.dx );
                idy = find(abs(self.ys - points(ii, 2)) <= self.dy );
                idz = find(abs(self.zs - points(ii, 3)) <= self.dz );
                idth = find(abs(self.vectorised_angle_minus(self.ths, points(ii, 4))) <= self.dth );

                if ~isempty(idx) && ~isempty(idy) && ~isempty(idz) && ~isempty(idth)
                    [~, i] = min(abs(self.xs(idx) - points(ii, 1)));
                    idx = idx(i);
                    [~, i] = min(abs(self.ys(idy) - points(ii, 2)));
                    idy = idy(i);
                    [~, i] = min(abs(self.zs(idz) - points(ii, 3)));
                    idz = idz(i);
                    [~, i] = min(abs(self.vectorised_angle_minus(self.ths(idth), points(ii, 4))));
                    idth = idth(i);
                    index(ii) = self.indexfun(idx, idy, idz, idth);
                else
                    index(ii) = -1;
                end

            end

        end

        %% Index2_
        function point = index2point(self, index)
            point = self.sparse_grid(index, :);
        end

        function bools = index2rbools(self, index)
            bools = self.sparse_bools(index, :);
        end

        %% iri
        function iri = comp_iri(self, bools, vec, tol)
            % Assumes vec is already rotated
            assert(size(bools, 1) == 1)
            angle_bools = acosd(self.zvecs(1:200, :) * vec) < tol;
            reduced_bools = (bools & (repmat(angle_bools', 1, 11))); %no need to divide cuz both norms are one
            iri = sum(reduced_bools, 2) / (sum(angle_bools) * 11);

        end

        function iris = comp_iris(self, bools)
            iris = sum(bools, 2) ./ self.m;
        end

        function [points, iris] = get_irm(self, varargin)
            [vec, tol] = parse_orientation_args(varargin{:});
            nc = size(self.compact_bools, 1);
            points = self.compact_grid;

            if isempty(vec)
                iris = self.comp_iris(self.compact_bools);
            else

                vecs = repmat(vec, 1, nc);

                for ii = 1:nc
                    cs = cos(-points(ii, 4));
                    sn = sin(-points(ii, 4));
                    v = [vecs(1, ii) * cs - sn * vecs(2, ii); vecs(1, ii) * sn + cs * vecs(2, ii); vecs(3, ii)]; %rotation bout inverse of point 4 z
                    iri = self.comp_iri(self.compact_bools, v, tol);
                end

            end

        end

        function [points, iris] = get_irm_at(self, task_point, varargin)

            z = -task_point(3);
            idz = find(abs(self.zs - z) <= self.dz);

            if ~isempty(idz)
                [~, i] = min(abs(self.zs(idz) - z));
                idz = idz(i);
            else
                disp("task point out of z reach")
                return
            end

            [vec, tol] = self.parse_orientation_args(varargin{:});

            zlayer = self.z_layers{idz};
            nc = size(zlayer.bools, 1);
            bools = zlayer.bools;
            points = zlayer.grid;
            points(:, 1) = points(:, 1) + task_point(1);
            points(:, 2) = points(:, 2) + task_point(2);
            points(:, 3) = points(:, 3) - self.zs(idz);
            iris = zeros(nc, 1);

            if isempty(vec)
                iris = self.comp_iris(bools);
            else

                vecs = repmat(vec, 1, nc);

                for ii = 1:nc
                    cs = cos(-points(ii, 4));
                    sn = sin(-points(ii, 4));
                    v = [vecs(1, ii) * cs - sn * vecs(2, ii); vecs(1, ii) * sn + cs * vecs(2, ii); vecs(3, ii)]; %rotation bout inverse of point 4 z
                    iris(ii) = self.comp_iri(bools(ii, :), v, tol);
                end

            end

        end

        function bools=scenePoint32Bools(self,task_point,ground_level,point3)
            z = ground_level - task_point(3);
            index = point2index(self, [point3(1)-task_point(1) point3(2)-task_point(2) z point3(3)]);

            if index < 0
                bools=false(1,self.m);
                return
            end

            bools = self.sparse_bools(index, :);
        end

        function [points, bools, iris] = sample_irm_at(self, task_point, ground_level, n)
            %     z: -1.2824
            %     zindex: [1267364×1 logical]
            %       grid: [9349×4 double]
            %      bools: [9349×2200 logical]
            %       iris: [9349×1 double]
            % cumsumiris: [9349×1 double]

            z = ground_level - task_point(3);
            idz = find(abs(self.zs - z) <= self.dz);

            if ~isempty(idz)
                [~, i] = min(abs(self.zs(idz) - z));
                idz = idz(i);
            else
                disp("task point out of z reach")
                points=[];
                bools=[];
                iris=[];
                return
            end

            zlayer = self.z_layers{idz};
            points = zlayer.grid;

            s = zlayer.cumsumiris(end);
            smpl_ids = sum(bsxfun(@le, zlayer.cumsumiris', rand(n, 1) * s), 2) + 1;
            points = points(smpl_ids, :);
            points(:, 1) = points(:, 1) + task_point(1);
            points(:, 2) = points(:, 2) + task_point(2);
            points(:, 3) = ground_level;
            iris = zlayer.iris(smpl_ids);
            bools = zlayer.bools(smpl_ids, :);
        end

        function [iris] = validate_basepoint(self, basepoints, bools, varargin)
            [vec, tol] = self.parse_orientation_args(varargin{:});
            nc = size(basepoints, 1);
            iris = zeros(nc, 1);

            if isempty(vec)
                iris = self.comp_iris(bools);
            else

                vecs = repmat(vec, 1, nc);

                for ii = 1:nc
                    cs = cos(-basepoints(ii, 4));
                    sn = sin(-basepoints(ii, 4));
                    v = [vecs(1, ii) * cs - sn * vecs(2, ii); vecs(1, ii) * sn + cs * vecs(2, ii); vecs(3, ii)]; %rotation bout inverse of point 4 z
                    iris(ii) = self.comp_iri(bools(ii, :), v, tol);
                end

            end

        end

        % Imagine this is bruteforced manipulability but its based on IRI
        function axes_limits = gen_elipse(self, taskpoint, basepoint, varargin)
            lb = [(basepoint(1:3) - taskpoint) basepoint(4)];            
            axes_limits = zeros(4, 1);
            axes_vecs=[1 0 0; 0 1 0]';
            
            axes_vecs=eul2rotm([basepoint(4) 0 0])*axes_vecs;
            axes_vecs=[[axes_vecs' zeros(2,1)]; 0 0 1 0; 0 0 0 1];

            for ii = 1:4

                for jj = 1:100
                    testpoint = lb;
                    testpoint = testpoint + axes_vecs(ii,:)*0.025* jj;
                    iri = self.point2iri(testpoint, varargin{:});

                    if iri < 0.5
                        break
                    end

                    testpoint = lb;
                    testpoint = testpoint - axes_vecs(ii,:)*0.025* jj;
                    iri = self.point2iri(testpoint, varargin{:});

                    if iri < 0.5
                        break
                    end

                end

                axes_limits(ii) = 0.025 * (jj - 1);
            end

        end

        %% helpers
        function [vec, angtol] = parse_orientation_args(self, varargin)
            vec = [];
            angtol = self.angtoll;

            if length(varargin) >= 1
                vec = varargin{1};

                if size(vec, 2) > size(vec, 1)
                    vec = vec';
                end

            end

            if length(varargin) == 2
                angtol = varargin{2};
            end

        end

        function save(self, fn)

            data.sparse_bools = self.sparse_bools;
            data.sparse_index = self.sparse_index;
            data.sparse_grid = self.sparse_grid;

            data.compact_bools = self.compact_bools;
            data.compact_grid = self.compact_grid;
            data.z_layers = self.z_layers;
            save(fn, "data", '-v7.3');
        end

        function get_size(this)
            props = properties(this);
            totSize = 0;

            for ii = 1:length(props)
                currentProperty = getfield(this, char(props(ii)));
                s = whos('currentProperty');
                totSize = totSize + s.bytes;
            end

            fprintf(1, '%d bytes\n', totSize);
        end

    end

end

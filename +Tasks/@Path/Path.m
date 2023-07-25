classdef (Abstract) Path < handle

    properties
        path% nx3 path
        traj%4x4xn trajectory
        s%path integral
    end

    methods

        function self = Path(seed_path)
            self.path = seed_path;
            self.s=self.gett(1);
%             self.normalise_path(seed_path);
            % self.traj = self.normalise_path(seed_path);

        end

        function ii=s2index(self,s)
            ii=round(size(self.path,1)* (s/self.s(end)));
            ii=max(min(ii,size(self.path,1)),1);            
        end

        function scale(self, scale_vec)
            % scale  Scale the path along xyz axes
            self.path = self.normalise_path(self.path);
            self.path(:, 1) = self.path(:, 1) * scale_vec(1);
            self.path(:, 2) = self.path(:, 2) * scale_vec(2);
            self.path(:, 3) = self.path(:, 3) * scale_vec(3);
        end

        function resample(self, density)
            cumlength = self.cumlen();
            [~, IA, ~] = uniquetol(cumlength);
            cumlength = cumlength(IA);
            self.path = self.path(IA, :);
            self.path = interp1(cumlength, self.path, linspace(0, cumlength(end), round(cumlength(end) / density)), 'linear');
%             self.path = interp1(cumlength, self.path, linspace(0, cumlength(end), round(cumlength(end) / density)), 'pchip');
           
        end

        function pts = superimpose(self, pattern)
            ptrn_l = pattern.cumlen();
            ptrn_f = pattern.asfun();
            ptrn_tf = @(t) interp1(ptrn_l, pattern.path(:, 1) ./ max(pattern.path(:, 1)), t);

            trj_l = self.cumlen();
            tforms = self.toTForm(self);
            vec = TForm.tform2vec(tforms);

            trj_f = @(l) TForm.vec2tform([interp1(trj_l, vec(:, 1:3), l) quatnormalize(interp1(trj_l, vec(:, 4:end), l))]); % bad interpolation but will work for now

            % Assuming x dir is the dir of pattern
            ttl = ptrn_l(end) * round(trj_l(end) / max(pattern.path(:, 1)));
            density = 0.0025;
            ll = 0:density:ttl;
            pts = zeros(length(ll), 4);

            for ii = 1:length(ll)
                l = ll(ii);
                pattern_progress = ptrn_tf(mod(l, ptrn_l(end)));

                lf = (l - mod(l, ptrn_l(end))) + pattern_progress * ptrn_l(end);
                lf = lf * max(pattern.path(end, 1)) / ptrn_l(end);

                if lf > trj_l
                    pts = pts(1:ii-1, :);
                    break
                end

                tform = trj_f(lf);
                pattern_point = ptrn_f(mod(l, ptrn_l(end)));
                pattern_point(1) = 0;
                pts(ii, :) = (tform * [pattern_point'; 1])';
                

                %                 plot(pts(1:ii,1),pts(1:ii,2))
                %                 drawnow
            end

            self.path = pts(:, 1:3);

        end

        function h = asfun(self)
            l = self.cumlen();
            % l = l ./ l(end);
            h = @(t) interp1(l, self.path, t, 'pchip');
        end

        function t = gett(self, speed)
            l = self.cumlen();
            t = l ./ speed;
        end

        function cumlength = cumlen(self, varargin)
            path_ = self.path;

            if ~isempty(varargin)
                path_ = varargin{1};
                path_ = path_.path;
            end

            % Compute cumulatative length of the path
            cumlength = [0; cumsum(sqrt(sum(diff(path_).^2, 2)))];
            if any(isnan(cumlength))
                a=5;
            end
        end

        function valid = is_repeating(self)
            valid = all(abs(self.path(1, 2:3) - self.path(end, 2:3)) < 1e-5);
        end

        function h = plot(self, varargin)
            iend = size(self.path(:, 1), 1);

            if ~isempty(varargin)
                iend = varargin{1};
            end

            % scale  Scale the path along xyz axes
            x = self.path(1:iend, 1);
            y = self.path(1:iend, 2);
            z = self.path(1:iend, 3);

            c = colormap("winter");
            l = (1:length(x)) ./ length(x);
            c = interp1(linspace(0, 1, length(c)), c, l);
            h = scatter3(x, y, z, 5, c, 'filled');
        end

        function smooth(self, s_ratio)
            s = size(self.path);
%             x = smooth([self.path(:, 1); self.path(:, 1); self.path(:, 1)], round(s(1) * s_ratio));
%             y = smooth([self.path(:, 2); self.path(:, 2) ;self.path(:, 2)  ], round(s(1) * s_ratio));
%             z = smooth([self.path(:, 3); self.path(:, 3); self.path(:, 3)], round(s(1) * s_ratio));
            
            x = smooth( self.path(:, 1), round(s(1) * s_ratio));
            y = smooth( self.path(:, 2) , round(s(1) * s_ratio));
            z = smooth( self.path(:, 3), round(s(1) * s_ratio));
%             x=x(s(1)+1:2*s(1));
%             y=y(s(1)+1:2*s(1));
%             z=z(s(1)+1:2*s(1));
            self.path = [x y z];
        end

    end

    methods (Static)

        function tform = toTForm(path_, varargin)
            % path nx3; tform 4x4xn. Turns a path into a trajectory. Assumes no rotation about local X axis. X axis pointed along the path.
            path_ = path_.path;
            n = length(path_);
            ax = diff(path_, 1);
            ax = [ax; ax(end, :)];
            tform = zeros(4, 4, n);

            for ii = 1:n
                %Follow the curve and compute rotations in the appropriate frame
                ax0 = ax(ii, :);
                rotmz = eul2rotm([atan2(ax0(2), ax0(1)), 0, 0], "ZYX");

                if ~isempty(varargin) && varargin{1} == "zonly"
                    ey = 0;
                else
                    ax1 = (rotmz) \ ax0';
                    ey = -atan2(ax1(3), ax1(1));
                end

                rotmy = eul2rotm([0, ey, 0], "ZYX");
                tform(:, :, ii) = rotm2tform(rotmz * rotmy);
                tform(1:3, 4, ii) = path_(ii, :)';
            end

        end

        function path_ = fromTForm(tform)
            % path nx3; tform 4x4xn. Turns a path into a trajectory. Assumes no rotation about local X axis. X axis pointed along the path.
            path_ = squeeze(tform(1:3, 4, :))';
        end

        function h = plotTForm(tform, frame_density)
            x = squeeze(tform(1, 4, :));
            y = squeeze(tform(2, 4, :));
            z = squeeze(tform(3, 4, :));

            ax = squeeze(tform(1:3, 1, :));
            ay = squeeze(tform(1:3, 2, :));
            az = squeeze(tform(1:3, 3, :));
            cumlen = [0; cumsum(sqrt(sum(diff([x y z]).^2, 2)))];
            n = find([0; diff(round(cumlen ./ frame_density))]); % find entries that go over density threshold

            c = colormap("winter");
            l = (1:length(x)) ./ length(x);
            c = interp1(linspace(0, 1, length(c)), c, l);

            h = scatter3(x, y, z, 5, c, 'filled');
            hold on
            quiver3(x(n), y(n), z(n), ax(1, n)', ax(2, n)', ax(3, n)',0.1, 'r');
            quiver3(x(n), y(n), z(n), ay(1, n)', ay(2, n)', ay(3, n)',0.1, 'g');
            quiver3(x(n), y(n), z(n), az(1, n)', az(2, n)', az(3, n)',0.1, 'b');
            hold off
        end

        function path_ = normalise_path(path_)
            % normalise  given path so that its amplitude in any direction is max 1.
            normalise = @(x) (x-min(abs(x))) ./ (max(abs(x))-min(abs(x)));
            path_ = [normalise(path_(:, 1)), normalise(path_(:, 2)), normalise(path_(:, 3))];
            path_(isnan(path_)) = 0;
        end

    end

end

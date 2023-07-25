% Generate a sphere-grid (see the plot) around xarm6 robot. This is used
% later for creation of RM/IRM
xarm_radius=1.2;% this is with extruder
xx=linspace(-xarm_radius,xarm_radius,round(2*xarm_radius/0.05));
[X,Y,Z]=ndgrid(xx,xx,xx);
%%
grid=[X(:),Y(:),Z(:)];
grid=grid(sqrt(sum(grid.^2,2))<=1.2,:);
plot3(grid(:,1),grid(:,2),grid(:,3),'.')
%%
% grid=[0.48,0,0.38]
% save("xarm_grid_checked","grid")
%%
% check=vertcat(arg{:});
% sum(check)
% grid=grid(check,:);
% save("xarm_grid_test","grid")
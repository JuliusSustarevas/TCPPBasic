function  draw_rm(points, ris)
%Produces 3d scatter plot of the voxel structure. Takes only xyz vox 

scatter3(points(:,1),points(:,2),points(:,3),200,ris./max(ris),'filled');
end


close
clear
clc
load Project1/allData2.mat

%% Distância linear total percorrida na simulação pelo ego-veículo

PP = [cell2mat(arrayfun(@(S) S.INSMeasurements{1,1}.Position', allData, 'UniformOutput', false))', cell2mat(arrayfun(@(S) S.INSMeasurements{1,1}.Orientation', allData, 'UniformOutput', false))'];


axis equal
subplot(1,2,1)
plot(PP(:,1), PP(:,2), '.b');

newPP = [];

for i=1:uint32(size(PP,1)/10)
    newPP = [newPP; mean(PP((i-1)*10+1:i*10,:),1)];
end

PP = newPP;
subplot(1,2,2)
plot(PP(:,1), PP(:,2), '.b');

%257,... is the right value
Lcar = sum(sqrt(sum(diff(PP(:,1:3)).^2,2)));

%% Distância linear total percorrida na simulação pelo ego-veículo

vehicleDims = vehicleDimensions(); %4.7m long, 1.8m wide, and 1.4m high

limits = [-2 vehicleDims.Length;
-vehicleDims.Width/2-1 vehicleDims.Width/2+1;
-1 vehicleDims.Height+1];

xlimits = allData(1).PointClouds{1,1}.XLimits;
%ylimits = allData(1).PointClouds{1,1}.YLimits;
ylimits = [-50 50];
zlimits = allData(1).PointClouds{1,1}.ZLimits;

lidarViewer = pcplayer(xlimits, ylimits, zlimits);
%Set axis labels
xlabel(lidarViewer.Axes, 'X (m)');
ylabel(lidarViewer.Axes, 'Y (m)');
zlabel(lidarViewer.Axes, 'Z (m)');

points3 = [];
for k=1:10:861-1
    pose = PP(uint32((k-1)/10)+1,:);
    T = geotransf(pose(1),pose(2),pose(3),pose(4),pose(5),pose(6));

    pc = allData(k).PointClouds{1,1};

    ground = segmentGroundFromLidarData(pc, 'ElevationAngleDelta', 10);

    points = pc.Location;

    points2 = [];

    for i=1:size(points,1)
        for j=1:size(points,2)
            point = [points(i,j,1) points(i,j,2) points(i,j,3) 1]';
            if and(and(~isnan(point(1)),~ground(i,j)), point(3)>0.01)
                if ~(point(1) > limits(1,1) & point(1) < limits(1,2) & point(2) > limits(2,1) ...
                    & point(2) < limits(2,2) & point(3) > limits(3,1) & point(3) < limits(3,2))
                    points2 = [points2; point(1:3)'];
                    point = T*point;
                    points3 = [points3; point(1:3)'];
                end
            end
        end
    end

    pc.removeInvalidPoints();
    
    %View the point cloud
    if size(points3,1)>0
        view(lidarViewer, points3);
        pause(0.2)
    end
end

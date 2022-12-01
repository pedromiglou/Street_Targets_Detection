close
clear
clc
load Street_Targets_Detection/allData3.mat

%% Distância linear total percorrida na simulação pelo ego-veículo

%PP = [cell2mat(arrayfun(@(S) S.INSMeasurements{1,1}.Position', allData, 'UniformOutput', false))', cell2mat(arrayfun(@(S) S.INSMeasurements{1,1}.Orientation', allData, 'UniformOutput', false))'];
PP = cell2mat(arrayfun(@(S) S.INSMeasurements{1,1}.Position', allData, 'UniformOutput', false))';

axis equal
subplot(1,2,1)
plot(PP(:,1), PP(:,2), '.b');

newPP = [];

for i=1:10:size(PP,1)
    newPP = [newPP; mean(PP(max(1, i-10):min(i+10, size(PP,1)),:),1)];
end

PP = newPP;

newPP = zeros([size(PP,1),6]);
for i=1:size(PP,1)-1
    newPP(i,:) = [PP(i,:) 0 0 atan2(PP(i+1,2)-PP(i,2), PP(i+1,1)-PP(i,1))];
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

new_points = [];
for k=1:10:size(allData,2)-1
    pose = PP(uint32((k-1)/10)+1,:);
    %PP = [cell2mat(arrayfun(@(S) S.ActorPoses.Position, allData, 'UniformOutput', false)), cell2mat(arrayfun(@(S) S.ActorPoses.Orientation, allData, 'UniformOutput', false))];

    %pose = PP(uint32((k-1)/10)+1,:);
    T = geotransf(pose(1),pose(2),pose(3),pose(4),pose(5),pose(6));

    ptCloud = allData(k).PointClouds{1,1};

    points = struct();
    points.EgoPoints = ptCloud.Location(:,:,1) > limits(1,1) ...
        & ptCloud.Location(:,:,1) < limits(1,2) ...
        & ptCloud.Location(:,:,2) > limits(2,1) ...
        & ptCloud.Location(:,:,2) < limits(2,2) ...
        & ptCloud.Location(:,:,3) > limits(3,1) ...
        & ptCloud.Location(:,:,3) < limits(3,2);
    
    points2 = struct();
    points2.EgoPoints = ptCloud.Location(:,:,3) < 0.1 ...
        | (ptCloud.Location(:,:,1)-pose(1)).^2 ...
        + (ptCloud.Location(:,:,2)-pose(2)).^2 ...
        + (ptCloud.Location(:,:,3)-pose(3)).^2 > 25;
    
    points.GroundPoints = segmentGroundFromLidarData(ptCloud,'ElevationAngleDelta', 10);

    nonEgoGroundPoints = ~points.EgoPoints & ~points.GroundPoints & ~points2.EgoPoints;

    ptCloudSegmented = select(ptCloud, nonEgoGroundPoints,'Output','full');

    minNumPoints = 3;
    [labels, numClusters] = segmentLidarData(ptCloudSegmented, 1, 180, 'NumClusterPoints', minNumPoints);

    % --------------------------DEBUG-----------------------------
%     idxValidPoints = find(labels);
% 
%     segmentedPtCloud = select(ptCloudSegmented, idxValidPoints);
% 
%      %View the point cloud
%     if size(segmentedPtCloud.Location,1)>2
%         
%         for i=1:size(segmentedPtCloud.Location,1)
%             labels
%             point = T*[segmentedPtCloud.Location(i,:) 1]';
%             new_points = [new_points; point(1:3)'];
%         end
%         view(lidarViewer, new_points)
%         pause(0.2)
%     end
    % ------------------------------------------------------------

    for j=1:uint8(numClusters)
        idxValidPoints = find(labels==j);

        if size(idxValidPoints,1)<2
            continue
        end
    
        segmentedPtCloud = select(ptCloudSegmented, idxValidPoints);

        points = [];
        for i=1:size(segmentedPtCloud.Location,1)
            point = T*[segmentedPtCloud.Location(i,:) 1]';
            points = [points; point(1:3)'];
        end

        center = mean(points, 1);
        
        new_points = [new_points; center];
    end

    if size(new_points)>0
        view(lidarViewer, new_points)
        %pause(0.1)
    end 
end

%Radar and Camera
X = [];
Y = [];
for i=1:10:size(allData,2)-1
    objects = allData(i).ObjectDetections;

    if size(objects,1)==0
        continue
    end

    pose = PP(uint32((i-1)/10)+1,:);

    T = geotransf(pose(1),pose(2),pose(3),pose(4),pose(5),pose(6));

    for j=1:size(objects,1)
        object = objects{j,1}.Measurement;
        
        object = T*[object(1) object(2) 0 1]';

        object = object(1:3)';

        new_points = [new_points; object];
        new_points = [new_points; object];

        if objects{j,1}.ObjectClassID == 4 % pedestrians
            X = [X; object];
            Y = [Y; 4];
        end

        if objects{j,1}.ObjectClassID == 3 % bycicles
            X = [X; object];
            Y = [Y; 3];
        end

        if objects{j,1}.ObjectClassID == 1 % cars
            X = [X; object];
            Y = [Y; 1];
        end

        if objects{j,1}.ObjectClassID == 5 % barriers
            X = [X; object];
            Y = [Y; 5];
        end
    end

end

%% Processing
ptCloud = pointCloud(new_points);
[labels,numClusters] = pcsegdist(ptCloud,2);

new_points = [];
peds = 0;
bikes = 0;
cars = 0;
barriers = 0;

for j=1:uint8(numClusters)
    idxValidPoints = find(labels==j);

    if size(idxValidPoints,1)<3
        continue
    end

    segmentedPtCloud = select(ptCloud, idxValidPoints);

    center = mean(segmentedPtCloud.Location, 1);
    
    new_points = [new_points; center];

    classification = kNearestNeighbors(X, Y, center);
    
    if classification == 1
        cars = cars + 1;
    elseif classification == 3
        bikes = bikes + 1;
    elseif classification == 4
        peds = peds+1;
    elseif classification == 5
        barriers = barriers+1;
    end
end

if size(new_points)>0
    view(lidarViewer, new_points)
end

%% Save on file
fileID = fopen('TP1_results_93283.txt','w');
fprintf(fileID,'93283,');
fprintf(fileID,'%3.2f,',Lcar);
fprintf(fileID,'%3.2f,',peds);
fprintf(fileID,'%3.2f,',cars);
fprintf(fileID,'%3.2f,',bikes);
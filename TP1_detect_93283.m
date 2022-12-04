close all
clear
clc
load Street_Targets_Detection/allData4.mat

%% Distância linear total percorrida na simulação pelo ego-veículo

%PP = [cell2mat(arrayfun(@(S) S.INSMeasurements{1,1}.Position', allData, 'UniformOutput', false))', cell2mat(arrayfun(@(S) S.INSMeasurements{1,1}.Orientation', allData, 'UniformOutput', false))'];
%PP = cell2mat(arrayfun(@(S) S.INSMeasurements{1,1}.Position', allData, 'UniformOutput', false))';
%PP = cell2mat(arrayfun(@(S) S.ActorPoses(1).Position', allData, 'UniformOutput', false))';
PP = cell2mat(arrayfun(@(S) S.INSMeasurements{1,1}.Velocity', allData, 'UniformOutput', false))';

%axis equal
%subplot(1,2,1)
%plot(PP(:,1), PP(:,2), '.b');

newPP = [];

cPos = [0 0 0];

for i=1:size(PP,1)
    %newPP = [newPP; mean(PP(max(1, i-10):min(i+10, size(PP,1)),:),1)];
    cPos = [cPos(1) + PP(i,1)*0.01, cPos(2) + PP(i,2)*0.01, cPos(3) + PP(i,3)*0.01];
    newPP = [newPP; cPos];
end

PP = newPP;

newPP = [];
for i=1:10:size(PP,1)-1
    newPP = [newPP; PP(i,:) 0 0 atan2(PP(i+1,2)-PP(i,2), PP(i+1,1)-PP(i,1))];
end
newPP = [newPP; PP(end,:) 0 0 newPP(end,6)];

PP = newPP;

%subplot(1,2,2)
%plot(PP(:,1), PP(:,2), '.b');

%257,... is the right value
Lcar = sum(sqrt(sum(diff(PP(:,1:3)).^2,2)));

%% Distância linear total percorrida na simulação pelo ego-veículo

vehicleDims = vehicleDimensions(); %4.7m long, 1.8m wide, and 1.4m high

limits = [-2 vehicleDims.Length;
-vehicleDims.Width/2-1 vehicleDims.Width/2+1;
-1 vehicleDims.Height+1];

%xlimits = allData(1).PointClouds{1,1}.XLimits;
%ylimits = allData(1).PointClouds{1,1}.YLimits;
xlimits = [-100 100];
ylimits = [-100 100];
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
        + (ptCloud.Location(:,:,3)-pose(3)).^2 > 2500;
    
    points.GroundPoints = segmentGroundFromLidarData(ptCloud,'ElevationAngleDelta', 10);

    nonEgoGroundPoints = ~points.EgoPoints & ~points.GroundPoints & ~points2.EgoPoints;

    ptCloudSegmented = select(ptCloud, nonEgoGroundPoints,'Output','full');

    minNumPoints = 3;
    [labels, numClusters] = segmentLidarData(ptCloudSegmented, 1, 180, 'NumClusterPoints', minNumPoints);

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
    laneObjects = allData(i).LaneDetections;

    pose = PP(uint32((i-1)/10)+1,:);

    T = geotransf(pose(1),pose(2),pose(3),pose(4),pose(5),pose(6));

    for j=1:size(objects,1)
        object = objects{j,1}.Measurement;
        
        object = T*[object(1) object(2) 0 1]';

        object = object(1:3)';

        %new_points = [new_points; object];
        %new_points = [new_points; object];

        if objects{j,1}.ObjectClassID == 4 % pedestrians
            X = [X; object];
            Y = [Y; 4 0];
        end

        if objects{j,1}.ObjectClassID == 3 % bycicles
            X = [X; object];
            Y = [Y; 3 0];
        end

        if objects{j,1}.ObjectClassID == 1 % cars
            X = [X; object];
            Y = [Y; 1 0];
        end

        if objects{j,1}.ObjectClassID == 5 % barriers
            X = [X; object];
            Y = [Y; 5 0];
        end
    end

    for j=1:size(laneObjects,1)
        object = laneObjects{j,1}.Measurement;
        
        object = T*[object(1) object(2) 0 1]';

        object = object(1:3)';

        %new_points = [new_points; object];
        %new_points = [new_points; object];

        if objects{j,1}.ObjectClassID == 4 % pedestrians
            X = [X; object];
            Y = [Y; 4 1];
        end

        if objects{j,1}.ObjectClassID == 3 % bycicles
            X = [X; object];
            Y = [Y; 3 1];
        end

        if objects{j,1}.ObjectClassID == 1 % cars
            X = [X; object];
            Y = [Y; 1 1];
        end

        if objects{j,1}.ObjectClassID == 5 % barriers
            X = [X; object];
            Y = [Y; 5 1];
        end
    end

end

%% Processing
ptCloud = pointCloud(new_points);
[labels,numClusters] = pcsegdist(ptCloud,3);

new_points = [];
peds = 0;
inPeds = 0;
bikes = 0;
MovCars = 0;
StopCars = 0;
Lped1 = -1;
LStopCar1=-1;
LBarrFirst=-1;
LBarrLast=-1;

for j=1:uint8(numClusters)
    idxValidPoints = find(labels==j);

    %if size(idxValidPoints,1)<3
    %    continue
    %end

    segmentedPtCloud = select(ptCloud, idxValidPoints);

    center = mean(segmentedPtCloud.Location, 1);
    
    new_points = [new_points; center];

    classification = kNearestNeighbors(X, Y, center);
    
    if classification(1) == 1
        StopCars = StopCars + 1;
        MovCars = MovCars + 1;
    elseif classification(1) == 3
        bikes = bikes + 1;
    elseif classification(1) == 4
        peds = peds+1;
        if classification(2) == 1
            inPeds = inPeds+1;
        end
    end

    % Distância linear (desde o início da simulação) do primeiro peão encontrado na faixa (Lped1)
    if classification(1) == 4 & classification(2) == 1 & Lped1==-1
        [~, index] = sort(sum((PP(:, 1:3) - center).^2,2),1);

        p1 = PP(index(1), 1:3);

        p2 = PP(index(2), 1:3);

        pf = polyfit([p2(1),p1(1)], [p2(2),p1(2)],1);

        a = pf(1); b = pf(2);

        a2 = -1/a;
        b2 = 1/a*center(1) + center(2);

        pointX = (b2-b)/(a-a2);
        pointY = a2*pointX+b2;

        Lped1 = sqrt(pointX^2 + pointY^2);
    end

    % Distância linear (desde o início da simulação) do primeiro veículo parado encontrado na estrada (LStopCar1)
    if classification(1) == 1 & classification(2) == 1 & LStopCar1==-1
        [~, index] = sort(sum((PP(:, 1:3) - center).^2,2),1);

        p1 = PP(index(1), 1:3);

        p2 = PP(index(2), 1:3);

        pf = polyfit([p2(1),p1(1)], [p2(2),p1(2)],1);

        a = pf(1); b = pf(2);

        a2 = -1/a;
        b2 = 1/a*center(1) + center(2);

        pointX = (b2-b)/(a-a2);
        pointY = a2*pointX+b2;

        LStopCar1 = sqrt(pointX^2 + pointY^2);
    end

    %Distância linear (desde o início da simulação) do início da primeira barreira encontrada junto à estrada (LBarrFirst)
    %Distância linear (desde o início da simulação) do fim da última barreira encontrada junto à estrada (LBarrLast)
    if classification(1) == 5 & classification(2) == 0 & LStopCar1==-1
        [~, index] = sort(sum((PP(:, 1:3) - center).^2,2),1);

        p1 = PP(index(1), 1:3);

        p2 = PP(index(2), 1:3);

        pf = polyfit([p2(1),p1(1)], [p2(2),p1(2)],1);

        a = pf(1); b = pf(2);

        a2 = -1/a;
        b2 = 1/a*center(1) + center(2);

        pointX = (b2-b)/(a-a2);
        pointY = a2*pointX+b2;

        LBarrLast = sqrt(pointX^2 + pointY^2);

        if LBarrFirst==-1
            LBarrFirst = LBarrLast;
        end
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
fprintf(fileID,'%3.2f,',StopCars);
fprintf(fileID,'%3.2f,',MovCars);
fprintf(fileID,'%3.2f,',bikes);
fprintf(fileID,'%3.2f,',Lped1);
fprintf(fileID,'%3.2f,',LStopCar1);
fprintf(fileID,'%3.2f,',LBarrFirst);
fprintf(fileID,'%3.2f,',LBarrLast);
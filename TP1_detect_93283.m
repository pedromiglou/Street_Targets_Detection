close all
clear
clc
load Street_Targets_Detection/allData_93283.mat

freq_INS = 0.01; % INS frequency
freq_sensors = 0.1; % other sensors frequency

cPos = [0 0 0]; % current position
PP = []; % position list

for i=1:size(allData,2)
    cPos = cPos + allData(i).INSMeasurements{1,1}.Velocity*freq_INS;
    PP = [PP; cPos];
end

newPP = [];
for i=1:size(PP,1)-1
    newPP = [newPP; PP(i,:) atan2(PP(i+1,2)-PP(i,2), PP(i+1,1)-PP(i,1)) 0 0];
end
newPP = [newPP; PP(end,:) newPP(end,6) 0 0];

PP = newPP;

%257,... is the right value
Lcar = sum(sqrt(sum(diff(PP(:,1:3)).^2,2)));

vehicleDims = vehicleDimensions(); %4.7m long, 1.8m wide, and 1.4m high

limits = [-2 vehicleDims.Length;
-vehicleDims.Width/2-1 vehicleDims.Width/2+1;
-1 vehicleDims.Height+1];

xlimits = [-100 100];
ylimits = [-100 100];
zlimits = [0 1];
lidarViewer = pcplayer(xlimits, ylimits, zlimits);
xlabel(lidarViewer.Axes, 'X (m)');
ylabel(lidarViewer.Axes, 'Y (m)');
zlabel(lidarViewer.Axes, 'Z (m)');

all_points = [];

X = [];
Y = [];

X2 = [];
Y2 = [];

X3 = [];
Y3 = [];

peds = 0;
inPeds = 0;
bikes = 0;
MovCars = 0;
StopCars = 0;
Lped1 = -1;
LStopCar1=-1;
LBarrFirst=-1;
LBarrLast=-1;

for i=1:10:size(allData,2)-1
    pose = PP(i,:);

    T = trvec2tform(pose(1:3)) * eul2tform(pose(4:6));

    % radar and camera

    objects = allData(i).ObjectDetections;
    lanes = allData(i).LaneDetections.LaneBoundaries;

    for j=1:size(objects,1)
        object = objects{j,1}.Measurement;

        laneLimits = [lanes(1,2).LateralOffset lanes(1,1).LateralOffset];

        closeby = 0;
        if sqrt(object(1)^2+object(2)^2)<5
            closeby = 1;
            inLane = object(2) > laneLimits(1) & object(2) < laneLimits(2);
        end

        if objects{j,1}.SensorIndex==3
            moving = sqrt(object(4)^2+object(5)^2)>29 & sqrt(object(4)^2+object(5)^2)<31;
        end
        
        object = T*[object(1) object(2) 0 1]';

        object = object(1:3)';

        if closeby
            X3 = [X3; object];
            Y3 = [Y3; inLane];
        end

        if objects{j,1}.SensorIndex==3
            X2 = [X2; object];
            Y2 = [Y2; moving];
        end

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

    % LIDAR

    ptCloud = allData(i).PointClouds{1,1};

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

    minNumPoints = 1;
    [labels, numClusters] = segmentLidarData(ptCloudSegmented, 1, 180, 'NumClusterPoints', minNumPoints);

    for j=1:uint8(numClusters)
        idxValidPoints = find(labels==j);

        if size(idxValidPoints,1)<2
            continue
        end
    
        segmentedPtCloud = select(ptCloudSegmented, idxValidPoints);

        points = [];
        for k=1:size(segmentedPtCloud.Location,1)
            point = T*[segmentedPtCloud.Location(k,:) 1]';
            points = [points; point(1:3)'];
        end

        %center = mean(points, 1);
        
        all_points = [all_points; points];
    end

    ptCloud = pointCloud(all_points);
    [labels,numClusters] = pcsegdist(ptCloud,4.5);

    cluster_centroids = [];
    for j=1:uint8(numClusters)
        idxValidPoints = find(labels==j);
    
        if size(idxValidPoints,1)<2
            continue
        end
    
        segmentedPtCloud = select(ptCloud, idxValidPoints);
    
        center = mean(segmentedPtCloud.Location, 1);
        
        cluster_centroids = [cluster_centroids; center];
    end

    if size(cluster_centroids,1)>0
        view(lidarViewer, cluster_centroids)
    end

    if size(X,1)==0 | size(X2,1)==0 | size(X3,1)==0
        continue
    end

    peds = 0;
    inPeds = 0;
    bikes = 0;
    MovCars = 0;
    StopCars = 0;

    for j=1:size(cluster_centroids,1)
        centroid = cluster_centroids(j,:);
        type = kNearestNeighbors(X, Y, centroid);
    
        moving = kNearestNeighbors(X2, Y2, centroid);
    
        inLane = kNearestNeighbors(X3, Y3, centroid);
        
        if type == 1
            if moving==0
                StopCars = StopCars + 1;
            else
                MovCars = MovCars + 1;
            end
        elseif type == 3
            bikes = bikes + 1;
        elseif type == 4
            peds = peds+1;
            if inLane == 1
                inPeds = inPeds+1;
            end
        end
    
        % Distância linear (desde o início da simulação) do primeiro peão encontrado na faixa (Lped1)
        if type == 4 & inLane == 1 & Lped1==-1
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
        if type == 1 & inLane == 1 & moving==0 & LStopCar1==-1
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
        if type == 5 %& inLane == 0
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
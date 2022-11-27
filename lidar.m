close
clear
clc
load Project1/allData2.mat


xlimits = allData(1).PointClouds{1,1}.XLimits;
ylimits = allData(1).PointClouds{1,1}.YLimits;
zlimits = allData(1).PointClouds{1,1}.ZLimits;

lidarViewer = pcplayer(xlimits, ylimits, zlimits);
%Set axis labels
xlabel(lidarViewer.Axes, 'X (m)');
ylabel(lidarViewer.Axes, 'Y (m)');
zlabel(lidarViewer.Axes, 'Z (m)');

for k=1:861
    pc = allData(k).PointClouds{1,1};

    ground = segmentGroundFromLidarData(pc, 'ElevationAngleDelta', 10);

    %pc = pc.removeInvalidPoints();    

    %pc = pc.Location(~ground,:);

    points = pc.Location;

    points2 = [];
    
    for i=1:size(points,1)
        for j=1:size(points,2)
            if and(and(~isnan(points(i,j,1)),~ground(i,j)), points(i,j,3)>0.01)
                points2 = [points2; points(i,j,1) points(i,j,2) points(i,j,3)];
            end
        end
    end

    pc.removeInvalidPoints();
    %pc.Location = points2;

    %points = points(~ground(:),:);
    
    if size(pc.Location,1)>0
        %View the point cloud
        if size(points2,1)>0
            view(lidarViewer, points2);
            pause(0.2)
            k
        end
        
    end
end

%% Distância linear total percorrida na simulação pelo ego-veículo

% PP = cell2mat(arrayfun(@(S) S.INSMeasurements{1,1}.Position', allData, 'UniformOutput', false))';
% 
% %plot(PP(:,1), PP(:,2), '.b');
% axis equal
% 
% %257,... is the right value
% Lcar = sum(sqrt(sum(diff(PP).^2,2)));
% 
% hold on
% points = allData(1).PointClouds{1,1}.Location(1,:,:);
% 
% points = reshape(points, [size(points,2), size(points,3)]);
% 
% scatter3(points(:,1),points(:,2),points(:,3))
% 
% points = allData(1).PointClouds{1,1}.Location(10,:,:);
% 
% points = reshape(points, [size(points,2), size(points,3)]);
% 
% scatter3(points(:,1),points(:,2),points(:,3),"green")
% 
% view(3)

% W = cell2mat(arrayfun(@(S) S.ActorPoses.Velocity', allData, 'UniformOutput', false));
% 
% t = [allData.Time];
% 
% figure(2)
% subplot(1,3,1);
% 
% plot(t,W);
% 
% legend('v_x','v_y','v_z')
% 
% dt = diff(t);
% 
% %Wcx = diff(PP(:,1))./dt;
% %Wcy = diff(PP(:,2))./dt;
% %Wcz = diff(PP(:,3))./dt;
% 
% Wc = diff(PP')./dt';
% 
% subplot(1,3,2);
% plot(t(1:end-1),Wc);
% legend('v_x','v_y','v_z')
% 
% errV = W(:,1:end-1)' - Wc;
% 
% subplot(1,3,3);
% plot(t(1:end-1),errV);
% legend('e_v_x','e_v_y','e_v_z')


%% With Person on the Road

% close all
% 
% PP = cell2mat(arrayfun(@(S) S.ActorPoses(1).Position', sns1, 'UniformOutput', false));
% 
% plot(PP(:,1), PP(:,2), '.b');
% view(-90,90)
% 
% axis equal
% hold on
% 
% % for n=1:numel(sns1)
% %     objs=sns1(n).ObjectDetections;
% %     for i=1:numel(objs)
% %         objs{i}.Measurement';
% %     end
% % end
% 
% for n=1:numel(sns1)
%     % obtain carr transformation
%     posCar=PP(n,:)'; %car position
%     orCar=[ %car orientations
%         sns1(n).ActorPoses(1).Yaw sns1(n).ActorPoses(1).Pitch sns1(n).ActorPoses(1).Roll
%     ]*pi/180; %convert to radians
%     TCtrans = trvec2tform(posCar); % position into matrix
%     TCrot = eul2tform(orCar); % angles into matrix
%     Tcarro = TCtrans * TCrot;
%     
%     % Obtain obj transformation
%     objs = sns1(n).ObjectDetections;
%     for i=1:numel(objs)
%         posSens=objs{i}.Measurement';
%         trvec = posSens(1:3);
%         eul = posSens(4:6)*pi/180;
%         trn = rvec2tform(trvec);
%         rtn = eul2tform(eul);
%         Tobj = trn*rtn;
%         pos=Tcarro*Tobj*[0 0 0 1]';
% 
%         plot(pos(1),pos(2),'o');
%     end
% end
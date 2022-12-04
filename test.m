%PP = [cell2mat(arrayfun(@(S) S.INSMeasurements{1,1}.Position', allData, 'UniformOutput', false))', cell2mat(arrayfun(@(S) S.INSMeasurements{1,1}.Orientation', allData, 'UniformOutput', false))'];
PP = cell2mat(arrayfun(@(S) S.INSMeasurements{1,1}.Velocity', allData, 'UniformOutput', false))';
PP2 = cell2mat(arrayfun(@(S) S.ActorPoses(1).Position', allData, 'UniformOutput', false))';

axis equal
%subplot(1,2,1)
plot(PP2(:,1), PP2(:,2), '.b');
hold on;

newPP = [];

cPos = [0 0 0];

for i=1:size(PP,1)
    %newPP = [newPP; mean(PP(max(1, i-10):min(i+10, size(PP,1)),:),1)];
    cPos = [cPos(1) + PP(i,1)*0.01, cPos(2) + PP(i,2)*0.01, cPos(3) + PP(i,3)*0.01];
    newPP = [newPP; cPos];
end

PP = newPP;

% newPP = zeros([size(PP,1),6]);
% for i=1:size(PP,1)-1
%     newPP(i,:) = [PP(i,:) 0 0 atan2(PP(i+1,2)-PP(i,2), PP(i+1,1)-PP(i,1))];
% end
% 
% PP = newPP;

%subplot(1,2,2)
plot(PP(:,1), PP(:,2), '.r');

%257,... is the right value
Lcar = sum(sqrt(sum(diff(PP(:,1:3)).^2,2)));
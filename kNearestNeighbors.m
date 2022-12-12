function y = kNearestNeighbors(X,Y,x)
%MINIMUMDISTANCETO Summary of this function goes here
%   Detailed explanation goes here
    k = 1;

    distances = (X-x).^2;

    [distances, order] = sort(sqrt(distances(:,1)+distances(:,2)+distances(:,3)));
    X = X(order,:);
    Y = Y(order);

    X = X(1:k,:);
    Y = Y(1:k);
    distances = distances(1:k);

    y = mode(Y);
end

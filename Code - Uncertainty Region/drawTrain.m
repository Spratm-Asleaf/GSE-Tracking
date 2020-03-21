function [] = drawTrain( Train )
%DRAWTRAIN Summary of this function goes here
%   Detailed explanation goes here
    hold on;%figure;
    
    Len = length(Train);
    
    for i = 1:Len
        P = Train{i}.Position;
        L = Train{i}.Length;
        W = Train{i}.Width;
        Ratio = Train{i}.SensorPosRatio;
        Theta = Train{i}.Heading;
        Type = Train{i}.Type;
        
        Px = [-W/2 W/2 W/2 -W/2 -W/2];
        Py = [  L * (1-Ratio)...
                L * (1-Ratio)...
               -L * Ratio...
               -L * Ratio...
                L * (1-Ratio)
        ];

        rectPoint = [Px; Py];

        outLine = R(Theta)'*rectPoint;
        van = repmat(P,1,length(outLine(1,:))) + outLine;
        
        if strcmp(Type, 'Master') == 1
            plot(van(1,:), van(2,:), 'r', 'linewidth', 2);
            hold on;
        else
            plot(van(1,:), van(2,:), 'b');
            hold on;
        end

        if strcmp(Type, 'Master') == 1
            plot(P(1),P(2),'ro','MarkerFaceColor','r');
        else
            plot(P(1),P(2),'go','MarkerFaceColor','g');
        end

        if strcmp(Type, 'Master') == 0
            linkFront = R(Theta)'*[0 0; Train{i}.Length * (1-Train{i}.SensorPosRatio) Train{i}.Length * (1-Train{i}.SensorPosRatio) + Train{i}.FrontLink];
            vanLinkFront = repmat(P,1,length(linkFront(1,:))) + linkFront;

            hold on;
            plot(vanLinkFront(1,:), vanLinkFront(2,:),'r');
        end

        if strcmp(Type, 'Tail') == 0
            linkBack = R(Theta)'*[0 0; -Train{i}.Length * Train{i}.SensorPosRatio -(Train{i}.Length * Train{i}.SensorPosRatio + Train{i}.BackLink)];
            vanLinkBack = repmat(P,1,length(linkBack(1,:))) + linkBack;

            hold on;
            plot(vanLinkBack(1,:), vanLinkBack(2,:),'r');
        end
    end
    
    axis equal;
end


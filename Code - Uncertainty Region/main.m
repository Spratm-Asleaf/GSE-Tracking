% Source: https://github.com/Spratm-Asleaf/GSE-Tracking
% Author: WANG Shixiong (s.wang@u.nus.edu)
% Affiliate: Department of Industrial SystemsEngineering and Management, National University of Singapore, Singapore 117576

clear all;
close all;
clc;

%% Position and Headings of the train
% Headings of the train
theHeading = [0.785398163397448*2;0.504209534826583;0.908899102895756;1.285979618848666;0.914473553499528;1.05521125265042;0.596338851572222;0.940309222709936;0.362821479574975];
% P0 (position of the leading vehicle)
P0 = [10; 20];

%% Simulation Parameter
SimuN = 500;            % Simulation Rounds
DrawN = 8;              % Draw how many rounds
VarP = 0.7*(10^-2)^2;            % Variance of Positioning (see Table 1 in the paper)
VarTheta = 0.0025*(pi/180)^2;      % Variance of Heading (see Table 1 in the paper)
P0 = P0 + sqrt(VarP)*randn(2,SimuN);
theHeading = theHeading + sqrt(VarTheta) * randn(length(theHeading),SimuN);

cornerPoint = [];

%% Monte Carlo Simulation
for simuN = 1:SimuN
    %% Number of Trailing vehicles
    N = 2;

    %% Construct the Train (define the data structure of the train)
    Train = cell(N + 1, 1);
    %{ 
    %@attribute: For each vehicle of the train, we define its attributes as:
        Type:           Master (leading vehicle); Tail (the last trailing
                        vehicle); Van (otherwise)
        ID:             Phisical permanent ID
        Mather:         The preceding vehicle (for "Master", it is "-1")
        Daughter:       The succeeding vehicle (for "Tral", it is "-1")
        Length:         The pyshical size (length)
        Width:          The pyshical size (width)
        SensorPosRatio: Where to place the sonsor (along the axis of symmetry of the vehicle)
        FrontLink:      How long the front linking rod
        BackLink:       How long the back linking rod
        Position:       The position of this vehicle in the global geodetic
                        coordinate system we defined at the airport (i.e., the coordinate x_e-O_e-y_e in the paper)
        Heading:        The position of this vehicle in the global geodetic coordinate system
    %}

    L = 1.5;
    W = 0.6;

    % For each vehicle (if i = 1, Master vehicle; if i = N+1, Tail vehicle)
    for i = 1:N+1
        % Type
        if i == 1
            Train{i}.Type = 'Master';
        elseif i == N + 1
            Train{i}.Type = 'Tail';
        else
            Train{i}.Type = 'Van';
        end

        % Phisical ID
        Train{i}.ID = i;

        % Mather and Daughter
        if i == 1
            Train{i}.Mather = -1;
            Train{i}.Daughter = i + 1;
        elseif i == N + 1
            Train{i}.Mather =  i - 1;
            Train{i}.Daughter = -1;
        else
            Train{i}.Mather = i - 1;
            Train{i}.Daughter = i + 1;
        end

        % Length, Width
        if i == 1
            Train{i}.Length = L ;%+ rand/2;
            Train{i}.Width = W ;%+ rand/2;
        else
            Train{i}.Length = L ;%+ rand/2;
            Train{i}.Width = W ;%+ rand/2;
        end

        % Where to Place the Sonsor
        if i == 1
            Train{i}.SensorPosRatio = 1/3 ;%+ rand/3;      % from the end
        else
            Train{i}.SensorPosRatio = 2/3 ;%+ rand/3;      % from the end
        end

        % How Long the Front and Back Link
        if i == 1
            Train{i}.FrontLink = 0;
            Train{i}.BackLink = 0.3 ;%+ rand/3;
        else
            Train{i}.FrontLink = 0.45 ;%+ rand/3;
            Train{i}.BackLink = 0.3 ;%+ rand/3;
        end

        % Position and Heading
        if i == 1
            Train{i}.Position = P0(:,simuN);
            Train{i}.Heading = theHeading(i,simuN) + (pi/180)*randn;
        else
            Train{i}.Position = []; % automatically determined by the algorithm
            Train{i}.Heading = theHeading(i,simuN) + (pi/180)*randn;
        end
    end

    %% Get the Positions of Every Trailing Vehicles
    % Eq. (1) in the paper
    for i = 2 : N+1
        relTheta = Train{i}.Heading - Train{Train{i}.Mather}.Heading;
        Train{i}.Position = Train{Train{i}.Mather}.Position + ...
                            R(Train{Train{i}.Mather}.Heading)'*( ...
                                    [0; -(Train{Train{i}.Mather}.Length * Train{Train{i}.Mather}.SensorPosRatio + Train{Train{i}.Mather}.BackLink)] + ...
                                    R(relTheta)'*[0; -(Train{i}.Length * (1-Train{i}.SensorPosRatio) + Train{i}.FrontLink)] ...
                            );
    end

    %% Plot Out the Whole Train
    if simuN <= DrawN
        drawTrain(Train);
    end

    %% Where to plot the corner point
    I = N+1; % {N+1} if the last van

    % The back corner
    isRight = 1;            % is the coner on the right side of a van ?
    RLSign = 2*isRight - 1;

    p_r = [RLSign*Train{I}.Width/2; -Train{I}.Length * Train{I}.SensorPosRatio];      % The relative position of this corner
    % Eq. (6) in the paper
    P = Train{I}.Position + R(Train{I}.Heading)'*p_r;                                 % The global position of this corner
    
    % record the corner point
    cornerPoint = [cornerPoint P];

    if simuN <= DrawN
        hold on;
        h = plot(P(1), P(2), 'mhexagram', 'Markersize', 10,'MarkerFaceColor','m');
    end
end

%% Draw uncertainty region
% Calculate the variance of the corner point
var_X = var(cornerPoint(1,:));   % x
var_Y = var(cornerPoint(2,:));   % y
center = mean(cornerPoint,2);
radius = 3*sqrt(sqrt(var_X^2 + var_Y^2))    % unit: cm

t=0:pi/100:2*pi;
x = center(1) + (radius)*sin(t);
y = center(2) + (radius)*cos(t);
h1 = plot(x,y,'k-','linewidth',2.5);

legend([h, h1],'Coner Point','Uncertainty Region');
xlabel('x (m)','fontsize',14);
ylabel('y (m)','fontsize',14);
set(gca,'fontsize',14);

% Source: https://github.com/Spratm-Asleaf/GSE-Tracking

%{ 
 Read me:
 Reference code to parse GPGGA sentence from RTK positioning board
 Data collected at 21:16 on Jan 15, 2019 at National University of Singapore

 For mathmatics: see "[English] Coordinate Transformation.pdf" ...
                 or "[Chinese] Coordinate Transformation.pdf"
%}
clear all;
close all;
clc;

% Load Data File
% Note: this data file contains two kinds of RTK sentences:
%(1) GPGGA sentence; (2) FVI sentence.
% We only care about GPGGA sentence here
filename = 'RTK-Raw-Data.txt';
fid = fopen(filename,'r');
if fid<=0
    error('Error in opening data file');
end

% Location Variables
LLH = [];   % in longitude-latitude-height (note: height is ellipsoid height) 
XYZ = [];   % in GWS84 (World Geodetic System)
ENU = [];   % in east-north-up (Local tangent plane coordinate)

% GPS Working Status
% (0=invalid; 1=GPS; 2=Difference GPS; 4=RTK Fix; 5=RTK Float)
Status = [];

% SVY21 Plane Coordinate System of Singapore (it is kind of ENU)
% (see: https://www.sla.gov.sg/sirent/CoordinateSystems.aspx)
phi0 = 103+49/60+31.975227/3600;
lamda0 = 1+22/60+02.915414/3600;
height0 = 26.824;
% The reference spheroid of SVY21 is the WGS84 ellipsoid
a0 = 6378137.00;
e0 = 0.081819191;
R0 = a0./sqrt(1-e0^2 * sin(phi0)^2);

% Origin of SVY21 in WGS84
SVY21Origin = [
                (R0 + height0)*cos(phi0)*cos(lamda0);
                (R0 + height0)*cos(phi0)*sin(lamda0);
                (R0 * (1-e0^2) + height0)*sin(phi0);
];

% Transformation matrix
M = [
    -sin(lamda0)                cos(lamda0)             0
    -sin(phi0)*cos(lamda0)     -sin(phi0)*sin(lamda0)   cos(phi0)
     cos(phi0)*cos(lamda0)      cos(phi0)*sin(lamda0)   sin(phi0)
    ];

% the k^{th} GPGGA sentence
k = 0;
while ~feof(fid)
    % read data file
    msg = fgetl(fid);
    msg = [msg ','];    % format data

    len = length(msg);
    data = cell(16,1);  % GPGGA has in total 16 data field (see "GPGGA Data Format.pdf")

    substr = [];        % data field in string format

    j = 1;     
    flag_isGPGGA = true;
    for i = 1:len
        if msg(i) == '*' % format data
            msg(i) = ',';
        end
        
        % extract data field
        if msg(i) ~= ','
            substr = [substr msg(i)];
            continue;
        end
        
        % read the message head
        if j == 1
            % we only parse GPGGA statements here
            if strcmp(substr, '$GPGGA') == 0
                flag_isGPGGA = false;
                break;
            end
        end
        
        % the j^{th} data field (in total: 16)
        data{j} = substr;
        j = j+1;
        
        substr = [];
    end
    
    % if it is not GPGGA statement, no longer parse
    if ~flag_isGPGGA
        flag_isGPGGA = true;
        continue;
    end
    
    k = k + 1;
    
    %% RTK Status
    Status = [Status; data(7)];

    %% Obtain LLH
    lon = str2double(data{3});
    lat = str2double(data{5});
    
    lonDeg = floor(lon/100);
    latDeg = floor(lat/100);
   
    lonMin = lon - 100*lonDeg;
    latMin = lat - 100*latDeg;
    
    phi = (lonDeg + lonMin/60)*pi/180;   % long
    lamda = (latDeg + latMin/60)*pi/180; % lat
    height = str2double(data{10}) + str2double(data{12});
    
    LLH = [LLH [phi; lamda; height]];

    %% Obtain XYZ of GWS84
    a = 6378137.00;
    e = 0.081819191;
    R = a./sqrt(1-e^2 * sin(phi)^2);

    XYZ = [XYZ [
                (R + height)*cos(phi)*cos(lamda);
                (R + height)*cos(phi)*sin(lamda);
                (R * (1-e^2) + height)*sin(phi);
                ];
           ];

    %% Obtain ENU of SVY21
    ENU = [ENU M*(XYZ(:,k) - SVY21Origin)];
end

% Experiemnt Coordinate (translated from SVY21, just change the origin)
ENU = ENU - ENU(:,1);

%% Plot 
range = 400:480;    % only plot a small piece of trajectory
cla;
plot3(ENU(1,range),ENU(2,range),ENU(3,range),'r');
xlabel('x');
ylabel('y');
zlabel('z');

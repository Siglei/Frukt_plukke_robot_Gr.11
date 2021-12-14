clc; 
clear all;

%% Lager kart
gaard = zeros(120, 170);

%Lager okkuperte celler for å simulere epletrær
gaard(20:100,[21:30 51:60 81:90 111:120 141:150]) = 1;


%% Lattice planner
% lp = Lattice(gaard, 'grid', 4, 'root', [4 4 0], 'inflate', 2);
% lp.plan('iterations', 60, 'cost', [1 20 20])
% lp2 = Lattice(gaard, 'grid', 4, 'root', [4 4 0], 'inflate', 2);
% save lp
% save lp2
% lp.plot()
%% Henter lattice planner
load lp
lp.plot()
%Hvis lp ikke eksisterer må seksjonen over kjøres
%% Lattice Goal
%Finner rute fra start posisjon og reting [x, y, retning i rad] til slutt posisjon or retning [x, y, retning i rad]
p = lp.query( [4 4 0], [68 52  pi/2])
%Henter ut x og y kordinatene
p1 = p(:,1);
p2 = p(:,2);
%Setter x og y kordinantene sammen til en matrise med waypoints
path = [p1,p2]/10
path_plot = path*10

%lp.plot(p)
%% Controller
%Setter start og slutt posisjon for kontroller
robotInitialLocation = path(1,:);
robotGoal = path(end,:);
%Begynnelses orientering, 0 betyr i posetiv x-retning
initialOrientation = 0;
%Setter start Pose til den mobile basen
robotCurrentPose = [robotInitialLocation initialOrientation]';
%Setter verdier for den kinematiske modellen til den mobilebasen
robotBase = differentialDriveKinematics("TrackWidth", 0.43, "VehicleInputs", "VehicleSpeedHeadingRate");

%Viser planlagt rute til controller
% figure
% plot(path(:,1), path(:,2),'k--d')
% xlim([0 170])
% ylim([0 120])
%Setter algorytme for kontroller
controller = controllerPurePursuit;
%Setter ruten kontroller skal følge
controller.Waypoints = path;
%Setter ønsket lineær hastighet
controller.DesiredLinearVelocity = 0.5;
%Setter ønsket svinghastighet / vinkelhastighet
controller.MaxAngularVelocity = 2.0;
%Setter hvor langt frem kontrolleren skal se for å beregne Pure Pursuit
controller.LookaheadDistance = 0.5;
%Hvor nerme mål vi må komme for å være fornøyd
goalRadius = 0.1;
%Setter avstand til mål
distanceToGoal = norm(robotInitialLocation - robotGoal);

%% Simulering
% Setter tidssteg
sampleTime = 0.1;
vizRate = rateControl(1/sampleTime);

% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robotBase.TrackWidth*10;

while( distanceToGoal > goalRadius )
    
    % Compute the controller outputs, i.e., the inputs to the robot
    [v, omega] = controller(robotCurrentPose);
    
    % Get the robot's velocity using controller inputs
    vel = derivative(robotBase, robotCurrentPose, [v omega]);
    
    % Update the current pose
    robotCurrentPose = robotCurrentPose + vel*sampleTime 
    
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:))
    
    % Update the plot
    hold off
    
    % Plot path each instance so that it stays persistent while robot mesh
    % moves
    lp2.plot()
    hold all
    plot(path_plot(:,1), path_plot(:,2),'k--d');
    
    
    
    % Plot the path of the robot as a set of transforms
    plotTrVec = [robotCurrentPose(1:2); 0]*10;
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 170])
    ylim([0 120])
    
    waitfor(vizRate);
end
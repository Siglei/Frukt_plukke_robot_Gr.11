%% Plukker eple
clc;clear;
%addpath må kjøres i command Window første gang scriptet kjøres etter
%matlab er startet på nytt:
%addpath rtb common smtb

%Hvis du får error på odom.Pose.Pose.Position må du skrive rosshutdown i
%command Window og kjøre igjen, da vil det fungere

%% Lager kart
% %Laging av kart og planner må gjennomføres første gang scriptet kjøres på
% %maskinen for å lagre lp og lp2 til disk. Hvis lp og lp2 er lagret til disk
% %trengs bare load lp å kjøres
% gaard = zeros(120, 170);
% 
% %Lager rader med epletrær
% gaard(20:100,[21:30 51:60 81:90 111:120 141:150]) = 1;
% 
% % Lattice planner
% lp = Lattice(gaard, 'grid', 4, 'root', [4 4 0], 'inflate', 2);
% lp.plan('iterations', 60, 'cost', [1 20 20])
% lp2 = Lattice(gaard, 'grid', 4, 'root', [4 4 0], 'inflate', 2);
% %Viser planlagt gård
% %lp.plot()
% save lp
% save lp2
%% Load
%Hvis ikke load funker, fjern kommenteringen over og kjør seksjonen over
load lp

%% Lager arm
import ETS3.*
%Arm lengder
L1=0.1564;
L2=0.1284;
L3=0.410;
L4=0.2084;
L5=0.1059;
L6=0.1059;
L7=0.0615;

%DH parametere for arm
L(1) = Link('a', 0, 'd', (156.43+128.38), 'alpha', -pi/2 ,'offset', 0, 'standard');
L(2) = Link('a', -410, 'd', -5.38, 'alpha', pi ,'offset', pi/2, 'standard');
L(3) = Link('a', 0, 'd', -6.38, 'alpha', pi/2 ,'offset', pi/2, 'standard');
L(4) = Link('d', -(208.43+105.93), 'a', 0, 'alpha', pi/2 ,'offset', pi, 'standard');
L(5) = Link('d', 0, 'a', 0, 'alpha', pi/2 ,'offset', pi, 'standard');
L(6) = Link('d', -(105.93+61.53), 'a', 0, 'alpha', pi ,'offset', pi, 'standard');

%Setter sammen DH parametere for å lage en robot
robot = SerialLink(L)

%% Arm Kinematikk
%Start Pose transformasjons matrise fås igjennom forward kinematics
T_robot_0 = robot.fkine([0,0,0,0,0,0])
%Vinkler for at arm skal se etter eple
q_seEple = [1,-pi/4,-pi/2,0,-pi/4,pi/2]
%Transformasjons matrise for base til end-effektor
bTe= robot.fkine(q_seEple)
%Kamera ser eple 310mm foran og 31mm opp fra kamera
cTep = transl(0,31,310)
%Kamera er plassert på endeffektor 7 cm opp langs y aksen til endeffector
eTc = transl(0,70,0)
%Base tilPlukk Epler (bTep)
bTep = bTe.T * eTc * cTep
%Base til Legg epler i kurv
bTk = robot.fkine([0,deg2rad(20),deg2rad(105),0,0,pi/2])

%Vinkler til armen for kjøre Posisjon
q_home = [0,-pi/4,-pi/2,0,-pi/4,pi/2];
%Start vinkler får ved å ta inverse kinematics på transformasjonsmatrien
q0 = robot.ikine(T_robot_0, 'mask', [1 1 1 0 0 1]) 
%Finner vinkler for at endefektor skal være i samme posisjon som eple
q1 = robot.ikine(bTep,'q0', q0(end,:), 'mask', [1 1 1 0 0 1])
%Vinkler for at endefektor skal være i posisjon over kurv
q2 = robot.ikine(bTk, 'q0', [0,deg2rad(20),deg2rad(105),0,0,pi/2])

%% Bane planlegging
%Tidssteg
t = [0:0.05:2]';
%Bane fra rett arm til kjøre posisjon (home)
m0 = mtraj(@tpoly, q0,q_home,t);
%Bane fra home til eple
m1 = mtraj(@tpoly, q_home, q_seEple, t);
m11 = mtraj(@tpoly, q_seEple, q1, t);
%Bane fra eple til kurv
m2 = mtraj(@tpoly, q1, q2, t);
%Bane fra kurv til home
m3 = mtraj(@tpoly, q2, q_home, t);

%Matlab simulering av arm bevegelse
% robot.plot(m0)  %Bane fra rett arm til kjøreposisjon
% robot.plot(m1)  %Bane fra kjøreposisjon se etter eple posisjon
% robot.plot(m11) %Bane fra se etter eple posisjon til plukke eple posisjon
% robot.plot(m2)  %Bane fra plukke eple posisjon til kurv
% robot.plot(m3)  %Bane fra kurv til kjøreposisjon

%% Sammenligning av Joint-space motion og Cartesian motion
% %Plotter Joint-space trajectory
% figure(1)
% plot(m11)
% title('Joint angles vs time:')
% xlabel('Time(s)') 
% ylabel('Joint coordinates (rad, m)')
% %Cartesian motion
% Ts = ctraj(bTe, SE3(bTep), length(t)); %bruker samme transformasjonsmatriser
% %plotter
% figure(2)
% plot(t, Ts.transl);
% title('Cartesian position vs time:')
% xlabel('Time(s)') 
% ylabel('Position  (m)')

%% Simulering i ROS
%Starter ROS
rosinit
global odom

sub_odom = rossubscriber("/fruit_robot/odom",@odom_callback);
%Mobil Base controller
[pub_vel,msg_vel] = rospublisher('/fruit_robot/cmd_vel','geometry_msgs/Twist');

%Arm controller
[pub_q1,msg_q1] = rospublisher('/fruit_robot/joint_1_controller/command','std_msgs/Float64');
[pub_q2,msg_q2] = rospublisher('/fruit_robot/joint_2_controller/command','std_msgs/Float64');
[pub_q3,msg_q3] = rospublisher('/fruit_robot/joint_3_controller/command','std_msgs/Float64');
[pub_q4,msg_q4] = rospublisher('/fruit_robot/joint_4_controller/command','std_msgs/Float64');
[pub_q5,msg_q5] = rospublisher('/fruit_robot/joint_5_controller/command','std_msgs/Float64');
[pub_q6,msg_q6] = rospublisher('/fruit_robot/joint_6_controller/command','std_msgs/Float64');

q_temp = [0,0,0,0,0,0];

%Antall bevegelser i hver bane
q0_moves = size(m0,1);
q1_moves = size(m1,1);
q11_moves = size(m11,1);
q2_moves = size(m2,1);
q3_moves = size(m3,1);
%Initialiserer Indexer for hver bane
index0 = 1;
index1 = 1;
index11 = 1;
index2 = 1;
index3 = 1;
%Stopper robot
msg_vel.Linear.X = 0.0;
msg_vel.Angular.Z = 0.0;
send(pub_vel,msg_vel)

%Setter arm i start posisjon:
rate = robotics.Rate(10);
kjor = true;
while kjor
    odom.Pose.Pose.Position
    %Stepper igjnennom banen for armen
    %Fra Home til eple
    if index0 <= q0_moves
        q_temp = m0(index0,:);
        index0 = index0 +1;  
    else
        kjor = false;
    end
    %Setter vinkel verdier for hvert ledd inn i rett ut melding
    msg_q1.Data = q_temp(1);
    msg_q2.Data = q_temp(2);
    msg_q3.Data = q_temp(3);
    msg_q4.Data = q_temp(4);
    msg_q5.Data = q_temp(5);
    msg_q6.Data = q_temp(6);    
    %Sender meldingene om vinklene ut på ros-netverket
    send(pub_q1,msg_q1)
    send(pub_q2,msg_q2)
    send(pub_q3,msg_q3)
    send(pub_q4,msg_q4) 
    send(pub_q5,msg_q5)
    send(pub_q6,msg_q6)
     
    waitfor(rate);
end


%% Finner rute til ønsket mål
%start posisjon og reting [x, y, retning i rad], slutt posisjon or retning [x, y, retning i rad]
p = lp.query( [4 4 0], [68 52  pi/2])
%Henter ut x og y posisjon for å kunne gi dette til konroller
p1 = p(:,1);
p2 = p(:,2);
path = [p1,p2]/10
path_plot = path * 10;
%Viser planlagt rute
%lp.plot(p)

%% controller
%Setter start og slutt posisjon for kontroller
robotInitialLocation = path(1,:);
robotGoal = path(end,:);
%Begynnelses orientering, 0 betyr i posetiv x-retning
initialOrientation = 0;
%Setter start Pose til den mobile basen
robotCurrentPose = [robotInitialLocation initialOrientation]';
%Setter verdier for den kinematiske modellen til den mobilebasen
robotBase = differentialDriveKinematics("TrackWidth", 0.43, "VehicleInputs", "VehicleSpeedHeadingRate");


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
% Determine vehicle frame size to most closely represent vehicle with plotTransforms
frameSize = robotBase.TrackWidth*10;
%Viser planlagt rute til controller
figure
lp2.plot()
hold all
plot(path_plot(:,1), path_plot(:,2),'k--d');
plotTrVec = [robotCurrentPose(1:2); 0]*10;
plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
light;
xlim([0 170])
ylim([0 120])

%% Initialize the simulation loop

% Setter hvor raskt kontrolleren skal oppdatere
sampleTime = 0.10;
vizRate = rateControl(1/sampleTime);

%Skjører kontroller til mål er nådd
while( distanceToGoal > goalRadius )
    odom.Pose.Pose.Position

    % Compute the controller outputs, i.e., the inputs to the robotBase
    [v, omega] = controller(robotCurrentPose)
    %Fart til mobile basen
	msg_vel.Linear.X = v;
	msg_vel.Angular.Z = omega;
    
    %Sender meldingen om den mobile basens hastig het og retning
    send(pub_vel,msg_vel)
    %Henter basens rotasjon og posisjon
    %Posisjon
    odomMsg = receive(sub_odom,3);
    pose = odomMsg.Pose.Pose;
    x = -pose.Position.X;
    y = -pose.Position.Y;
    %rotasjon
    quat = pose.Orientation;
    angles = quat2eul([quat.W quat.X quat.Y quat.Z]);

    % Update the current pose
    robotCurrentPose = [x;y;angles(1)]
    % Re-compute the distance to the goal
    distanceToGoal = norm(robotCurrentPose(1:2) - robotGoal(:))
    % Update the plot
    hold off    
    lp2.plot()
    hold all
    plot(path_plot(:,1), path_plot(:,2),'k--d');
    %Plot MobilBase
    plotTrVec = [robotCurrentPose(1:2); 0]*10;
    plotRot = axang2quat([0 0 1 robotCurrentPose(3)]);
    plotTransforms(plotTrVec', plotRot, "MeshFilePath", "groundvehicle.stl", "Parent", gca, "View","2D", "FrameSize", frameSize);
    light;
    xlim([0 170])
    ylim([0 120])
    
    waitfor(vizRate);
end
%Stopper robot
msg_vel.Linear.X = 0.0;
msg_vel.Angular.Z = 0.0;
send(pub_vel,msg_vel)
%% Plukker et eple
%Setter hvor mange ganger i sekundet komandoer skal bli sendt
rate = robotics.Rate(10);
kjor = true;

while kjor 
    odom.Pose.Pose.Position
    %Stepper igjnennom banen for armen
    %Fra home til SeEple
    if index1 <= q1_moves
        q_temp = m1(index1,:);
        index1 = index1 +1;  
    %Fra SeEple til eple
    elseif index11 <= q11_moves
        if index11 == 1 
            pause(2)
        end
        q_temp = m11(index11,:);
        index11 = index11 +1;  
    %Fra eple til kurv
    elseif index2 <= q2_moves
        if index2 == 1 
            pause(2)
        end
        q_temp = m2(index2,:);
        index2 =  index2 +1;  
    %Fra kurv til home
    elseif index3 <= q3_moves
        if index3 == 1 
            pause(2)
        end
        q_temp = m3(index3,:);
        index3 =  index3 +1;
    %Resetter indexer for å kunne plukke nytt eple
    else
        index1 = 1;
        index11 = 1;
        index2 = 1;
        index3 = 1;
        kjor = false;
    end
    
    %Setter vinkel verdier for hvert ledd inn i rett ut melding
    msg_q1.Data = q_temp(1);
    msg_q2.Data = q_temp(2);
    msg_q3.Data = q_temp(3);
    msg_q4.Data = q_temp(4);
    msg_q5.Data = q_temp(5);
    msg_q6.Data = q_temp(6);    
    %Sender meldingene om vinklene ut på ros-netverket
    send(pub_q1,msg_q1)
    send(pub_q2,msg_q2)
    send(pub_q3,msg_q3)
    send(pub_q4,msg_q4) 
    send(pub_q5,msg_q5)
    send(pub_q6,msg_q6)
     
    waitfor(rate);
end
%%Stopper Ros
rosshutdown

function odom_callback(src,msg)
    global odom
    odom = msg; 
end
close all;
clc;
clear;

Dobot_1 = Dobot();
%animate(Dobot_1.model,deg2rad( [-90,-60,100,-130,-90, 0]));


%% 
close all;
clc;
clear;

% Generate Serial Link based on derived DH parameters
L1 = Link('d',0.135,'a',0,'alpha',-pi/2,'qlim', deg2rad([-135,135]), 'offset', 0);
L2 = Link('d',0,'a',0.139,'alpha',0,'qlim', deg2rad([5,80]), 'offset', -pi/2);
L3 = Link('d',0,'a',0.160,'alpha',0,'qlim', deg2rad([15,170]), 'offset', 0);
L4 = Link('d',0,'a',0.05,'alpha',-pi/2,'qlim', deg2rad([-90,90]), 'offset', 0);
L5 = Link('d',0.025,'a',0,'alpha',0,'qlim', deg2rad([-85,85]), 'offset', 0);

Dobot_1 = SerialLink([L1 L2 L3 L4 L5],'name', 'MyDobot');
workspace = [-1 1 -1 1 -1 1]; 
scale = 0.5;        
q = deg2rad([0, 60, 65, -35, 0]);     
Dobot_1.plot(q,'workspace',workspace,'scale',scale);

Dobot_1.teach();


%%
% 2.2     Define our first end-effector pose as a 4x4 Homogeneous Transformation Matrix: 
T1 = transl(0.5,-0.4,0.5); 
% 2.3     Solve the inverse kinematics to get the required joint angles 
q1 = deg2rad([0, 0, 0, 0, 0]);

% 2.4     Define the second end-effector pose as a 4x4 Homogeneous Transformation Matrix: 
T2 = transl(0.1,0.3,0.4); 
% 2.5     Solve the inverse kinematics to get the required joint angles 
q2 = deg2rad([0, 85, 33, -29, 0]);

steps = 50;

% Method 2: Trapezoidal Velocity Profile
s = lspb(0,1,steps);
qMatrix = nan(steps,5);
for i = 1:steps
    qMatrix(i,:) = (1-s(i))*q1 + s(i)*q2;
    
    qMatrix(i,4) = (0.5*pi) - qMatrix(i,2) - qMatrix(i,3);
end

Dobot_1.plot(qMatrix,'trail','r-') 


%% 

stepRads = deg2rad(10);
qlim = Dobot_1.qlim;
% Don't need to worry about joint 6
pointCloudeSize = prod(floor((qlim(1:5,2)-qlim(1:5,1))/stepRads + 1));
pointCloud = zeros(pointCloudeSize,3);
counter = 1;
tic

for q1 = qlim(1,1):stepRads:qlim(1,2)
    for q2 = qlim(2,1):stepRads:qlim(2,2)
        for q3 = qlim(3,1):stepRads:qlim(3,2)
            q4 = (0.5*pi) - q2 - q3;
                q5 = 0;
                q = [q1,q2,q3,q4,q5];
                tr = Dobot_1.fkine(q);                        
                pointCloud(counter,:) = tr(1:3,4)';
                counter = counter + 1; 
                if mod(counter/pointCloudeSize * 100,1) == 0
                    display(['After ',num2str(toc),' seconds, completed ',num2str(counter/pointCloudeSize * 100),'% of poses']);
                end
        end
    end
end

plot3(pointCloud(:,1),pointCloud(:,2),pointCloud(:,3),'r.');


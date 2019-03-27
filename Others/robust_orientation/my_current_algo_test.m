clc;
clear;
close all;

% Joint solution
% solIKs = [-0.6920, -0.5793, 0.6850, 2.3451, 1.8846, -0.6290, -1.0881];
% solIKs = [1.24779,0.834836,-1.36595,2.23806,2.14386,2.07934,-0.0476521];
% solIKs = [1.0363, 0.1455, -1.1386, 2.2603, -1.4137, -1.4241, 2.8591];
% solIKs = [-0.4121    0.6217    2.6749    1.1558 2.4453    2.0793   -0.4956];
solIKs = [0.6589, 0.7006,- 2.3075, 1.8310, -1.5773, -1.5560, 2.2785];

% Left Arm
%             Theta d     a         alpha r/p  theta offset
add_len = 0.15;
Ll(1) = Link([0    0.27035  0.069  -pi/2  0    0], 'standard'); % start at joint s0 and move to joint s1
Ll(2) = Link([0    0        0       pi/2  0 pi/2], 'standard'); % start at joint s1 and move to joint e0
Ll(3) = Link([0    0.36435  0.0690 -pi/2  0    0], 'standard'); % start at joint e0 and move to joint e1
Ll(4) = Link([0    0        0       pi/2  0    0], 'standard'); % start at joint e1 and move to joint w0
Ll(5) = Link([0    0.37429  0.010  -pi/2  0    0], 'standard'); % start at joint w0 and move to joint w1
Ll(6) = Link([0    0        0       pi/2  0    0], 'standard'); % start at joint w1 and move to joint w2
Ll(7) = Link([0    0.229525+add_len 0       0     0    0], 'standard'); % start at joint w2 and move to end-effector

% Create the Robots Baxter_L(left arm) and Baxter_R(right arm)
Baxter_l = SerialLink(Ll, 'name', 'Baxter_L', 'base' , ...
                      transl(0.024645, 0.219645, 0.118588) * trotz(pi/4)...
                      * transl(0.055695, 0, 0.011038));
                  
% Solve forward kinematics
forward_kin = Baxter_l.fkine(solIKs);
fk = [forward_kin.n,forward_kin.o,forward_kin.a,forward_kin.t];

% Rotation quaternion
rot_quat = rotm2quat(fk(1:3,1:3));

% Load all joint solutions
str_join = ["../utilities/data_files/left_ik", num2str(1), ".txt"];
str = join(str_join, "");
ik_array = load(str);


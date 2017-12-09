
clear all
close all
clc
theta1= sym('theta1','real');
theta2= sym('theta2','real');
theta3= sym('theta3','real');
theta4= sym('theta4','real');
theta5= sym('theta5','real');
theta6= sym('theta6','real');
theta7= sym('theta7','real');
L1= sym('L1','real');
L2= sym('L2','real');
W = sym('W','real');
Pw = sym('Pw','real');
Ps = sym('Ps','real');
pi = sym('pi');
deg2rad=pi/180;
%%for left arm
link_number = [1,2,3,4,5,6,7]';
theta = [theta1+pi-(32.94)*deg2rad,theta2+pi/2-(28.54)*deg2rad,theta3+pi-(53.6)*deg2rad,theta4,theta5-pi/2,theta6+pi/2,theta7+pi]';
d = [0,0,0,L1,0,L2,0]';
alpha = [pi/2,pi/2,pi/2,pi/2,-pi/2,-pi/2,pi/2]';
a = [0,0,0,0,0,0,0]';

DHParameters = [link_number,theta,d,alpha,a];
disp(DHParameters)%left arm dh parmeters
Rotxxl = [1 0 0 0;...
         0 cos(132.5*deg2rad) -sin(132.5*deg2rad) 0;...
         0 sin(132.5*deg2rad) cos(132.5*deg2rad) 0;...
         0 0 0 1];
Rotzyl = [cos(45*deg2rad) 0 sin(45*deg2rad) 0;...
         0       1 0       0;...
         -sin(45*deg2rad) 0 cos(45*deg2rad) 0;...
         0 0 0 1];
Rotzzl = [cos(90*deg2rad) -sin(90*deg2rad) 0 0;...
         sin(90*deg2rad) cos(90*deg2rad) 0 0;...
         0 0 1 0;...
         0 0 0 1];
TbaseL = double(Rotxxl*Rotzyl*Rotzzl)
for n=1:7
    T(:,:,n) = Fk(theta(n),d(n),alpha(n),a(n));
end
TT(:,:,1) = T(:,:,1);
for n=2:7
TT(:,:,n) = TT(:,:,n-1)*T(:,:,n);
end
T07L = simplify(TT(:,:,7));
Tbase7left= simplify(TbaseL*T07L);
EndeffectorLeftarm = Tbase7left(1:3,4)

% % inverse kinematics for left arm
link_number = [1,2,3,4,5,6,7]';
theta = [theta1,theta2,theta3,theta4,theta5,theta6,theta7]';
d = [0,0,0,L1,0,L2,0]';
alpha = [pi/2,pi/2,pi/2,pi/2,-pi/2,-pi/2,pi/2]';
a = [0,0,0,0,0,0,0]';
% % alpha = [alpha1,alpha2,alpha3,alpha4,alpha5,alpha6,alpha7]';
% % a = [a1,a2,a3,a4,a5,a6,a7]';
% % d = [d1,d2,d3,d4,d5,d6,d7]';

for n=1:7
    T(:,:,n) = Fk(theta(n),d(n),alpha(n),a(n));
end
T01 = T(:,:,1);
T12 = T(:,:,2);
T23 = T(:,:,3);
T34 = T(:,:,4);
T45 = T(:,:,5);
T56 = T(:,:,6);
T67 = T(:,:,7);


W = sqrt(sum(Pw -Ps).^2);
cos(theta4) = (L1^2 + L2^2 - W^2)/(2*L1*L2);
sin(theta4) = sqrt(1-(cos(theta4))^2);
theta4 = pi- atan2(sin(theta4),cos(theta4))

T04=T01*T12*T23*T34;
Pe =T04(1:3,4);
Pey=Pe(2);
Pex=Pe(1);
Pez=Pe(3);
% % for theta 2
cos(theta2)=Pey/L1;
sin(theta2)=sqrt(1-(cos(theta2))^2);
theta2= atan2(sin(theta2),cos(theta2))
% %  for theta1
cos(theta1) = Pex/(L1*sin(theta2));
sin(theta1) = Pez/(L1*sin(theta2));
theta1= atan2(sin(theta1),cos(theta1)) - (pi-(32.94)*deg2rad)

T27=T23*T34*T45*T56*T67;
Pw = T27(1:3,4);
Pwx = Pw(1);
Pwy = Pw(2);
Pwz = Pw(3);
%% for theta3
cos(theta3)= Pwx/(-L2*sin(theta4));
sin(theta3)= Pwz/(L2*sin(theta4));
theta3= atan2(sin(theta3),cos(theta3))-(pi-(53.6)*deg2rad)- 2*pi

%%
 
T47 = T45*T56*T67;
r11=T47(1,1);
r12=T47(1,2);
r13=T47(1,3);
r21=T47(2,1);
r22=T47(2,2);
r23=T47(2,3);
r31=T47(3,1);
r32=T47(3,2);
r33=T47(3,3);
%% for theta6
cos(theta6)=r23;
sin(theta6)=sqrt(1-(cos(theta6))^2);
theta6= atan2(sin(theta6),cos(theta6))-pi/2

%% for theta 5
cos(theta5)= r13/sin(theta6);
sin(theta5)= -(r33/sin(theta6));
theta5 = atan2(sin(theta5),cos(theta5))+pi/2


%%for theta 7
cos(theta7)= -(r21/sin(theta6));
sin(theta7)= r22/sin(theta6);
theta7 = atan2(sin(theta7),cos(theta7))-pi+ 2*pi

%%for Pe redundancy using swivel angle

n = (Pw -Ps)/(sqrt(sum(Pw -Ps).^2));
f = (Pe - Ps)/(sqrt(sum(Pe - Ps).^2));
f1 = f - dot(f,n)*n;
a1 = [0 0 -1];
a = transpose(a1);
u = (a - dot(a,n)*n)/(sqrt(sum(a - dot(a,n)*n).^2));
x = cross(f1,u);
y = dot(n,x);
fu = dot(f1,u);
phi = atan2d(y,fu);
cos(alpha)= (L1^2 + L2^2 - (sqrt(sum(Pw -Ps).^2))^2)/(-2*(L2^2)*(sqrt(sum(Pw -Ps).^2)));
sin(alpha)= sqrt(1-(cos(alpha))^2);
R = L1*sin(alpha);
Pc = Ps + L1*cos(aplha)*n;
Pe = R*(cos(phi)*u + sin(phi)*n) + Pc

function [ transMatrix ] =Fk(theta,d,alpha,a)
    rotOldZAxis = [cos(theta) -sin(theta) 0 0;...
    sin(theta) cos(theta) 0 0;...
    0 0 1 0;...
    0 0 0 1];
    translationOldZAxis = [1 0 0 0;...
    0 1 0 0;...
    0 0 1 d;...
    0 0 0 1];
    translationNewXAxis = [1 0 0 a;...
    0 1 0 0;...
    0 0 1 0;...
    0 0 0 1];
    rotNewXAxis = [1 0 0 0;...
    0 cos(alpha) -sin(alpha) 0;...
    0 sin(alpha) cos(alpha) 0;...
    0 0 0 1];

    transMatrix = rotOldZAxis*translationOldZAxis*translationNewXAxis*rotNewXAxis;
end
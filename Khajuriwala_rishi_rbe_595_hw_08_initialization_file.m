close all
clear all
clc

% Sampling Time
T = 0.001;

% Time delay
 Tdn = 0.5;
 Td = T*Tdn;
% Td = 0;

% Hand force
fh1 = 5;
Fh = 5;

% Properties of Haptic device
mh = 10;     % mass
bh = 25;   % damping
xhd0 = -0.05;
vhd0 = 0;

% Properties of virtual tool
mc = 20;     % mass
bc = bh;   % damping
xc0 =xhd0;
vc0 = vhd0;

% PD controller between haptic device and cursor
Kvc = 2000;
Bvc = 3;

%Stiffness and damping of virtual coupling for object 
Kt = 2000;
Bt = 300;
% Stiffness and damping of virtual wall
% unstable for this values
% Krdp = 20000;
% Brdp = 20000;
% unstable for this values
% Krdp = 2000;
% Brdp = 5000;
% unstable
% Krdp = 2000;
% Brdp = 3500;
% unstable 
% Krdp = 1500;
% Brdp = 3000;
% unstable
% Krdp = 1500;
% Brdp = 2800;
% unstable
% Krdp = 1000;
% Brdp = 2000;
% my system is stable for the following values or values less than this,  when the time delay is zero line number 11 in the code,
% you can check for the different values 
Krdp = 100;
Brdp = 2000;
%  also when td i.e time delay is equal to 0.5 it gets unstable , you can
%  check it by un commenting it on line number 8
clear all
close all
clc
load Pe_sub1_Session1_Trial01.mat;
load Ps_sub1_Session1_Trial01.mat;
load Pw_sub1_Session1_Trial01.mat;
swivel = zeros(167,1);
for c = 1:167
    Pel = Pe(c,:);
    Psh = Ps(c,:);
    Pwr = Pw(c,:);    
    n = (Pwr -Psh)/(sqrt(sum(Pwr -Psh).^2));
    f = (Pel - Psh)/(sqrt(sum(Pel - Psh).^2));
    f1 = f - dot(f,n)*n;
    a1 = [0 0 -1];
    a = transpose(a1);
    u = (a - dot(a,n)*n)/(sqrt(sum(a - dot(a,n)*n).^2));
    x = cross(f1,u);
    y = dot(n,x);
    fu = dot(f1,u);
    phi = atan2d(y,fu);
    swivel(c) = phi;
end
disp('Time Sequence of Swivel Angle')
disp(swivel)
plot(swivel)
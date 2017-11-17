% standard dh paramter matrix
syms th d a alph
T = [ cos(th)  -sin(th)*cos(alph)   sin(th)*sin(alph)  acos(th)
      sin(th)   cos(th)*cos(alph)  -cos(th)*sin(alph)  asin(th)
      0         sin(alph)          cos(alph)         d
      0         0                  0                 1];
  
%link 1(dh parameters of link 1)
syms th1 
T1 = subs(T,{th,d,a,alph},{th1,0,0,0})

%link 2(dh parameters of link 2)
syms th2 
T2 = subs(T,{th,d,a,alph},{th2,0,0,-pi/2})

%link 3(dh parameters of link 3)
syms th3 d3 a2
T3 = subs(T,{th,d,a,alph},{th3,d3,a2,0})

%link 4(dh parameters of link 4)
syms th4 d4 a3
T4 = subs(T,{th,d,a,alph},{th4,d4,a3,0})

%link 5(dh parameters of link 5)
syms th5 
T5 = subs(T,{th,d,a,alph},{th5,0,0,pi/2})

%link 6(dh parameters of link 6)
syms th6 
T6 = subs(T,{th,d,a,alph},{th6,0,0,-pi/2})

%total forward matrix T06
T06 =simplify(T1*T2*T3*T4*T5*T6)%matrix of forward kinematics
%end effector positions
X = T06(1,4)
Y = T06(2,4)
Z = T06(3,4)

Thome = subs(A06,{th1,th2,th3,th4,th5,th6},{0,0,0,0,0,0})%matrix for home position where theta values are zero
X = Thome(1,4)%end effector position 
Y = Thome(2,4)%end effector position
Z = Thome(3,4)%end effector position




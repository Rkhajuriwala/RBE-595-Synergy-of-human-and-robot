syms r11 r12 r13 r21 r22 r23 r31 r32 r33 px py pz th23 th1 th2 th3 th4 th5 th6 K a4 a2 a3 d4 d3
T06 = [ r11 r12 r13 px 
        r21 r22 r23 py
        r31 r32 r33 pz
        0   0   0   1];

    
T01 = [ cos(th1) -sin(th1) 0 0
       sin(th1) cos(th1) 0 0
       0         0        1 0
       0         0        0 1];
   
T12 = [ cos(th2) -sin(th2) 0 0
        0         0        1 0
       -sin(th2) -cos(th2) 0 0
        0         0        0 1];
    
T23 = [ cos(th3) -sin(th3) 0 a2
        sin(th3)  cos(th3) 0 0
        0         0        1 d3
        0         0        0 1];
  
T34 = [ cos(th4) -sin(th4) 0 a3
        0         0        1 d4
       -sin(th4) -cos(th4) 0 0
       0          0        0 1];
  
T45 = [ cos(th5) -sin(th5) 0 0
        0         0       -1 0
        sin(th5)  cos(th5) 0 0
        0         0        0 1];

 T56 = [ cos(th6) -sin(th6) 0 0
         0         0        1 0
        -sin(th6) -cos(th6) 0 0
         0         0        0 1];
  T06 = T01*T12*T23*T34*T45*T56;
     
%inv(T01)*T06 = inv(T01)*T01*T12*T23*T34*T45*T56;
%BY COMPUTING EQUATIONS ON PAPER WE GET THE FOLLOWING EQUATIONS
%-sin(th1)*px + cos(th1)*py = d3;
%from equations from paper and solving we get value of joint angle theta 1
th1 = atan2(py,px)- atan2(d3,sqrt(px^2+py^2-d3^2))
%for joint angle theta 3 , we compute equations as discussed earlier on
%paper
%K = a3*cos(th3) - d4*sin(th3)
K = (px^2+py^2+pz^2-(a2)^2-(a3)^2-(d3)^2-(a4)^2)/(2*(a2));
th3 = atan2(a3,d4) - atan2(K,sqrt((a3)^2+(d4)^2-K^2))
%for joint angle theta 2 and theta 4 we use the following equations
% we use this --> inv(T03)*T06 = T34*T45*T56
%we compute them and get the following equations
% cos(th1)*cos(th2+th3)*px + sin(th1)*cos(th2+th3)*py - sin(th2+th3)*pz -
% a2*cos(th3)= a3
%and 
%-c1*s23*px - s1*s23*py -c23*pz + a2*s3= d4
% USING THIS EQUATIONS WE GET VALUES FOR ANGLE 23 
sin(th23)= ((-a3-a2*cos(th3))*pz + (cos(th1)*px + sin(th1)*py)*(a2*sin(th3)- d4)/(pz^2+(cos(th1)*px + sin(th1)*py)^2));
cos(th23) = ((a2*sin(th3)- d4)*pz -(a3+a2*cos(th3))*(cos(th1)*px + sin(th1)*py)/(pz^2+(cos(th1)*px + sin(th1)*py)^2));
th23= atan2(sin(th2+th3),cos(th2+th3));
th2 = (th23)-th3
%for joint angle theta 4 we can compute the following equations 
% r13*c1*c23 +r23*s1*c23 - r33*s23 = -c4*s5
%-r13*s1+r23*c1=s4*s5
%we can cancel s5 from both and find theta 4,as long as s5 is not equal to
%0
cos(th4) = (-1)*(r12*cos(th1)*cos(th2+th3)+r23*sin(th1)*cos(th2+th3)-r33*sin(th2+th3)/sin(th5));
sin(th4)= (-r13*sin(th1)+ r23*cos(th1))/sin(th5);
th4 = atan2(sin(th4),cos(th4))

%for joint angle theta 5
%we use this --> inv(T04)*T06 = T45*T56
% by computing the above equation and comparing the best equation we get
% are the following which gives use values of theta 5 in  sin() and
% cosine()
sin(th5) = (-1)*(r13*(cos(th1)*cos(th2+th3)*cos(th4)+sin(th1)*sin(th4))+r23*(sin(th1)*cos(th2+th3+th4)-cos(th1)*sin(th4))-r33*(sin(th2+th3)*cos(th4)));
cos(th5) = (-1)*(r13*(cos(th1)*sin(th2+th3))+r23*(sin(th1)*sin(th2+th3))+r22*(cos(th2+th3)));
%using this equations we can get value of theta 5
th5 = atan2(sin(th5),cos(th5))
%For Joint angle theta 6 we can use equations similar by computing on paper
%and use matlab to find the final angle
% inv(T05)*T06 = T56
%by computing on paper we can get the value of theta 6 in sin and cosine
sin(th6) = (-1)*r11*((cos(th1)*cos(th2+th3)*sin(th4))-(sin(th1)*cos(th4)))- r21*((sin(th1)*cos(th2+th3)*sin(th4))+(cos(th1)*cos(th4)))+ r31(sin(th2+th3)*sin(th4));
cos(th6) = r11*((cos(th1)*cos(th2+th3)*cos(th4)+sin(th1)*sin(th4))*cos(th5)-cos(th1)*sin(th2+th3)*sin(th5))+ r21*((sin(th1)*cos(th2+th3)*cos(th4))-cos(th1)*sin(th4)*cos(th5)-sin(th1)*sin(th2+th3)*sin(th5))- r31*((sin(th2+th3)*cos(th4)*cos(th5))+(cos(th2+th3)*sin(th5)));
%by using this values we can find values of theta 6
th6 = atan2(sin(th6),cos(th6))

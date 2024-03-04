function O = direct_kinematics(A1, A2, A3, A4, A5, A6)
    T1=[1 0 0 0
        0 1 0 0
        0 0 1 0.103
        0 0 0 1];
    R01=[cos(A1) -sin(A1) 0 0
        sin(A1) cos(A1) 0 0
        0 0 1 0
        0 0 0 1];
    T2=[1 0 0 0
        0 1 0 0
        0 0 1 0.080
        0 0 0 1];
    R12=[1 0 0 0
        0 cos(A2) -sin(A2) 0
        0 sin(A2) cos(A2) 0
        0 0 0 1];
    T3=[1 0 0 0
        0 1 0 0
        0 0 1 0.210
        0 0 0 1];
    R23=[1 0 0 0
        0 cos(A3) -sin(A3) 0
        0 sin(A3) cos(A3) 0
        0 0 0 1];
    T4=[1 0 0 0
        0 1 0 -0.0415
        0 0 1 0.030
        0 0 0 1];
    R34=[cos(A4) 0 sin(A4) 0
        0 1 0 0
        -sin(A4) 0 cos(A4) 0
        0 0 0 1];
    T5=[1 0 0 0
        0 1 0 -0.180
        0 0 1 0
        0 0 0 1];
    R45=[1 0 0 0
        0 cos(A5) -sin(A5) 0
        0 sin(A5) cos(A5) 0
        0 0 0 1];
    T6=[1 0 0 0
        0 1 0 -0.0237
        0 0 1 -0.0055
        0 0 0 1];
    R56=[cos(A6) 0 sin(A6) 0
        0 1 0 0
        -sin(A6) 0 cos(A6) 0
        0 0 0 1];

    T01=T1*R01;
    T12=T2*R12;
    T23=T3*R23;
    T34=T4*R34;
    T45=T5*R45;
    T56=T6*R56;

    O=zeros(1,6);
 
    Transform=T01*T12*T23*T34*T45*T56;
    display(Transform);
    
    O(1:4)=Transform*[0;0;0;1];
    r=Transform(1:3,1:3);
    
    O(5)=atan2(sqrt(r(3,1)^2+r(3,2)^2),r(3,3));
    
    if sin(O(5))> 0
        O(6) = atan2(r(3,2), -r(3,1));
        O(4) = atan2(r(2,3), r(1,3));
    else
        O(6) = atan2(-r(3,2), r(3,1));
        O(4) = atan2(-r(2,3), -r(1,3));
    end
    
    %computing the joint's coordinates
    j=zeros(6,4); %junctions matrix
    j(1,:)=T01*[0 0 0 1]';
    j(2,:)=T01 * T12*[0 0 0 1]';
    j(3,:)=T01 * T12 * T23*[0 0 0 1]';
    j(4,:)=T01 * T12 * T23 * T34*[0 0 0 1]';
    j(5,:)=T01 * T12 * T23 * T34 * T45*[0 0 0 1]';
    j(6,:)=Transform*[0 0 0 1]';
    
    %auxilary points to draw the robot
    aux1=j(3,:)'+R01*R12*R23*[0; 0; 0.03; 0];
    aux2=j(5,:)'+R01*R12*R23*R34*R45*[0; 0; -0.0055; 0];
    
    %points for drawing the arms of the robot
    x_arm=[0;j(1:3,1);aux1(1);j(4:5,1);aux2(1);j(6,1)];
    y_arm=[0;j(1:3,2);aux1(2);j(4:5,2);aux2(2);j(6,2)];
    z_arm=[0;j(1:3,3);aux1(3);j(4:5,3);aux2(3);j(6,3)];
    
    figure(2)
    %drawing the joints of the robot
    joints=plot3(j(:,1),j(:,2),j(:,3),'o');hold on
    %drawing the robot's arms
    line(x_arm,y_arm,z_arm);
    %drawing the base of the robot
    line([0.02 -0.02 0 0 0.02 0 -0.02 0],[0 0 0.02 -0.02 0 0.02 0 -0.02], [0 0 0 0 0 0 0 0]); 
    view(3);
    legend([joints],'Joints');
    xlim([-0.5, 0.5]);
    ylim([-0.5, 0.5]);
    zlim([0, 0.8]);
    xlabel('x');
    ylabel('y');
    zlabel('z');

    
    
end


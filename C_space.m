% creating three polygon 
pgon1 = nsidedpoly(6,'Center',[3 1],'SideLength',1);
pgon2 = nsidedpoly(4,'Center',[-3 -3],'SideLength',2);
pgon3 = polyshape([-2.5 -3 0], [1 3 2.5]);
% creating subplot 1x3 
subplot(1,3,1);
% this subplot will plot obstacle in cartesian space
plot(pgon1,'Facecolor','red')
hold on
plot(pgon2,'Facecolor','green')
hold on
plot(pgon3,'Facecolor','blue')
hold on
title('Obstacle Showcase');
axis([-6 6 -6 6]);
xlabel('X') 
ylabel('Y')
daspect([1 1 1]);
grid on;
% length of links
L1 = 3;
L2 = 2;
% ct is used for capturing frames
ct = 1;
% loops for generating c-space
for theta1 = 0:9:360
    for theta2 = 0:9:360
        %x1 is an array consisting of 1000 points between origin and end point of link1
        x1 = linspace(0, L1*cos(theta1*pi/180), 1000);
        %x2 is an array consisting of 1000 points between link1 and link2
        x2 = linspace(L1*cos(theta1*pi/180),L1*cos(theta1*pi/180)+ L2*cos((theta1*pi/180)+(theta2*pi/180)),1000);
        y1 = linspace(0, L1*sin(theta1*pi/180), 1000);
        y2 = linspace(L1*sin(theta1*pi/180),L1*sin(theta1*pi/180)+ L2*sin((theta1*pi/180)+(theta2*pi/180)),1000);
        subplot(1,3,2);
        title('Robo-WorkSpace');
        xlabel('X ') 
        ylabel('Y')
        axis([-8 8 -8 8]);
        daspect([1 1 1]);
        grid on;
        plot(x2, y2)
        M(ct)=getframe(gcf);
        ct = ct+1;
        hold on
        X = [x1 x2];
        Y = [y1 y2];
        % isinterior returns a logical array whose elements are 1 when
        % the corresponding planar points in querypoint are in the polygon shape.
        T1 = isinterior(pgon1,X,Y);
        T2 = isinterior(pgon2,X,Y);
        T3 = isinterior(pgon3,X,Y);
        if T1'*T1>0
            subplot(1,3,3);
            title('Configuration Space');
            xlabel('Theta1') 
            ylabel('Theta2')
            axis([0 360 0 360]);
            daspect([1 1 1]);
            grid on;
            scatter(theta1,theta2,'r*');
            hold on
        end
        if T2'*T2>0
           subplot(1,3,3);
           title('Configuration Space');
           xlabel('Theta1') 
           ylabel('Theta2')
           axis([0 360 0 360]);
           daspect([1 1 1]);
           grid on;
           scatter(theta1,theta2,'b*');
           hold on    
        end
        if T3'*T3>0
           subplot(1,3,3);
           title('Configuration Space');
           xlabel('Theta1') 
           ylabel('Theta2')
           axis([0 360 0 360]);
           daspect([1 1 1]);
           grid on;
           scatter(theta1,theta2,'g*');
           hold on
        end
    end
end

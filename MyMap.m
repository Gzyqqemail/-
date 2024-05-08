function out = MyMap(UavTeam,~,Highway)

M = UavTeam.AvailableNumMax;

line11 = [Highway(1).ph1(1),Highway(1).ph2(1)-4500];
line12 = [Highway(1).ph1(2)+Highway(1).rh,Highway(1).ph2(2)+Highway(1).rh];
plot(line11,line12);
hold on
line21 = [Highway(1).ph1(1)  Highway(1).ph2(1)-4500]';
line22 = [Highway(1).ph1(2)-Highway(1).rh  Highway(1).ph2(2)-Highway(1).rh]';
plot(line21,line22);
hold on

for k = 1: M
    o1 = [UavTeam.Uav(k).CurrentPos(1) UavTeam.Uav(k).CurrentPos(2)]';
    o2 = [UavTeam.Uav(k).CurrentPos(1)+1/UavTeam.gain*UavTeam.Uav(k).Velocity(1) UavTeam.Uav(k).CurrentPos(2)+1/UavTeam.gain*UavTeam.Uav(k).Velocity(2)]';
    mydrawcolorball(o1,o2,k);  
end
axis([-2000 2000 -2000 2000]) 
grid on
xlabel('x')
ylabel('y')

out = 0;
end

function mydrawcolorball(o1,o2,k)
        radiusplot(o1,o2,k)
end

function radiusplot(o1,o2,~)
global  rs ra rd
%         mycircle(o1,rm)
mycircle_y(o1,rd)
mycircle_g(o2,ra)
mycircle_b(o2,rs)
plot(o1(1),o1(2),'k-o','MarkerFaceColor','k','MarkerSize',4)
end

function mycircle_b(o,r)
 alpha = 0:pi/20:2*pi;    
 x = o(1) +  r*cos(alpha); 
 y = o(2) +  r*sin(alpha); 
 plot(x,y,'b-') 
end

function mycircle_y(o,r)
 alpha = 0:pi/20:2*pi;    
 x = o(1) +  r*cos(alpha); 
 y = o(2) +  r*sin(alpha); 
 plot(x,y,'y-') 
end

function mycircle_g(o,r)
 alpha = 0:pi/20:2*pi;    
 x = o(1) +  r*cos(alpha); 
 y = o(2) +  r*sin(alpha); 
 plot(x,y,'g-') 
end
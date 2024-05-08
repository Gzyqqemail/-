function out = MyMap(UavTeam,~,Highway)

M = UavTeam.AvailableNumMax;
%line11是第一条线段的左端点
%line12是第一条线段的右端点
line11 = [Highway(1).ph1(1),Highway(1).ph2(1)-4500];
line12 = [Highway(1).ph1(2)+Highway(1).rh,Highway(1).ph2(2)+Highway(1).rh];
plot(line11,line12);
hold on
line21 = [Highway(1).ph1(1)  Highway(1).ph2(1)-4500]';
line22 = [Highway(1).ph1(2)-Highway(1).rh  Highway(1).ph2(2)-Highway(1).rh]';
plot(line21,line22);
hold on
%每个无人机都有两个圆心，O1是真正的圆心，O2是考虑速度之后的圆心
for k = 1: M
    o1 = [UavTeam.Uav(k).CurrentPos(1) UavTeam.Uav(k).CurrentPos(2)]';
    o2 = [UavTeam.Uav(k).CurrentPos(1)+1/UavTeam.gain*UavTeam.Uav(k).Velocity(1) UavTeam.Uav(k).CurrentPos(2)+1/UavTeam.gain*UavTeam.Uav(k).Velocity(2)]';
    mydrawcolorball(o1,o2,k);  
end
axis([-800 800 -800 800]) 
grid on
xlabel('x')
ylabel('y')

out = 0;
end

function mydrawcolorball(o1,o2,k)
        radiusplot(o1,o2,k)
end

function radiusplot(o1,o2,~)
global  rs ra rd rm

mycircle_y(o1,rd)%探测半径
mycircle_g(o2,ra)
mycircle_b(o2,rs)

alpha = 0:pi/180:2*pi;    
x = o1(1) +  rm*cos(alpha); 
y = o1(2) +  rm*sin(alpha); 
fill(x, y, 'k');  % 绘制一个实心黑色圆球
 
end


function mycircle_b(o,r)
 alpha = 0:pi/180:2*pi;    
 x = o(1) +  r*cos(alpha); 
 y = o(2) +  r*sin(alpha); 
 plot(x,y,'b-') 
end

function mycircle_y(o,r)
 alpha = 0:pi/180:2*pi;    
 x = o(1) +  r*cos(alpha); 
 y = o(2) +  r*sin(alpha); 
 plot(x,y,'y-') 
end

function mycircle_g(o,r)
 alpha = 0:pi/180:2*pi;    
 x = o(1) +  r*cos(alpha); 
 y = o(2) +  r*sin(alpha); 
 plot(x,y,'g-') 
end
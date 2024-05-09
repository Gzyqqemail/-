%此函数获取 所有无人机当前的位置，当前的速度，输出无人机接下来的速度，并在适当时刻更新地图、输出仿真时间。
function u = collisioncon(in)
global Highway Obstacle
global UavTeam
global gcount gfigure

gcount = gcount + 1;

M = UavTeam.AvailableNumMax;
V = [];
rh = Highway(1).rh ;%管道宽度的一半
rb=Highway(1).rb;%管道缓冲区宽度

for k = 1:M
    UavTeam.Uav(k).CurrentPos =  in(2*(k-1)+1:2*k); %in这个变量前2M个元素是UAV当前坐标
    UavTeam.Uav(k).Velocity   =  in(2*M+2*(k-1)+1: 2*M+2*k);%后2M个元素是UAV当前速度
end

for k = 1:M
    % Velocity control command
    %将UAV的一些变量重命名
      Pcur                      =  UavTeam.Uav(k).CurrentPos;
      Pdes                      =  UavTeam.Uav(k).Waypoint(:,UavTeam.Uav(k).CurrentTaskNum);
      Vcur                      =  UavTeam.Uav(k).Velocity;
      ksicur= Pcur+1/UavTeam.gain*Vcur; %计算滤波后的当前位置


      %根据当前位置调整无人机所属区域，并修改无人机当前速度
      if norm(ksicur)~=0 %这用于检查向量是否为零向量
      if ksicur(1)<-rb && UavTeam.Uav(k).State == 1 
          UavTeam.Uav(k).State = 3;
      end
          if ksicur(1)<-rb && UavTeam.Uav(k).State == 2 
          UavTeam.Uav(k).State = 4;
      end
       if ksicur(2)<rh-rb && UavTeam.Uav(k).State == 3
              UavTeam.Uav(k).State =5;
      end
              if  ksicur(2)>-rh+rb && UavTeam.Uav(k).State == 4
                                UavTeam.Uav(k).State =5;
              end

   UavTeam.Uav(k).VelocityCom   =  mycontrol(k);
    % Collect all the control 。根据速度指令修改无人机当前的速度
     V  = [V;UavTeam.Uav(k).VelocityCom];
      else
          V  = [V; 0;0];
      end


% plot online,更新地图  
  if gcount>=2500
     figure(1);
     gfigure=gfigure+1;
%      subplot(3,2,gfigure)
     MyMap(UavTeam,Obstacle,Highway);
     for k = 1:M
        o = [UavTeam.Uav(k).CurrentPos(1) UavTeam.Uav(k).CurrentPos(2)]';
        mydrawcolorball(o,k); 
     end
     gcount = 0;
     in(end)
  end

     if gcount>=1000
        figure(1);
    
         hold off
         MyMap(UavTeam,Obstacle,Highway);%更新地图

     end
end

if gcount>=1000
gcount = 0;
gfigure = gfigure+1;
t = in(end)%in的最后一个元素是仿真时间，将仿真时间输出。
end

u = V;


end




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%此函数用于返回无人机当前的速度
function u = mycontrol(i)
global Highway
global UavTeam

%相当于将无人机的变量重命名了
Pcur  =  UavTeam.Uav(i).CurrentPos;
Vcur  =  UavTeam.Uav(i).Velocity;
Pdes  =  UavTeam.Uav(i).Waypoint(:,UavTeam.Uav(i).CurrentTaskNum);
   rm = UavTeam.Uav(i).r;
   ra = UavTeam.Uav(i).ra;     
 vmax = UavTeam.Uav(i).vmax;

%相当于把管道的变量重命名了
 % Area Definition
pt1  = Highway(1).ph1; 
pt2  = Highway(1).ph2;
 rt = Highway(1).rh;
pt3 = Highway(1).ph2 + [0 rt]';
 ksi  = Pcur + 1/UavTeam.gain*Vcur;
 rb = 0;
 rsr= 10000;
 rrt= 10000;
 
 if UavTeam.Uav(i).State == 1  % Left Standby Area
    psr1 = [pt2(1) rt+rsr]';
    psr2 = [-rsr rt+rsr]';
    psr3 = [-rsr rt]';
       u = mytunnel(i,psr1,psr2,psr3,rsr);
    
 elseif  UavTeam.Uav(i).State == 2 % Right Standby Area
   psr1p = [pt2(1) -rt-rsr]';
   psr2p = [-rsr -rt-rsr]';
   psr3p = [-rsr -rt]';
       u = mytunnel(i,psr1p,psr2p,psr3p,rsr); 
       
 elseif  UavTeam.Uav(i).State == 3 % Left Ready Area
   prt1 = [-rrt rt+rrt]';
   prt2 = [-rrt 0]';
   prt3 = [-2*rrt    0]';
       u = mytunnel(i,prt1,prt2,prt3,rrt); 
       
 elseif  UavTeam.Uav(i).State == 4 % Right Ready Area 
   prt1p = [-rrt -rt-rrt]';
   prt2p = [-rrt 0]';
   prt3p = [0    0]';
       u = mytunnel(i,prt1p,prt2p,prt3p,rrt); 
       
 else% Tunnel and Tunnel Extension
       u = mytunnel(i,pt1,pt2,pt3,rt); 
 end
end
    




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function u = mytunnel(i,p1,p2,p3,rh)%p1,p2是管道中心线的左右端点，p3是上管道线的右端点
global UavTeam

Pcur  =  UavTeam.Uav(i).CurrentPos;
Vcur  =  UavTeam.Uav(i).Velocity;
Pdes  =  UavTeam.Uav(i).Waypoint(:,UavTeam.Uav(i).CurrentTaskNum);
   rs = UavTeam.Uav(i).rs;
   ra = UavTeam.Uav(i).ra;     
 vmax = UavTeam.Uav(i).vmax;

%两个矩阵，方便计算无人机到管道线的距离
 At12 = eye(2) - (p1-p2)*(p1-p2)'/(norm(p1-p2)*norm(p1-p2));
 At23 = eye(2) - (p2-p3)*(p2-p3)'/(norm(p2-p3)*norm(p2-p3)); 
 
% Velocity to line
    k1 = 1;
    el = Pcur + 1/UavTeam.gain*Vcur - p2;
    Vl = -mysat(k1*At23*el,vmax);
 
% Velocity away other multicoptee
    Vm = [0;0];
    for l = 1:UavTeam.AvailableNumMax
        if i~=l && norm(Pcur)~=0
     k2 = 1; e = 0.000001; 
 ksimil = (Pcur - UavTeam.Uav(l).CurrentPos) + 1/UavTeam.gain*(Vcur - UavTeam.Uav(l).Velocity);
   temp = (1+e)*norm(ksimil) - (2*rs)*mys(norm(ksimil)/(2*rs),e);
    bil = k2*dmysigma(norm(ksimil),2*rs,rs+ra)/temp ...
        - k2*mysigma(norm(ksimil),2*rs,rs+ra)*((1+e)-dmys(norm(ksimil)/(2*rs),e))/(temp^2);
     Vm = Vm - bil*(ksimil/norm(ksimil));
        end
    end
 
 % Velocity within the tunnel
    Vt = [0;0];
      k3 = 1; e = 0.001;
   ksihi = At12*((Pcur-p2) + 1/UavTeam.gain*Vcur);
  temp1  = (rh-rs)/(norm(ksihi)+e);
  temp2  = (rh-rs) - norm(ksihi)*mys(temp1,e);
  %% wrong 负号
     ci = -k3*dmysigma(rh-norm(ksihi),rs,rs+ra)/temp2 ...
        - k3*mysigma(rh-norm(ksihi),rs,rs+ra)*(-mys(temp1,e)+norm(ksihi)*dmys(temp1,e)*(rh-rs)/(norm(ksihi)+e)/(norm(ksihi)+e))/(temp2^2);
     Vt = Vt - ci*(ksihi/norm(ksihi));
     
   % Sum of all velocities
    u = mysat(Vl+Vm+Vt,vmax);%根据3个约束条件得到无人机当前的速度，并返回。
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    function u=mys(x,rs)

x2 =  1 + 1/tan(67.5/180*pi)*rs;
x1 = x2 - sin(45/180*pi)*rs;
if x<=x1
    u = x;
elseif x1<=x  & x<=x2
    u = (1-rs)+sqrt(rs^2-(x-x2)^2);
else
    u = 1;
end
    end

function u=dmys(x,rs)

x2 =  1 + 1/tan(67.5/180*pi)*rs;
x1 = x2 - sin(45/180*pi)*rs;
if x<=x1
    u = 1;
elseif x1<=x  & x<=x2
    u = (x2-x)/sqrt(rs^2-(x-x2)^2)/2;
else
    u = 0;
end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function u=mysat(x,a)
% ensure the direction to be the same
    if norm(x)>a
       u =a*x/norm(x);
    else
       u =x ;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function u=mysigma(x,d1,d2)

if x<=d1
    u = 1;
elseif d1<=x  & x<=d2
    A = -2/((d1-d2)^3); B = 3*(d1+d2)/((d1-d2)^3); C = -6*d1*d2/((d1-d2)^3); D = d2^2*(3*d1-d2)/((d1-d2)^3);
    u = A*x^3 + B*x^2 + C*x + D;
else
    u = 0;
end
end

function u=dmysigma(x,d1,d2)

if x<=d1
    u = 0;
elseif d1<=x  & x<=d2
    A = -2/((d1-d2)^3); B = 3*(d1+d2)/((d1-d2)^3); C = -6*d1*d2/((d1-d2)^3);
    u = 3*A*x^2 + 2*B*x + C;
else
    u = 0;
end
end

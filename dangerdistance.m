function u = dangerdistance(in)

global Highway Obstacle
global UavTeam

M     =  UavTeam.AvailableNumMax;
Pcur  = in(1:2*M);
Vcur  = in(2*M+1:4*M);

mindism = 100000;

mindiso = 100000;
  sz   =  size(Obstacle);

 rh = Highway(1).rh;
mindish = rh;

u = [mindiso;mindish;mindism];


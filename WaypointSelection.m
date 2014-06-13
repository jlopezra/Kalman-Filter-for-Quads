function iw=WaypointSelection(Xtrue,Xw,iw,P)
x=Xtrue(1);
y=Xtrue(2);
z=Xtrue(3);
R=((x-Xw(1))^2+(y-Xw(2))^2+(z-Xw(3))^2)^0.5; % Calculates the distance that the vehicle needs to travel to get to the next waypoint JCL
if(R<P.Rd)
    if(iw<P.Wn)
  iw=iw+1;
    else
        iw=1;
    end
end



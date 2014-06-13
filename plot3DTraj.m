function h=plot3DTraj(h,Xfilter,Xtrue,P)
set(h.Xtrue,'xdata',Xtrue(1,:),'ydata',Xtrue(2,:),'zdata',Xtrue(3,:));
set(h.Xfilter,'xdata',Xfilter(1,:),'ydata',Xfilter(2,:),'zdata',Xfilter(3,:));
drawnow;

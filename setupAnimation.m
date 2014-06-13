function [h, h2]=setupAnimation(P)
figure(1)
axis(100*[-1.3 1.3 -1.3 1.3 0 1.3])
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');
view(60,60)
for i=1:P.N
    plot3(100*P.Xl(i),100*P.Yl(i),100*P.Zl(i),'*','markersize',15,'LineWidth',2)
    hold on
    text(100*P.Xl(i)+9,100*P.Yl(i)+9,100*P.Zl(i)+9,['LM' int2str(i)])
end
view(60,60)
xlabel('X(m)');
ylabel('Y(m)');
zlabel('Z(m)');
title(['R_{Sensor}=' num2str(P.Rsensor) 'm' ', ' 'T_s=' num2str(P.Ts) 's'])
%grid on
%legend('LM_1','UAV_{true}')
axis(100*[-1.3 1.3 -1.3 1.3 0 1.3])
view(60,60)
h.Xtrue=plot3(0,0,0,'s','markersize',12,'linewidth',2,'erasemode','normal');
hold on
h.Xfilter=plot3(0,0,0,'ro','markersize',4,'linewidth',2,'erasemode','normal');
hold on
view(60,60)
legend('LM_1','UAV_{true}','UAV_{filter}')
% for i=1:P.Nq
%  h.text(i)= text(0,0,0,['UAV' int2str(i)]);
%  hold on
% end
%axes
for i=1:P.Nq
    h2(i)=line(0,0,'Color','k','LineWidth',1,'LineStyle','-.','EraseMode','background');
end
view(60,60)
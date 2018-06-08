% Program 6.6 Animation program for bridge using IVP solver
% Inputs: inter = time interval inter, 
%   ic = [y(1,1) y(1,2) y(1,3) y(1,4)],
% y = [y, y', theta, theta']
% y' = f(y) = [y', y'', theta', theta''] 
%   number of steps n, steps per point plotted p
% Calls a one-step method such as trapstep.m
% Example usage: tacoma([0 1000],[0 0 0.001 0],25000,5);
function tacoma(inter,ic,n,p)

figure;
subplot(3,1,1);
h=(inter(2)-inter(1))/n;
% y1(1,:)=ic;                           % enter initial conds in y
y2(1,:)=ic;  
t(1)=inter(1);len=6;
set(gca,'XLim',[-8 8],'YLim',[-8 8], ...
   'XTick',[-8 0 8],'YTick',[-8 0 8]);
cla;                                 % clear screen
axis square                          % make aspect ratio 1 - 1
%road1=animatedline('color','b','LineStyle','-','LineWidth',1);
%lcable1=animatedline('color','r','LineStyle','-','LineWidth',1);
%rcable1=animatedline('color','r','LineStyle','-','LineWidth',1);

road2=animatedline('color','b','LineStyle','--','LineWidth',1);
lcable2=animatedline('color','r','LineStyle','--','LineWidth',1);
rcable2=animatedline('color','r','LineStyle','--','LineWidth',1);

subplot(3,1,2);
% yy1 = animatedline('color','b','LineStyle','-','LineWidth',1);
yy2 = animatedline('color','r','LineStyle','-','LineWidth',1);
xlim([inter(1), inter(2)]);
title('y')

subplot(3,1,3);
% tt1 = animatedline('color','b','LineStyle','-','LineWidth',1);
tt2 = animatedline('color','r','LineStyle','-','LineWidth',1);
xlim([inter(1), inter(2)]);
title('\theta')


for k=1:n/p
  for i=1:p
    t(i+1) = t(i)+h;
  %  y1(i+1,:) = trapstep(t(i),y1(i,:),h);
    y2(i+1,:) = rk4(t(i),y2(i,:),h);
  end
  t(1)=t(p+1);
  % y1(1,:) = y1(p+1,:);
  y2(1,:) = y2(p+1,:);
  % z1(k)=y(1,1);z3(k)=y(1,3);
  % c1=len*cos(y1(1,3));s1=len*sin(y1(1,3));
  subplot(3,1,1);
  % clearpoints(road1);addpoints(road1,[-c1 c1],[-s1-y1(1,1) s1-y1(1,1)])
  % clearpoints(lcable1);addpoints(lcable1,[-c1 -c1],[-s1-y1(1,1) 8])
  % clearpoints(rcable1);addpoints(rcable1,[c1 c1],[s1-y1(1,1) 8])
  
  c2=len*cos(y2(1,3));s2=len*sin(y2(1,3));
  clearpoints(road2);addpoints(road2,[-c2 c2],[-s2-y2(1,1) s2-y2(1,1)])
  clearpoints(lcable2);addpoints(lcable2,[-c2 -c2],[-s2-y2(1,1) 8])
  clearpoints(rcable2);addpoints(rcable2,[c2 c2],[s2-y2(1,1) 8])
  
  subplot(3,1,2);
  % addpoints(yy1,t,y1(:,1));
  addpoints(yy2,t,y2(:,1));
  
  subplot(3,1,3);
  % addpoints(tt1,t,y1(:,3));
  addpoints(tt2,t,y2(:,3));
  drawnow;
  pause(h);
end
end

function y = trapstep(t,x,h)
%one step of the Trapezoid Method
z1=ydot(t,x);
g=x+h*z1;
z2=ydot(t+h,g);
y=x+h*(z1+z2)/2;
end

function y = rk4(t,x,h)

K1 = h*ydot(t, x);
K2 = h*ydot(t + h/2, x + K1/2); 
K3 = h*ydot(t + h/2, x + K2/2);
K4 = h*ydot(t + h, x + K3);
y = x + (K1 + K2 + K3 + K4)/6;
end

function ydot=ydot(t,y)
len=6; a=0.2; W=610; omega=2*pi*38/60;
a1=exp(a*(y(1)-len*sin(y(3))));
a2=exp(a*(y(1)+len*sin(y(3))));
ydot(1) = y(2);
ydot(2) = -0.01*y(2)-0.4*(a1+a2-2)/a+0.2*W*sin(omega*t);
ydot(3) = y(4);
ydot(4) = -0.01*y(4)+1.2*cos(y(3))*(a1-a2)/(len*a);
end
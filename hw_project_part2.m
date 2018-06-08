% 
burgers1(0,1,0,1,10,10);


%%
burgers2(0,1,0,1,100,16);

%%
p = [4, 5, 6, 7, 8];
err = zeros(1,length(p));
alf=4;bet=3;D=0.2; 
u = @(x,t) 2*D*bet*pi*exp(-D*pi^2*t)*sin(pi*x)./(alf + bet*exp(-D*pi^2*t)*cos(pi*x));
for i = 1:length(p)
    k = 2^(-p(i));
    w = burgers2(0,1,0,1,100,1/k, false);
    approx = w(50,end); 
    actual = u(0.5,1);
    err(i) = abs(actual - approx);
end

figure;
loglog(p, err);
xlabel('p');
ylabel('Error');
grid on;

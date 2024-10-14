%Transmitted signal 
B= 10e7;   
Tp = 20e-6; 
mu = B / Tp;
f_c = 10e3;   
Fs = 2 * B; 
t = -10*Tp : 1/Fs : 10*Tp;
N=5;
c = 3e8;
R=100;

p = @(t) ( (-Tp/2 <= t) & (t <= Tp/2) ) .* exp(1i*pi*mu*(t.^2)); 
fm = @(t) ( (-Tp/2 <= t) & (t <= Tp/2) ) .* (f_c+ (mu*t));

function [S,f] =S_Tx(N,t, Tp,f_c,p, fm)
    S = 0; 
    f = 0; 
    for K=0:N
      S=S + p(t-K*Tp).*exp(1i*2*pi*f_c*t);
      f=f+fm(t-K*Tp);
    end
end
[S, f] = S_Tx(N, t, Tp, f_c, p, fm);
% figure;
% plot(t, real(S));
% figure;
% plot(t, f);
% Y = fft(S);
% N = length(Y);
% ff = (-N/2 : N/2-1) * (Fs / N);
% Y_shifted = fftshift(Y);
% figure;
% plot(ff , abs(Y_shifted));

n=length(t);

w=wgn(1,n,0.01,'linear');
%Received Signal
Td= 2*R/c;
R_f=0;
A= [0.1, 0.22, 0.5, 0.3, 0,2];
radial_velocity = 2;
fd= 2*radial_velocity*f_c/c;
fl=0;
for j=0:N
    % [S, f] = S_Tx(N, t-Td, Tp, f_c, p, fm);
    R_f=R_f+(A(j+1).* p(t-j*Tp-Td).*exp(1i*2*pi*f_c*(t-Td)).* exp(-1i * 2 * pi * fd )+w);
    fl=fl+f_c+fd+fm(t-j*Tp-Td);
end
% plot(t,w);
% figure;
% plot(t,real(R_f))

% 
% figure;
% plot(t,f,'b');
% hold on;
% plot(t,fl,'r');




    
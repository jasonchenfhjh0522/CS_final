% model
J = 3.2284E-6;
b = 3.5077E-6;
K = 0.0274;
R = 4;
L = 2.75E-6;
s = tf('s');
motor = K/(s*((J*s+b)*(L*s+R)+K^2));

% analysis
figure
step(motor);
figure
pzmap(motor);

figure
syscl = feedback(motor,1);
step(syscl);
figure
pzmap(syscl);
figure
margin(motor);

% design
Kp = 1;
for i = 1:3
    C(:,:,i) = pid(Kp);
    Kp = Kp + 10;
end
syscl = feedback(C*motor,1);
figure
step(syscl(:,:,1), syscl(:,:,2), syscl(:,:,3));
ylabel('Position, \theta (radians)');
title('Response to a Step Reference with Different Values of Kp');
legend('Kp = 1', 'Kp = 11','Kp = 21');


distcl = feedback(motor,C);
figure
step(distcl(:,:,1), distcl(:,:,2), distcl(:,:,3))
ylabel('Position, \theta (radians)');
title('Response to a Step Disturbance with Different Values of Kp')
legend('Kp = 1', 'Kp = 11','Kp = 21');

Kp = 21;
Ki = 100;
for i = 1:5
    C(:,:,i) = pid(Kp,Ki);
    Ki = Ki + 200;
end

syscl = feedback(C*motor,1);
figure
step(syscl(:,:,1), syscl(:,:,2), syscl(:,:,3));
ylabel('Position, \theta (radians)');
title('Response to a Step Reference with Kp = 21 and Different Values of Ki');
legend('Ki = 100', 'Ki = 300', 'Ki = 500');

distcl = feedback(motor,C);
figure
step(distcl(:,:,1), distcl(:,:,2), distcl(:,:,3));
ylabel('Position, \theta (radians)');
title('Response to a Step Disturbance with Kp = 21 and Different Values of Ki');
legend('Ki = 100',  'Ki = 300',  'Ki = 500');

Kp = 21;
Ki = 500;
Kd = 0.05;

for i = 1:3
    C(:,:,i) = pid(Kp,Ki,Kd);
    Kd = Kd + 0.1;
end

syscl = feedback(C*motor,1);
figure
step(syscl(:,:,1), syscl(:,:,2), syscl(:,:,3));
ylabel('Position, \theta (radians)');
title('Step Reference with Kp = 21, Ki = 500 and Different Values of Kd');
legend('Kd = 0.05', 'Kd = 0.15', 'Kd = 0.25');

distcl = feedback(motor,C);
figure
step(distcl(:,:,1), distcl(:,:,2), distcl(:,:,3));
ylabel('Position, \theta (radians)');
title('Step Disturbance with Kp = 21, Ki = 500 and Different values of Kd');
legend('Kd = 0.05', 'Kd = 0.15', 'Kd = 0.25');

figure
step(syscl(:,:,2));
ylabel('Position, \theta (radians)');
title('Response to a Step Reference');
figure
step(distcl(:,:,2));
ylabel('Position, \theta (radians)');
title('Response to a Step Disturbance');

figure
rlocus(motor);
axis([ -300 100 -400 400]);
sgrid(.5,0);
sigrid(115);

figure
C = (s + 50)/s;
rlocus(C*motor);
axis([ -300 100 -400 400]);
sgrid(.5,0);
sigrid(115);

figure
C = (s + 50)/s*(1/80*s+1);
rlocus(C*motor);
axis([ -300 100 -400 400]);
sgrid(.5,0);
sigrid(115);


K = 11;
syscl=feedback(K*C*motor,1);
discl=feedback(motor,K*C);
figure
step(syscl);
figure
step(discl);

figure
margin(motor);

C = 1/s;
figure
margin(C*motor);

C = 1/s*((s/40+1)/(s/2000+1))^2;
figure
bode(C*motor);

K = 450;

syscl=feedback(K*C*motor,1);
discl=feedback(motor,K*C);

figure
step(syscl);
figure
step(discl);

function[ ] = sigrid(sig)
    hold on
    limits = axis;
    mx=limits(1,4);
    mn=limits(1,3);
    stz=abs(mx)+abs(mn);
    st=stz/50;
    im=mn:st:mx;
    lim=length(im);
    for i=1:lim
        re(i)=-sig;
    end
    re(:);
    plot(re,im,'.')
    hold off
    return
end

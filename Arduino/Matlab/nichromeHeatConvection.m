l = 0.15;
a = 0.32 * 10^-3;
v = 1.32;
r = 2.5;

wN = 95.1 * 10-6;
hN = 450;

wP = 0.01;
hP = 1.46;

i = v / r;
figure;
for t = 1:1:400
    h = i^2 * r * t;
    dt = (h * t) / (hN*wN);
    
    plot(t,dt);
    hold on
    %disp(t + " = " + dt); 
end


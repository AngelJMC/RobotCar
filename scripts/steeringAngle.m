offset =20
signal = [60:10:140]
theta_l= [80 83 87 90 93 97 103 112 119]
theta_r= [60 68 75 82 89 93 99 104 107]

plot(theta_l - 90,signal, theta_r -90, signal)

grid on
hold on

theta_avg = ( theta_l + theta_r ) / 2 - 90
plot(theta_avg, signal)

p = polyfit( theta_avg, signal, 1)


aprox = p(1)*theta_avg + p(2) + offset;

plot(theta_avg, aprox)
syms t s

G=tf(100,[1 2 1],'InputDelay',0.1);
T=0.1;
G_z=c2d(G,T,'zoh')


%phi = 0;
syms phi
assume(phi, 'real')

A = [cos(phi), -sin(phi), 0;
     sin(phi),  cos(phi), 0;
            0,         0, 1];

C = [-sin(phi), -cos(phi), 0;
      cos(phi), -sin(phi), 0;
             0,         0, 0];
        
syms m
assume(m, 'real')
syms j
assume(j, 'real')
B = [m, 0, 0;
     0, m, 0;
     0, 0, j];
 
C = A'*B*C; %A'*B*C
C = simplify(C)

%theta = [0, pi/2, pi, 3*pi/2];
syms theta1 theta2 theta3 theta4;
assume(theta1, 'real')
assume(theta2, 'real')
assume(theta3, 'real')
assume(theta4, 'real')
theta = [theta1, theta2, theta3, theta4];
syms l

G = [-sin(theta(1)), -sin(theta(2)), -sin(theta(3)), -sin(theta(4));
      cos(theta(1)),  cos(theta(2)),  cos(theta(3)),  cos(theta(4));
                  l,              l,              l,              l];

D = pinv(G);
E = D';
%F = B*E
G = D*B*E;
%D = simplify(D)
%E = simplify(E)
%F = simplify(F)
G = simplify(G)
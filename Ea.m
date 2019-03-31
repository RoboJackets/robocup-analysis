function y = Ea(u)
y = 0;
%u = abs(rem(u, 2*pi()));
if (0 <= u && u < 2*pi()/3)
    y = 1;
elseif (2*pi()/3 <= u && u < pi())
    y = 1 - (6/pi())*(u - (2*pi()/3));
elseif (pi() <= u && u < (5*pi()/3))
    y = -1;
else
    y = -1 + (6/pi())*(u - (5*pi()/3));
end

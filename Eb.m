function y = Eb(u)
y = 0;
% u = abs(rem(u, 2*pi()));
% u = u - (2*pi()/3);

% if u < 0
%      u = 2*pi() + u;
% end

if (0 <= u && u < pi()/3)
    y = -1;
elseif (pi()/3 <= u && u < 2*pi()/3)
    y = -1 + (6/pi())*(u - (pi()/3));
elseif (2*pi()/3 <= u && u < (4*pi()/3))
    y = 1;
elseif ((4*pi()/3) <= u && u < 5*pi()/3)
    y = 1 - ((6/pi()) * (u - (4*pi()/3)));
elseif (5*pi()/3 <= u && u < 2*pi())
    y = -1;
end

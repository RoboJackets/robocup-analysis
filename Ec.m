function y = Ec(u)
y = 0;
% u = u - (4*pi()/3);
% u = abs(rem(u, 2*pi()));

% if u < 0
%      u = 2*pi() + u;
% end

if (0 <= u && u < pi()/3)
    y = 1 - (6/pi())*(u);
elseif (pi()/3 <= u && u < pi())
    y = -1;
elseif (pi() <= u && u < (4*pi()/3))
    y = -1 + (6/pi())*(u - pi());
elseif ((4*pi()/3) <= u && u < 2*pi())
    y = 1;
end

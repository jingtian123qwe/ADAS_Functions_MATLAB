function dxdt = VehicleTrailerAModelCT(x, u)

carLength = 5;
lv = carLength;
lt = 3;
lc = 0;

theta = x(3);
theta_t = x(4);
V = -3;
delta = u;


dxdt = [V*cos(theta-theta_t)*cos(theta_t); V*cos(theta-theta_t)*sin(theta_t);...
    tan(delta)/carLength*V;...
    1/lt*(V*sin(theta-theta_t) - V*lc/lv*tan(delta)*cos(theta-theta_t))];
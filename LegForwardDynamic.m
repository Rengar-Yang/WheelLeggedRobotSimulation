function [L,Theta] = LegForwardDynamic(u1,u4,l1,l2,l3,l4,l5)
    u1 = pi+u1;% 角度变换
    u4 = pi+u4;

    xb = l1 * cos(u1);% 公式
    yb = l1 * sin(u1);
    xd = l5 + l4 * cos(u4);
    yd = l4 * sin(u4);
    lbd = sqrt((xd - xb)^2 + (yd - yb)^2);
    A0 = 2 * l2 * (xd - xb);
    B0 = 2 * l2 * (yd - yb);
    C0 = l2^2 + lbd^2 - l3^2;
    u2 = 2 * atan2((B0 + sqrt(A0^2 + B0^2 - C0^2)),(A0 + C0));
    xc = xb + l2 * cos(u2);
    yc = yb + l2 * sin(u2);

    L = sqrt((xc-l5/2)^2+yc^2);% 直角坐标转换成极坐标
    Theta = atan2(yc,(xc-l5/2))-pi/2;
end

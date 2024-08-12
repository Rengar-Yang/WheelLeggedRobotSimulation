function pos = LegPosition(l1,l2,l3,l4,l5,phi1,phi4)
%LegPosition
%    POS = LegPosition(L1,L2,L3,L4,L5,PHI1,PHI4)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    2024-08-10 23:17:37

t2 = cos(phi1);
t3 = cos(phi4);
t4 = sin(phi1);
t5 = sin(phi4);
t6 = l2.^2;
t7 = l3.^2;
t12 = l5./2.0;
t8 = l1.*t2;
t9 = l4.*t3;
t10 = l1.*t4;
t11 = l4.*t5;
t13 = -t7;
t16 = -t12;
t14 = -t8;
t15 = -t11;
t17 = t10+t15;
t18 = l5+t9+t14;
t19 = t17.^2;
t20 = t18.^2;
t21 = l2.*t17.*2.0;
t23 = l2.*t18.*2.0;
t24 = t6.*t19.*4.0;
t25 = t6.*t20.*4.0;
t26 = t6+t13+t19+t20;
t27 = t26.^2;
t29 = t23+t26;
t28 = -t27;
t30 = 1.0./t29;
t31 = t24+t25+t28;
t32 = sqrt(t31);
t34 = -t30.*(t21-t32);
t35 = atan(t34);
t36 = t35.*2.0;
t37 = cos(t36);
t38 = sin(t36);
t39 = l2.*t37;
pos = [sqrt((t8+t16+t39).^2+(t10+l2.*t38).^2);atan2(t10+l2.*t38,t8+t16+t39)];
end

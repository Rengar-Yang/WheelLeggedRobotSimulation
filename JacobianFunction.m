function J = JacobianFunction(u1,u4,l1,l2,l3,l4,l5)
%JacobianFunction
%    J = JacobianFunction(U1,U4,L1,L2,L3,L4,L5)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    2024-07-29 22:35:58

t2 = cos(u1);
t3 = cos(u4);
t4 = sin(u1);
t5 = sin(u4);
t6 = l2.^2;
t7 = l3.^2;
t12 = l5./2.0;
t8 = l1.*t2;
t9 = l4.*t3;
t10 = l1.*t4;
t11 = l4.*t5;
t17 = -t7;
t13 = l2.*t8.*2.0;
t14 = l2.*t9.*2.0;
t15 = l2.*t10.*2.0;
t16 = l2.*t11.*2.0;
t18 = -t9;
t20 = -t11;
t25 = t10+t20;
t26 = l5+t8+t18;
t27 = t25.^2;
t28 = t26.^2;
t29 = l2.*t25.*2.0;
t30 = l2.*t26.*2.0;
t31 = t8.*t25.*2.0;
t32 = t9.*t25.*2.0;
t33 = l2.*t25.*2.0i;
t34 = t10.*t26.*2.0;
t35 = t11.*t26.*2.0;
t40 = t6.*t8.*t25.*8.0;
t41 = t6.*t9.*t25.*8.0;
t44 = t6.*t10.*t26.*8.0;
t45 = t6.*t11.*t26.*8.0;
t36 = t6.*t27.*4.0;
t37 = -t34;
t38 = -t35;
t39 = t6.*t28.*4.0;
t42 = -t40;
t43 = -t41;
t48 = t6+t17+t27+t28;
t46 = t31+t37;
t47 = t32+t38;
t49 = t48.^2;
t50 = -t49;
t51 = t46.*t48.*2.0;
t52 = t47.*t48.*2.0;
t53 = t36+t39+t50;
t62 = t42+t44+t51;
t63 = t43+t45+t52;
t54 = sqrt(t53);
t55 = 1.0./t54;
t56 = imag(t54);
t57 = real(t54);
t59 = t54.*1i;
t58 = -t56;
t60 = t29+t57;
t68 = t30+t33+t48+t59;
t82 = t55.*t62;
t83 = t55.*t63;
t61 = t60.^2;
t64 = t30+t48+t58;
t69 = angle(t68);
t84 = imag(t82);
t85 = imag(t83);
t86 = real(t82);
t87 = real(t83);
t65 = t64.^2;
t66 = 1.0./t64;
t70 = t69.*2.0;
t88 = t84./2.0;
t89 = t85./2.0;
t90 = t86./2.0;
t91 = t87./2.0;
t67 = 1.0./t65;
t71 = cos(t70);
t72 = sin(t70);
t94 = t61+t65;
t73 = l2.*t71;
t74 = l2.*t72;
t95 = 1.0./t94;
t75 = -t73;
t77 = (t10-t74).^2;
t107 = t65.*t73.*t95.*(t66.*(t13-t90)+t60.*t67.*(t15-t31+t34-t88)).*-2.0;
t108 = t65.*t74.*t95.*(t66.*(t13-t90)+t60.*t67.*(t15-t31+t34-t88)).*-2.0;
t78 = t8+t12+t75;
t109 = t8+t107;
t110 = t10+t108;
t79 = t78.^2;
t80 = 1.0./t78;
t81 = 1.0./t79;
t98 = t77+t79;
t99 = 1.0./t98;
t100 = 1.0./sqrt(t98);
J = reshape([t100.*(t78.*t110.*2.0-t109.*(t10-t74).*2.0).*(-1.0./2.0),t79.*t99.*(t80.*t109+t81.*t110.*(t10-t74)),t100.*(t65.*t74.*t78.*t95.*(t66.*(t14-t91)+t60.*t67.*(t16-t32+t35-t89)).*4.0-t65.*t73.*t95.*(t66.*(t14-t91)+t60.*t67.*(t16-t32+t35-t89)).*(t10-t74).*4.0).*(-1.0./2.0),t79.*t99.*(t65.*t73.*t80.*t95.*(t66.*(t14-t91)+t60.*t67.*(t16-t32+t35-t89)).*2.0+t65.*t74.*t81.*t95.*(t66.*(t14-t91)+t60.*t67.*(t16-t32+t35-t89)).*(t10-t74).*2.0)],[2,2]);
end

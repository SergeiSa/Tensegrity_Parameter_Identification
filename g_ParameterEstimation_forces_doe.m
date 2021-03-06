function external_forces = g_ParameterEstimation_forces_doe(in1)
%G_PARAMETERESTIMATION_FORCES_DOE
%    EXTERNAL_FORCES = G_PARAMETERESTIMATION_FORCES_DOE(IN1)

%    This function was generated by the Symbolic Math Toolbox version 8.4.
%    02-Sep-2020 17:53:32

r1_1 = in1(1);
r1_2 = in1(4);
r1_3 = in1(7);
r1_4 = in1(10);
r1_5 = in1(13);
r1_6 = in1(16);
r2_1 = in1(2);
r2_2 = in1(5);
r2_3 = in1(8);
r2_4 = in1(11);
r2_5 = in1(14);
r2_6 = in1(17);
r3_1 = in1(3);
r3_2 = in1(6);
r3_3 = in1(9);
r3_4 = in1(12);
r3_5 = in1(15);
r3_6 = in1(18);
t2 = -r1_4;
t3 = -r1_5;
t4 = -r1_6;
t5 = -r2_4;
t6 = -r2_5;
t7 = -r2_6;
t8 = -r3_4;
t9 = -r3_5;
t10 = -r3_6;
t11 = r1_1.*r2_4.*1.0e+1;
t12 = r1_4.*r2_1.*1.0e+1;
t13 = r1_2.*r2_5.*1.0e+1;
t14 = r1_5.*r2_2.*1.0e+1;
t15 = r1_3.*r2_6.*1.0e+1;
t16 = r1_6.*r2_3.*1.0e+1;
t17 = r1_1.*r3_4.*1.0e+1;
t18 = r1_4.*r3_1.*1.0e+1;
t19 = r1_2.*r3_5.*1.0e+1;
t20 = r1_5.*r3_2.*1.0e+1;
t21 = r1_3.*r3_6.*1.0e+1;
t22 = r1_6.*r3_3.*1.0e+1;
t23 = r2_1.*r3_4.*1.0e+1;
t24 = r2_4.*r3_1.*1.0e+1;
t25 = r2_2.*r3_5.*1.0e+1;
t26 = r2_5.*r3_2.*1.0e+1;
t27 = r2_3.*r3_6.*1.0e+1;
t28 = r2_6.*r3_3.*1.0e+1;
t29 = r1_1+t2;
t30 = r1_2+t3;
t31 = r1_3+t4;
t32 = r2_1+t5;
t33 = r2_2+t6;
t34 = r2_3+t7;
t35 = r3_1+t8;
t36 = r3_2+t9;
t37 = r3_3+t10;
t38 = t29.^2;
t39 = t30.^2;
t40 = t31.^2;
t41 = t32.^2;
t42 = t33.^2;
t43 = t34.^2;
t44 = t35.^2;
t45 = t36.^2;
t46 = t37.^2;
t47 = t38+t41+t44;
t48 = t39+t42+t45;
t49 = t40+t43+t46;
t50 = sqrt(t47);
t51 = sqrt(t48);
t52 = sqrt(t49);
t53 = 1.0./t50;
t54 = 1.0./t51;
t55 = 1.0./t52;
t56 = t50.*1.0e+1;
t57 = t51.*1.0e+1;
t58 = t52.*1.0e+1;
t59 = r1_1.*r2_4.*t53.*5.0;
t60 = r1_4.*r2_1.*t53.*5.0;
t61 = r1_2.*r2_5.*t54.*5.0;
t62 = r1_5.*r2_2.*t54.*5.0;
t63 = r1_1.*r3_4.*t53.*5.0;
t64 = r1_4.*r3_1.*t53.*5.0;
t65 = r1_3.*r2_6.*t55.*5.0;
t66 = r1_6.*r2_3.*t55.*5.0;
t67 = r1_2.*r3_5.*t54.*5.0;
t68 = r1_5.*r3_2.*t54.*5.0;
t69 = r2_1.*r3_4.*t53.*5.0;
t70 = r2_4.*r3_1.*t53.*5.0;
t71 = r1_3.*r3_6.*t55.*5.0;
t72 = r1_6.*r3_3.*t55.*5.0;
t73 = r2_2.*r3_5.*t54.*5.0;
t74 = r2_5.*r3_2.*t54.*5.0;
t75 = r2_3.*r3_6.*t55.*5.0;
t76 = r2_6.*r3_3.*t55.*5.0;
t77 = t56-5.0;
t78 = t57-5.0;
t79 = t58-5.0;
t80 = t29.*t53.*t77;
t81 = t30.*t54.*t78;
t82 = t32.*t53.*t77;
t83 = t31.*t55.*t79;
t84 = t33.*t54.*t78;
t85 = t35.*t53.*t77;
t86 = t34.*t55.*t79;
t87 = t36.*t54.*t78;
t88 = t37.*t55.*t79;
external_forces = [-t80+t83;-t82+t86;-t85+t88;t23-t24-t27+t28-t69+t70+t75-t76;-t17+t18+t21-t22+t63-t64-t71+t72;t11-t12-t15+t16-t59+t60+t65-t66;t80-t81;t82-t84;t85-t87;-t23+t24+t25-t26+t69-t70-t73+t74;t17-t18-t19+t20-t63+t64+t67-t68;-t11+t12+t13-t14+t59-t60-t61+t62;t81-t83;t84-t86;t87-t88;-t25+t26+t27-t28+t73-t74-t75+t76;t19-t20-t21+t22-t67+t68+t71-t72;-t13+t14+t15-t16+t61-t62-t65+t66];

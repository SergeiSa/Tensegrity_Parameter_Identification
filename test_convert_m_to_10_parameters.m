N = 10;


% r = randn(3, N);
r = sym('r', [3, N]);
m = sym('m', [N, 1]);

In = sym(zeros(6, 1));

for i = 1:N
In(1) = In(1) + (r(2, i)^2 + r(3, i)^2)*m(i);
In(2) = In(2) + (r(3, i)^2 + r(1, i)^2)*m(i);
In(3) = In(3) + (r(1, i)^2 + r(2, i)^2)*m(i);

In(4) = In(4) - r(1, i) * r(2, i) * m(i);
In(5) = In(5) - r(1, i) * r(3, i) * m(i);
In(6) = In(6) - r(2, i) * r(3, i) * m(i);
end

pC = r*m;
M = sum(m);

exp = [In; pC; M]

J = jacobian(exp, m);
J = simplify(J)

rank(J)

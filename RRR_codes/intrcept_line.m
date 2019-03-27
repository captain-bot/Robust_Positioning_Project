function[max_intrcpt] = intrcept_line(Jmat, cval, v)
    L = chol(inv(Jmat*Jmat')/cval, 'lower');
    w = inv(L)*v;
    max_intrcpt = norm(w);
end
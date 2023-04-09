

function dJ =  Jacobian(J, q)
    N = size(J);
    M = size(q);
    for i = 1:M
        for j = 1:N
           dJ(i,j) = diff(J(i),q(j));
           dJ(i,j) = simplify(dJ(i,j));
        end
    end
end
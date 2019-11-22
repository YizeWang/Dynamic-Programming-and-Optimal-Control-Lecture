function diffG = CompareResultG(G1,G2)
for i = 1:size(G1, 1)
    for j = 1:size(G1, 2)
        if isinf(G1(i,j)) && isinf(G2(i,j))
            G1(i,j) = 0;
            G2(i,j) = 0;
        end
    end
end

tol = 1e-5;
diff = G1 - G2;

for i = 1:size(G1, 1)
    for j = 1:size(G1, 2)
        if diff(i,j) < tol
            diff(i,j) = 0;
        end
    end
end

[x,y] = find(diff);
diffG = [x y];

end


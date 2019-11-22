function diffP = CompareResultP(P1,P2)
if isequal(P1, P2)
    diffP = [];
else
    diffP1 = P1(:,:,1) - P2(:,:,1);
    diffP2 = P1(:,:,2) - P2(:,:,2);
    diffP3 = P1(:,:,3) - P2(:,:,3);
    diffP4 = P1(:,:,4) - P2(:,:,4);
    diffP5 = P1(:,:,5) - P2(:,:,5);
    [x1,y1] = find(diffP1);
    [x2,y2] = find(diffP2);
    [x3,y3] = find(diffP3);
    [x4,y4] = find(diffP4);
    [x5,y5] = find(diffP5);
    diffP = [x1 y1 1*ones(size(x1,1),1);
             x2 y2 2*ones(size(x2,1),1);
             x3 y3 3*ones(size(x3,1),1);
             x4 y4 4*ones(size(x4,1),1);
             x5 y5 5*ones(size(x5,1),1)];
end


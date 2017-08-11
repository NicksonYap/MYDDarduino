%prevents rounding when displaying fractions
format long
%Reader 3-D Coordinates [x, y, z]
Reader = [-2, 2, 2; 2, 1, -2; -1, -2, 2; 3, 3, 3];
%plot reader locations
scatter3(getcolumn(Reader(1:4,1:3),1),getcolumn(Reader(1:4,1:3),2),getcolumn(Reader(1:4,1:3),3), 'MarkerEdgeColor', [1 0 0], 'MarkerFaceColor', [1 0 0]);figure(gcf)
%x = 0;0;0;
%y = 0;0;0;
%x = getcolumn(Reader(1:3,1:2),1)
%y = getcolumn(Reader(1:3,1:2),2)
%e = [1;1;1]
hold on
%errorbar(x,y, e, 'og', 'Marker', '+');
axis([-3 3 -3 3 -3 3]) %set axis for 2-D graphs
set(gca, 'XTick', -3:1:3);
set(gca, 'YTick', -3:1:3);
set(gca, 'ZTick', -3:1:3);
grid on;
%distances to Tag from Reader(i)
%Distance = [2.82842715;2.236067977;2.236067977]; %(0, 0) tag
Distance = [3.464101615; 3; 3; 5.196152423]; %(0, 0 , 0) tag
%initialize variables
x = 0;
y = 0;
z = 0;
[x, y, z] = three_tri(Reader(1, 1), Reader(1, 2), Reader(1, 3), Distance(1), Reader(2,1), Reader(2, 2), Reader(2, 3), Distance(2), Reader(3, 1), Reader(3, 2), Reader(3, 3),Distance(3), Reader(4, 1), Reader(4, 2), Reader(4, 3), Distance(4));
scatter3(x, y, z, 'MarkerEdgeColor', [1 1 0], 'MarkerFaceColor', [1 1 0]);
axis square 
figure(gcf)

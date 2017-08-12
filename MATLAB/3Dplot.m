%prevents rounding when displaying fractions
format long
%Reader 3-D Coordinates [x, y, z]
%Reader = [0,0,0; 0,0,242*4; 0,206*4,242*4; 422*4,206*4,242*4];
Reader = [530,600,300; 530,600,0; 0,600,0; 0, 0, 0];
%plot reader locations
figure(gcf)
scatter3(getcolumn(Reader(1:4,1:3),1),getcolumn(Reader(1:4,1:3),2),getcolumn(Reader(1:4,1:3),3), 'MarkerEdgeColor', [1 0 0], 'MarkerFaceColor', [1 0 0]);
axis([-50 1000 -50 1000 -50 1000]) %set axis for 2-D graphs
rotate3d;
drawnow;
%x = 0;0;0;
%y = 0;0;0;
%x = getcolumn(Reader(1:3,1:2),1)
%y = getcolumn(Reader(1:3,1:2),2)
%e = [1;1;1]
hold on
%errorbar(x,y, e, 'og', 'Marker', '+');

axis square 
%set(gca, 'XTick', -50:10:500);
%set(gca, 'YTick', -50:10:500);
%set(gca, 'ZTick', -50:10:500);
grid on;

%pause

if(numel(instrfind) > 0)
    fclose(instrfind); %close all ports before opening
end

s = serial('COM5');
set(s,'Timeout',5);
set(s,'DataBits',8);
set(s,'StopBits',1);
set(s,'BaudRate',115200);
set(s,'Parity','none');
s.ReadAsyncMode='continuous';
fopen(s);
drawLoc = 0;

sampleCount = 0;
while(1)
    f = fscanf(s, '%f,%f,%f,%f\n');
    if(numel(f) == 4)
       

        %distances to Tag from Reader(i)
        %Distance = [2.82842715;2.236067977;2.236067977]; %(0, 0) tag
        %Distance = [3.464101615; 3; 3; 5.196152423]; %(0, 0 , 0) tag
        %initialize variables
        
        Distance = f;
        delete(findobj('MarkerEdgeColor',[0 0 1]))
        if(f(1) ~= -1 && f(2) ~= -1 && f(3) ~= -1 && f(4) ~= -1)
            [x, y, z] = three_tri(Reader(1, 1), Reader(1, 2), Reader(1, 3), Distance(1), Reader(2,1), Reader(2, 2), Reader(2, 3), Distance(2), Reader(3, 1), Reader(3, 2), Reader(3, 3),Distance(3), Reader(4, 1), Reader(4, 2), Reader(4, 3), Distance(4));
            drawLoc = scatter3(x, y, z, 'MarkerEdgeColor', [0 0 1], 'MarkerFaceColor', [0 0 1]);
            drawnow;
            
            sampleCount = sampleCount + 1;
            
            if(sampleCount >= 10)
                response = webwrite('http://api.seamlesshq.com/tests/MYDD3dpos','x', x, 'y', y, 'z', z);
                disp(response);
                sampleCount= 0;
            end
        end
       
    end
    disp(f);
    %response = webwrite('https://requestb.in/st2nilst','f1',f(1));
    %disp(response);
end
fclose(s);
fclose(instrfind); 


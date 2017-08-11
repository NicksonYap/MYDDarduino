
fclose(instrfind); %close all ports before opening
s = serial('COM6');
set(s,'Timeout',1);
set(s,'DataBits',8);
set(s,'StopBits',1);
set(s,'BaudRate',115200);
set(s,'Parity','none');
s.ReadAsyncMode='continuous';
fopen(s);

sampleCount = 0;
figure(1); clf

 old_f = [-1,-1,-1,-1];
while(1)
    f = fscanf(s, '%f,%f,%f,%f\n');
    if(numel(f) == 4)
        %plot(sampleCount, f(1));
        if(f(1) ~= -1)
            if(old_f(1)== -1)
                old_f(1) = f(1);
            end
            plot([sampleCount - 1; sampleCount], [old_f(1); f(1)], '-r.')
        end
        if(f(2) ~= -1)
            if(old_f(2)== -1)
                old_f(2) = f(2);
            end
            plot([sampleCount - 1; sampleCount], [old_f(2); f(2)], '-g.')
        end
        if(f(3) ~= -1)
            if(old_f(3)== -1)
                old_f(3) = f(3);
            end
            plot([sampleCount - 1; sampleCount], [old_f(3); f(3)], '-b.')
        end
        if(f(4) ~= -1)
            if(old_f(4)== -1)
                old_f(4) = f(4);
            end
            plot([sampleCount - 1; sampleCount], [old_f(4); f(4)], '-k.')
        end
        old_f = f;
        axis([sampleCount-50 sampleCount 0 5])
        hold on
        drawnow;
        
    end
    disp(f);
    sampleCount = sampleCount+1;
    %C = strsplit(f,',');
    %disp(C);
end
fclose(s);
fclose(instrfind); 

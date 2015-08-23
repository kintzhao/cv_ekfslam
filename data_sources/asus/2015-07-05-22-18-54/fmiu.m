close all;
[hang,lie]=size(miu);
color=['r.','g.','b.','y','k.'];
for i=1:hang
    base=(lie-3)/10;
    figure(base_i);plot(miu(i,1),miu(i,2),'k+') ;hold on    %plot robot
    figure(200) ;plot(miu(i,1),miu(i,2),'k+') ;hold on    %plot robot
    for base_i=1:base
        for j=0:4
            if miu(i,(base_i-1)*10+3+2*j)~=NaN
                figure(base_i);plot(miu(i,(base_i-1)*10+3+2*j+1),miu(i,(base_i-1)*10+3+2*j+2),color(j+1)) ;hold on  %plot mark
                figure(200) ;plot(miu(i,(base_i-1)*10+3+2*j+1),miu(i,(base_i-1)*10+3+2*j+2),color(j+1)) ;hold on  %plot mark
            end
            
        end
    end
end






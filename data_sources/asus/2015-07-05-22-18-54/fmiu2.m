%
%         
% 
%
%%
close all;
[hang,lie]=size(miu);
miu2=miu;
for i=1:hang
    for j =1:lie
        if isnan(miu2(i,j))
            miu2(i,j)=0;
        
        end
    end
end
color=['r.','g.','b.','y','k.'];
[hang,lie]=size(miu2);
base=(lie-3)/10;
figure(1)   ;plot(miu2(1:hang,1),miu2(1:hang,2),'k+') ;hold on    %plot robot
figure(200) ;plot(miu2(1:hang,1),miu2(1:hang,2),'k+') ;hold on    %plot robot
for base_i=1:base
    for j=0:4
        figure(base_i);plot(miu2(1:hang,(base_i-1)*10+3+2*j+1),miu2(1:hang,(base_i-1)*10+3+2*j+2),color(j+1)) ;hold on  %plot mark
        grid on;
        figure(200)   ;plot(miu2(1:hang,(base_i-1)*10+3+2*j+1),miu2(1:hang,(base_i-1)*10+3+2*j+2),color(j+1)) ;hold on  %plot mark
    end
end
grid on;








% 
% 比较慢
% 
% color=['r.','g.','b.','y','k.'];
% for i=1:hang
%     base=(lie-3)/10;
%     figure(base_i);plot(miu(i,1),miu(i,2),'k+') ;hold on    %plot robot
%     figure(200) ;plot(miu(i,1),miu(i,2),'k+') ;hold on    %plot robot
%     for base_i=1:base
%         for j=0:4
%             if miu(i,(base_i-1)*10+3+2*j)~=NaN
%                 figure(base_i);plot(miu(i,(base_i-1)*10+3+2*j+1),miu(i,(base_i-1)*10+3+2*j+2),color(j+1)) ;hold on  %plot mark
%                 figure(200) ;plot(miu(i,(base_i-1)*10+3+2*j+1),miu(i,(base_i-1)*10+3+2*j+2),color(j+1)) ;hold on  %plot mark
%             end
%             
%         end
%     end
% end

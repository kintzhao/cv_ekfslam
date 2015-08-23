close all;
a=11;
[hang,lie]=size(miu);
color=['r.','g.','b.','y','k.'];
for i=1:hang
    base=(lie-3)/10
    figure(base);plot(miu(i,1),miu(i,2),'k+') ;hold on    //plot robot
    figure(200) ;plot(miu(i,1),miu(i,2),'k+') ;hold on    //plot robot
    for base_i=1:base
        for j=1:5
            figure(base);plot(miu(i,(base_i-1)*10+3+2*j),miu(i,(base_i-1)*10+3+2*j+1),color(j)) ;hold on  //plot mark
            figure(200) ;plot(miu(i,(base_i-1)*10+3+2*j),miu(i,(base_i-1)*10+3+2*j+1),color(j)) ;hold on  //plot mark
        end
    end
end

      
  
  
  

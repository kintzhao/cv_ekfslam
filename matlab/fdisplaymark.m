 function [] = fdisplaymark(marks,num)
% marks = qrmeterdata;
% num = 1;
figure(num)
[hang,lie] = size(marks);
title('r g b k y 0 1 2 3 middle');
axis equal;
plot(0,0,'r+');hold on;
line([0,10],[0,0]) ;hold on
line([0,0],[0,10]) ;hold on
for row = 500:1:hang
    k = marks(row,1);
    row
    for num = 1:k
        figure(marks(row, 2+11*(num-1)));
        plot(marks(row, 2+11*(num-1)+1),marks(row, 2+11*(num-1)+2),'r*'); hold on;
        line( [marks(row, 2+11*(num-1)+1), marks(row, 2+11*(num-1)+3) ],[ marks(row, 2+11*(num-1)+2), marks(row, 2+11*(num-1)+4) ] );hold on;
        
        plot(marks(row, 2+11*(num-1)+3),marks(row, 2+11*(num-1)+4),'g*');hold on;
        line( [marks(row, 2+11*(num-1)+3), marks(row, 2+11*(num-1)+5) ],[ marks(row, 2+11*(num-1)+4), marks(row, 2+11*(num-1)+6) ] );hold on;
        
        plot(marks(row, 2+11*(num-1)+5),marks(row, 2+11*(num-1)+6),'b*');hold on;
        line( [marks(row, 2+11*(num-1)+5), marks(row, 2+11*(num-1)+7) ],[ marks(row, 2+11*(num-1)+6), marks(row, 2+11*(num-1)+8) ] );hold on;
        
        plot(marks(row, 2+11*(num-1)+7),marks(row, 2+11*(num-1)+8),'k*'); hold on;
        line( [marks(row, 2+11*(num-1)+7), marks(row, 2+11*(num-1)+1) ],[ marks(row, 2+11*(num-1)+8), marks(row, 2+11*(num-1)+2) ] );hold on;
        
        plot(marks(row, 2+11*(num-1)+9),marks(row, 2+11*(num-1)+10),'y*');hold on;
    end
end
grid on;



 
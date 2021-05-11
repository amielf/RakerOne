% arr = xlsread('subsection.xlsx');
ptCloud = pcread('1086.029000000.pcd');
x=ptCloud.Location(:,1);
y=ptCloud.Location(:,2);
z=ptCloud.Location(:,3);

plot3(x,y,z,'.')

%xbase = min(x);
%ybase = min(y);
xbase = -6;
ybase = 13;
planes = [];
figure(1)
hold on

for a = min(x)+1:1:max(x)
    for b = min(y)+1:1:max(y)
        newrow = 1;
        temp = [0,0,0];
        for i=1:length(x)
            if (x(i) < a) && (x(i) >= xbase)
                if (y(i) < b) && (y(i) >= ybase)
                    temp(newrow,1) = x(i); 
                    temp(newrow,2) = y(i);
                    temp(newrow,3) = z(i);
                    %temp(newrow,4) = 1;
                    newrow = newrow+1;
                end
            end
        end
        DM = [temp(:,1), temp(:,2), ones(size(temp(:,3)))];
        if length(DM) > 20
            B = DM\temp(:,3);
            [X,Y] = meshgrid(linspace(xbase,a,2), linspace(ybase,b,2));
            Z = B(1)*X + B(2)*Y + B(3)*ones(size(X));
            
            %if z < 5
            surf(X,Y,Z)
            %end
            planes(length(planes)+1,:)=B';
        end
        ybase = b;
    end
    xbase = a;
end
plot3(x,y,z,'.')
%                         
% figure(1)
% plot3(arr(:,1),arr(:,2),arr(:,3),'.')
% DM = [x, y, ones(size(z))];                             % Design Matrix
% B = DM\z;                                               % Estimate Parameters
% [X,Y] = meshgrid(linspace(min(x),max(x),50), linspace(min(y),max(y),50));
% Z = B(1)*X + B(2)*Y + B(3)*ones(size(X));
% figure(2)
% plot3(arr(:,1),arr(:,2),arr(:,3),'.')
% hold on
% meshc(X, Y, Z)
% hold off
% grid on
% xlabel('x(m)'); ylabel('y(m)'); zlabel('z(m)');
% title('Masked plot');
% grid on
% text(-20, 50, 450, sprintf('Z = %.3f\\cdotX %+.3f\\cdotY %+3.0f', B))
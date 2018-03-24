function nodes=nativeExplore
    close all
    x_max = 100;
    y_max = 100;
    q=[x_max/2 y_max/2];
    scatter(x_max/2,y_max/2)
    Nbnodes= 3000;
    nodes(1,:)=q;
    delta=1;
    
    figure(1)
    axis equal
    axis([0 x_max 0 y_max]);
    scatter(q(1),q(2),'ob')
    hold on
    for i=2:Nbnodes
        angle=-pi+rand(1)*2*pi;
        q(1)=q(1)+delta*cos(angle);
        q(2)=q(2)+delta*sin(angle);
        nodes(i,:)=q;
        line(nodes(i-1:i,1),nodes(i-1:i,2),'Color','k');
    end
    axis equal
    axis([0 x_max 0 y_max]);
end
function RRTexplore(q_ini,q_fin)

%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
    x_max = 50;
    y_max = 50;
    q=[x_max/2 y_max/2];
    Nbnodes= 100;
    nodes(1,:)=q;
    delta=5;
    
    figure
    axis equal
    axis([0 x_max 0 y_max]);
    scatter(q(1),q(2),'ob')
    hold on
    
    
    for j = 1:length(nodes)
        q_rand = [floor(rand(1)*x_max) floor(rand(1)*y_max)];
        scatter(q_rand(1), q_rand(2),'k','filled')
        if nodes(j,:)== q_fini
            break
        end
    end
    
    distances = [];
    for i = 1:length(nodes)
        n = nodes(i,:);
        d=dist(n, q_rand);
        distances = [distances d];
    end
    [d, indice] = min(distances);
    q_near = nodes(indice);
    
    q_new = steer(q_rand, q_near, d, eps);
    line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'Color', 'k', 'LineWidth', 2);
    drawnow
    hold on
    q_new.cost = dist(q_new.coord, q_near.coord) + q_near.cost;
    q_nearest = [];
    r = 60;
    neighbor_count = 1;
        for j = 1:1:length(nodes)
            if noCollision(nodes(j).coord, q_new.coord, obstacle) && dist(nodes(j).coord, q_new.coord) <= r
                q_nearest(neighbor_count).coord = nodes(j).coord;
                q_nearest(neighbor_count).cost = nodes(j).cost;
                neighbor_count = neighbor_count+1;
            end
        end
        q_min = q_near;
        C_min = q_new.cost;
        
        for k = 1:1:length(q_nearest)
            if noCollision(q_nearest(k).coord, q_new.coord, obstacle) && q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord) < C_min
                q_min = q_nearest(k);
                C_min = q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord);
                line([q_min.coord(1), q_new.coord(1)], [q_min.coord(2), q_new.coord(2)], 'Color', 'g');                
                hold on
            end
        end
        
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_min.coord
                q_new.parent = j;
            end
        end
        
        % Append to nodes
        nodes = [nodes q_new];
    end
end
    
    for i=2:Nbnodes
        q_rand=[rand*x_max rand*y_max]
        
    end
end


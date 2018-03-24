clear all
close all

M=100;
eps=0.1;
x_max = 100;
y_max = 100;
obstacle = [70,20,5,60];
obstacle2 =[10,40,50,20];
epsilon=1;
numNodes =3000;    

q_start.coord = [0 99];
scatter(q_start.coord(1),q_start.coord(2));
hold on

q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = [75 10];
q_goal.son=0;
scatter(q_goal.coord(1),q_goal.coord(2));

q_goal.cost = 0;

commandes=[];

nodesi(1) = q_start;
nodesf(1)=q_goal;

figure(1)
axis equal
axis([0 x_max 0 y_max])
rectangle('Position',obstacle,'FaceColor','b')
rectangle('Position',obstacle2,'FaceColor','b')
hold on

q_new=q_start;
count=1;
while count<numNodes && dist(q_new.coord, q_goal.coord)>epsilon
    q_rand = [floor(rand(1)*x_max) floor(rand(1)*y_max)]; %2*pi*rand-pi];
   % plot(q_rand(1), q_rand(2), 'x', 'Color',  [0 0.4470 0.7410])        
    
    % On cherche le plus proche voisin de q_rand dans le graphe
    ndist = [];
    for j = 1:1:length(nodesi)
        n = nodesi(j);
        tmp = dist(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodesi(idx); 
    
    [q_new.coord ~] = Extend(q_rand, q_near.coord,val, eps);
    
    if noCollision(q_near.coord, q_new.coord, obstacle) && ...
            noCollision(q_near.coord, q_new.coord, obstacle2) 
        line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'color','k');
        hold on
        q_new.cost = dist(q_new.coord, q_near.coord) + q_near.cost;
        q_min = q_near;
        C_min = q_new.cost;
        
        % Update parent to least cost-from node
        for j = 1:1:length(nodesi)
            if nodesi(j).coord == q_min.coord
                q_new.parent = j;
            end
        end
        
        % Append to nodes
        nodesi = [nodesi q_new];
        
    end
    ndist1 = [];
    for j = 1:1:length(nodesf)
        n = nodesf(j);
        tmp = dist(n.coord, q_new.coord);
        ndist1 = [ndist1 tmp];
    end
    [val1, idx1] = min(ndist1);
    q_nearf = nodesf(idx1);
    
    D=[];
    for n1=1:length(nodesi)
            for n2=1:length(nodesf)
               tmpdist = dist(nodesi(n1).coord,nodesf(n2).coord);
               D(n1,n2)=tmpdist;
            end
    end
     %[~, idx2] = min(min(D,[],1));
     [~, idx1] = min(min(D,[],2));
     q_new1=nodesi(idx1);
        
    [q_newf.coord ~] = Extend1(q_new1.coord, q_nearf.coord, 2*val1,eps);
    if noCollision(q_nearf.coord, q_newf.coord, obstacle) &&...
            noCollision(q_nearf.coord, q_newf.coord, obstacle2)
        line([q_nearf.coord(1), q_newf.coord(1)], [q_nearf.coord(2), q_newf.coord(2)], 'color','g');
       drawnow
        hold on
        q_newf.cost = dist(q_newf.coord, q_nearf.coord) + q_nearf.cost;
        q_minf = q_nearf;
        C_minf = q_newf.cost;
        
        for j = 1:1:length(nodesf)
            if nodesf(j).coord == q_minf.coord
                q_newf.son= j;
            end
        end
        nodesf = [nodesf q_newf];
    else
        
    end
    q_goal=q_newf;
    count=count+1;
end

D=[];
for n1=1:length(nodesi)
        for n2=1:length(nodesf)
           tmpdist = dist(nodesi(n1).coord,nodesf(n2).coord);
           D(n1,n2)=tmpdist;
        end
end
 [~, idx2] = min(min(D,[],1));
 [~, idx1] = min(min(D,[],2));
 q_goal=nodesi(idx1);
 scatter(nodesf(idx2).coord(1),nodesf(idx2).coord(2),'filled')

D = [];
for j = 1:1:length(nodesi)
    tmpdist = dist(nodesi(j).coord, q_goal.coord);
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
[val, idx] = min(D);
q_goal.parent = idx;
q_end = q_goal;
nodesi= [nodesi q_goal];
while q_end.parent ~= 0
    start = q_end.parent;
    line([q_end.coord(1), nodesi(start).coord(1)], [q_end.coord(2), nodesi(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
    hold on
    q_end = nodesi(start);
end

q_end = nodesf(idx2);
while q_end.son ~= 0
    start = q_end.son;
    line([q_end.coord(1), nodesf(start).coord(1)], [q_end.coord(2), nodesf(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
    hold on
    q_end = nodesf(start);
end

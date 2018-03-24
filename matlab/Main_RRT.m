clear all
close all
%% 

M=50;
eps=2;
x_max = 100;
y_max = 100;
obstacle = [70,20,5,60];
obstacle2 =[10,40,50,20];
EPS = 20;
numNodes = 3000;        

q_start.coord = [0 99];
scatter(q_start.coord(1),q_start.coord(2));
q_start.cost = 0;
q_start.parent = 0;
q_goal.coord = [75 10];
scatter(q_goal.coord(1),q_goal.coord(2));
q_goal.cost = 0;

commandes=[];

nodes(1) = q_start;

figure(1)
axis equal
axis([0 x_max 0 y_max])
rectangle('Position',obstacle,'FaceColor',[0 .5 .5])
rectangle('Position',obstacle2,'FaceColor',[0 .5 .5])
hold on

for i = 1:1:numNodes
    q_rand = [floor(rand(1)*x_max) floor(rand(1)*y_max)]; %2*pi*rand-pi];
    for j = 1:1:length(nodes)
        if nodes(j).coord == q_goal.coord
            break
        end
    end
    
    % On cherche le plus proche voisin dans le graphe d'où part pour se
    % rapprocher de q_rand
    ndist = [];
    for j = 1:1:length(nodes)
        n = nodes(j);
        tmp = dist(n.coord, q_rand);
        ndist = [ndist tmp];
    end
    [val, idx] = min(ndist);
    q_near = nodes(idx); % Plus proche voisin dans le graphe
    
    % On intègre ensuite le système d'état en choisissant des commandes de
    % sorte à se rapprocher de q_rand. On appelle q_new le nouvel état
    % qu'on atteint après intégration.
    [q_new.coord] = Extend(q_rand, q_near.coord,val, eps);
       
    if noCollision(q_rand, q_new.coord, obstacle) && noCollision(q_rand, q_new.coord, obstacle2)
        line([q_near.coord(1), q_new.coord(1)], [q_near.coord(2), q_new.coord(2)], 'color','k');
        hold on
        q_new.cost = dist(q_new.coord, q_near.coord) + q_near.cost;
        
        % On cherche tous les noeud existant dans un rayon r
        q_nearest = [];
        r = 5;
        neighbor_count = 1;
        for j = 1:1:length(nodes)
            if noCollision(nodes(j).coord, q_new.coord, obstacle) && noCollision(q_rand, q_new.coord, obstacle2) && dist(nodes(j).coord, q_new.coord) <= r
                q_nearest(neighbor_count).coord = nodes(j).coord;
                q_nearest(neighbor_count).cost = nodes(j).cost;
                neighbor_count = neighbor_count+1;
            end
        end
        
        % Initialisation du coût
        q_min = q_near;
        C_min = q_new.cost;
        
        % Itération sur tous les voisins dans la boule de rayon jusqu'à
        % trouver le noeud qui a le moindre coût
        
        for k = 1:1:length(q_nearest)
            if noCollision(q_nearest(k).coord, q_new.coord, obstacle) && noCollision(q_rand, q_new.coord, obstacle2) && q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord) < C_min
                q_min = q_nearest(k);
                C_min = q_nearest(k).cost + dist(q_nearest(k).coord, q_new.coord);          
            end
        end
        
        % On modifie les "parent du noeud q_new
        for j = 1:1:length(nodes)
            if nodes(j).coord == q_min.coord
                q_new.parent = j;
            end
        end
        
        % On ajoute q_new dans le graphe, puis on recommence avec un
        % nouveau point tirer au hasard dans l'espace d'état
        nodes = [nodes q_new];
    end
end

D = [];
for j = 1:1:length(nodes)
    tmpdist = dist(nodes(j).coord, q_goal.coord);
    D = [D tmpdist];
end

% Search backwards from goal to start to find the optimal least cost path
[val, idx] = min(D);
q_final = nodes(idx);
q_goal.parent = idx;
q_end = q_goal;
nodes = [nodes q_goal];
while q_end.parent ~= 0
    start = q_end.parent;
    line([q_end.coord(1), nodes(start).coord(1)], [q_end.coord(2), nodes(start).coord(2)], 'Color', 'r', 'LineWidth', 2);
    hold on
    q_end = nodes(start);
end
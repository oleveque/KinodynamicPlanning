function [q_new commandes]=Extend1(q_rand,q_near,M,eps)

sinu2=(q_rand(2)-q_near(2))/M;
cosu2=(q_rand(1)-q_near(1))/M;

t=0.5;
    if eps<M
       q_new(1) = q_near(1) + t*cosu2;
       q_new(2) = q_near(2) + t*sinu2;
       theta = acos(cosu2);
    else
        q_new=q_near;
        theta = acos(cosu2);
    end
    commandes=[1 acos(cosu2)];
end
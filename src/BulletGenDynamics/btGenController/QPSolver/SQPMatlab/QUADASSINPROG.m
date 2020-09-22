function [xx,ffval,eexitflag] = QUADASSINPROG(H, f, A, b, Aeq, beq, num_iters)
    oopt = optimoptions('quadprog','MaxIter', num_iters);
    [xx,ffval,eexitflag,~,~] = quadprog(H, f, A, b,Aeq, beq,[],[],[],[]);
end
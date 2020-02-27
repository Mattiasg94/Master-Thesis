function [Ju, Jp, Jeps] = costs_23(us, epss, dt)
    Ju = norm(us, 1)*dt;
    Jp = norm(us, inf);
    Jeps = norm(epss, 1)*dt;
end
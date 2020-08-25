function dydt = sixDOF(v)
    y = v(1);
    u = v(2);
    
    ydot = u;
    udot = -4*y-2*u;
        
dydt = [ydot; udot];
end
function yf = rk4(t,y,uu,h)
% addpath('../1_dependencies/parameters/');  
    NUM_STATEVARS = 12;

    hh = h/2;
	h6 = h/6;
	
	dydt = sixDOF(y,uu);
    
    yt = [0;0;0;0;0;0;0;0;0;0;0;0];
	
    for i = (1:NUM_STATEVARS)
        yt(i) = y(i) + (hh*dydt(i));
    end
	
	th = t+hh;
	
    dyt = sixDOF(yt,uu);
	
    for i=1:NUM_STATEVARS
		yt(i) = y(i) + (hh*dyt(i));
    end

	dym = sixDOF(yt,uu);
	
    for i=1:NUM_STATEVARS
		yt(i) = y(i)+h*dym(i);
    end
    
    for i=1:NUM_STATEVARS
		dym(i) = dym(i) + dyt(i);
    end
	
    dyt = sixDOF(yt,uu);
	
    for i=1:NUM_STATEVARS
		y(i)=y(i)+h6*(dydt(i) + dyt(i) + (2.0*dym(i)));
    end

    yf = y;
  end
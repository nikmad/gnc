function yf = rk4(y,uu,h,scraft)

    num_states = 9;

	yt = [0;0;0;0;0;0;0;0;0];

    %_______________________________________________________

    K1 = threeDOF(y,uu,scraft);
    
    for i = (1:num_states)
        yt(i) = y(i) + h/2 * K1(i);
    end
    
    %_______________________________________________________

    K2 = threeDOF(yt,uu,scraft);
	
    for i=1:num_states
		yt(i) = y(i) + h/2 * K2(i);
    end
    
    %_______________________________________________________

    K3 = threeDOF(yt,uu,scraft);
	
    for i=1:num_states
		yt(i) = y(i) + h * K3(i);
    end
    
    %_______________________________________________________

    K4 = threeDOF(yt,uu,scraft);
    %_______________________________________________________
    	
    for i=1:num_states
        y(i) = y(i) + h/6*( K1(i) + 2.0*( K2(i) + K3(i) ) + K4(i) );
    end

    yf = y;
  end
%     num_states = 2;
%     y = [1, 1];
%     yt = [0;0];
%     h = 0.1;
    num_states = 12;
    y = [0,0,0, 0,0,0, 0,0,0, 0,0,0];
    yt = [0;0];
    h = 0.1;

for idx = 0.1:h:0.2
    %_______________________________________________________
    % K1: following evaluation gives the conventional textbook RK4 term K3
    % - nkm
    K1 = sixDOF(y);
    
    for i = (1:num_states)
        yt(i) = y(i) + h/2 * K1(i);
    end
    
    %_______________________________________________________
    % K2: following evaluation gives the conventional textbook RK4 term K2
    % - nkm
    K2 = sixDOF(yt);
	
    for i=1:num_states
		yt(i) = y(i) + h/2 * K2(i);
    end
    
    %_______________________________________________________
    % K3: following evaluation gives the conventional textbook RK4 term K3
    % - nkm
    K3 = sixDOF(yt);
	
    for i=1:num_states
		yt(i) = y(i) + h * K3(i);
    end
    
    %_______________________________________________________
    % K4: following evaluation gives the conventional textbook RK4 term K4
    % - nkm
    K4 = sixDOF(yt);
    %_______________________________________________________
    	
    for i=1:num_states
		% The following line gives: u_j+1 = u_j + 1/6*(K1 + 2*(K2+K3) + K4)
		% - nkm
        y(i) = y(i) + (h/6)*( K1(i) + 2.0*( K2(i) + K3(i) ) + K4(i) )
    end

end
function y = rk4(plane_states,t,h, acc_real_plant)

	hh=h/2;
	h6=h/6;
	
	derivative(y,t,dydt,acc_real_plant);	
	for(i=0;i<NUM_STATEVARS;i++)
	{
		yt[i] = y[i]+ (hh*dydt[i]);
	}

	th = t+hh;
	derivative(yt,th,dyt,acc_real_plant);
	for(i=0;i<NUM_STATEVARS;i++)
	{
		yt[i] = y[i] + (hh*dyt[i]);
	}

	derivative(yt,th,dym,acc_real_plant);
	for(i=0;i<NUM_STATEVARS;i++)
	{
		yt[i] = y[i]+h*dym[i];
	}
	for(i=0;i<NUM_STATEVARS;i++)
	{
		dym[i] = dym[i] + dyt[i];
	}
	derivative(yt,t+h,dyt,acc_real_plant);
	for(i=0;i<NUM_STATEVARS;i++)
	{
		y[i]=y[i]+h6*(dydt[i] +dyt[i] + (2.0*dym[i]));
	}
  end
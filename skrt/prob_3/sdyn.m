function yf = sdyn(yi,uu,scraft)

h = 1;

yf = rk4(yi,uu,h,scraft);

end


function [y] = sensitive90zone(theta)
%SENSITIVE90ZONE Summary of this function goes here
%   Detailed explanation goes here

y = theta;

%____________________________________________________________
	% nkm - "Around 2.8648 deg (in rad 0.05) below and above 90 deg
	% has to be discarded for sec() and tan() functions because
	% in this zone, increase in sec(theta) and tan(theta) for a
	% small rise in theta is huge. So we discard the zone (90-2.86)
	% to (90+2.86) i.e., 87.14 to 92.86 deg".

	sensitive90zone = 0.05; 
	
%____________________________________________________________

theta1 = mod(abs(theta),2*pi);
theta2 = floor(abs(theta)/(2*pi))*(2*pi);

if     ((floor(theta1/(pi/2))==0) &&   (pi/2-theta1 < sensitive90zone))
	y = sign(theta)*(pi/2 - sensitive90zone + theta2);

elseif ((floor(theta1/(pi/2))==1) &&   (theta1-pi/2 < sensitive90zone))
	y = sign(theta)*(pi/2 + sensitive90zone + theta2);

elseif ((floor(theta1/(pi/2))==2) && (3*pi/2-theta1 < sensitive90zone))
	y = sign(theta)*(3*pi/2 - sensitive90zone + theta2);

elseif ((floor(theta1/(pi/2))==3) && (theta1-3*pi/2 < sensitive90zone))
	y = sign(theta)*(3*pi/2 + sensitive90zone + theta2);
	
end

end


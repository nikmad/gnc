function dubinspath = dubinsParameters(start_node, end_node, R)

  ell = norm(start_node(1:2)-end_node(1:2));
  if ell<2*R,
      disp('The distance between nodes must be larger than 2R.');
      dubinspath = [];
  else
    
    ps   = start_node(1:3)';
    chis = start_node(4);
    pe   = end_node(1:3)';
    chie = end_node(4);
    
    crs = ps + R*rotz(pi/2)*[cos(chis),sin(chis),0]';
    cls = ps + R*rotz(-pi/2)*[cos(chis),sin(chis),0]';
    cre = pe + R*rotz(pi/2)*[cos(chie),sin(chie),0]';
    cle = pe + R*rotz(-pi/2)*[cos(chie),sin(chie),0]';
    
   
    % compute L1
    theta = mo(atan2(cre(2)-crs(2), cre(1)-crs(1))+2*pi);
    L1 = norm(crs-cre) + R*mo(2*pi + mo(theta-pi/2) - mo(chis-pi/2))...
        +R*mo(2*pi+mo(chie-pi/2) - mo(theta-pi/2));
    % compute L2
    ell = norm(cle-crs);
    theta = mo(atan2(cle(2)-crs(2),cle(1)-crs(1))+2*pi);
    theta2 = theta-pi/2 +asin(2*R/ell);
    if isreal(theta2)==0, 
      L2 = 9999; 
    else
      L2 = sqrt(ell^2 - 4*R^2) + R*mo(2*pi+mo(theta2) - mo(chis-pi/2))...
          +R*mo(2*pi + mo(theta2+pi) - mo(chie+pi/2));
    end
    % compute L3
    ell = norm(cre-cls);
    theta = mo(atan2(cls(2)-cre(2), cls(1)-cre(1))+2*pi);
    theta2 = acos(2*R/ell);
    if isreal(theta2)==0,
      L3 = 9999;
    else
      L3 = sqrt(ell^2 - 4*R^2) + R*mo(2*pi + mo(chis+pi/2) - mo(theta+theta2))...
          +R*(2*pi + mo(chie-pi/2) - mo(theta+theta2-pi));
    end
    % compute L4
    theta = mo(atan2(cls(2)-cle(2),cls(1)-cle(1))+2*pi);
    L4 = norm(cls-cle) + R*mo(2*pi+mo(chis+pi/2) - mo(theta+pi/2))...
        +R*mo(2*pi + mo(theta+pi/2) - mo(chie+pi/2));
    % L is the minimum distance
    [L,idx] = min([L1,L2,L3,L4]);
    e1 = [1; 0; 0];
    switch(idx),
        case 1,
            cs = crs;
            lams = 1;
            ce = cre;
            lame = 1;
            q1 = (ce-cs)/norm(ce-cs);
            w1 = cs + R*rotz(-pi/2)*q1;
            w2 = ce + R*rotz(-pi/2)*q1;
        case 2,   
            cs = crs;
            lams = 1;
            ce = cle;
            lame = -1;
            ell = norm(ce-cs);
            theta = atan2(ce(2)-cs(2), ce(1)-cs(1)+2*pi);
            theta2 = theta - pi/2 + asin(2*R/ell);
            q1 = rotz(theta2+pi/2)*e1;
            w1 = cs + R*rotz(theta2)*e1;
            w2 = ce + R*rotz(theta2+pi)*e1;
        case 3,
            cs = cls;
            lams = -1;
            ce = cre;
            lame = 1;
            ell = norm(ce-cs);
            theta = atan2(ce(2)-cs(2),ce(1)-cs(1)+2*pi);
            theta2 = acos(2*R/ell);
            q1 = rotz(theta+theta2-pi/2)*e1;
            w1 = cs + R*rotz(theta+theta2)*e1;
            w2 = ce + R*rotz(theta+theta2-pi)*e1;
         case 4,
            cs = cls;
            lams = -1;
            ce = cle;
            lame = -1;
            q1 = (ce-cs)/norm(ce-cs);
            w1 = cs + R*rotz(pi/2)*q1;
            w2 = ce + R*rotz(pi/2)*q1;
    end
    w3 = pe';
    q3 = rotz(chie)*e1;
    
    % assign path variables
    dubinspath.ps   = ps;
    dubinspath.chis = chis;
    dubinspath.pe   = pe;
    dubinspath.chie = chie;
    dubinspath.R    = R;
    dubinspath.L    = L;
    dubinspath.cs   = cs;
    dubinspath.lams = lams;
    dubinspath.ce   = ce;
    dubinspath.lame = lame;
    dubinspath.w1   = w1;
    dubinspath.q1   = q1;
    dubinspath.w2   = w2;
    dubinspath.w3   = w3';
    dubinspath.q3   = q3;
  end
end

% rotz(theta)
%   rotation matrix about the z axis.
function R = rotz(theta)
    R = [...
        cos(theta), -sin(theta), 0;...
        sin(theta), cos(theta), 0;...
        0, 0, 1;...
        ];
end

function out = mo(angle)
    out = mod(angle,2*pi);
end
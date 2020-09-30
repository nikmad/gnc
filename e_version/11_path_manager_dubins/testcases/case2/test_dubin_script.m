pplanner = path_planner;
% path_manager(pplanner; 5; 5; 5; 30; 0.5; 0.1; 0.2; 0.2; 0.2; 0.1; 1; 0.4; 20; 3; 2; 0.4; 3);
inp = [pplanner; 5; 5; 5; 30; 0.5; 0.1; 0.2; 0.2; 0.2; 0.1; 1; 0.4; 20; 3; 2; 0.4; 3];
pmanager = path_manager(inp);
pin = [pmanager; 3];
path_follow(pin)




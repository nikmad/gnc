
function map = createWorld(city_width, building_height, num_blocks, street_width)

    map.width = city_width;          % the city is of size (width)x(width)
    map.MaxHeight = building_height; % maximum height of buildings
    map.NumBlocks = num_blocks;      % number of blocks in city
    map.StreetWidth = street_width;  % percent of block that is street.
    
    % computed parameters
    map.BuildingWidth = map.width/map.NumBlocks*(1-map.StreetWidth);
    map.StreetWidth   = map.width/map.NumBlocks*map.StreetWidth;
    map.heights = map.MaxHeight*rand(map.NumBlocks,map.NumBlocks);
    for i=1:map.NumBlocks,
       map.buildings_n(i) = [...
                0.5*map.width/map.NumBlocks*(2*(i-1)+1),...
                ];
    end
    map.buildings_e = map.buildings_n;
    
end
    

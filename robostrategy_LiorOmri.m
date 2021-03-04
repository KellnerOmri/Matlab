function [move,mem] = robostrategy_LiorOmri(env,mem)
%Strategy for robot tournament game, following opponent
%
%Environment Struct
% field:
% info,  STRUCT{team, fuel, myPos, oppPos}
% basic, STRUCT{walls, rRbt, rMF, lmax} //radius of the Robot, radius of
% the fuel and mine, lmax =  max movement
% mines, STRUCT{nMine, mPos, mineScr, mineExist} // mineScr = size 
% fuels, STRUCT{nFuel, fPos, fuelScr, fuelExist}
%
%Memory Struct
% field:
% path, STRUCT{start, dest, pathpt, nPt, proc, lv}

% Stop incase there is no more fuel on the map to preserve remaining fuel.
if (env.fuels.fExist(:) == 0) 
    move = [0, 0];
    return
end

% Attack opponent incase he close and there is a points difference.
if ptdist(env.info.myPos, env.info.opPos) < 1.8 &&...
        env.info.fuel > env.info.fuel_op + 55
    % Get the angle to attack the opponent
    angleOp = abs(atan((env.info.myPos(2) - env.info.opPos(2))/...
                 (env.info.myPos(1) - env.info.opPos(1))));
             
    if env.info.opPos(1) > env.info.myPos(1)
        if env.info.opPos(2) < env.info.myPos(2) % square 3
            angleOp = -angleOp;
        end
    else
        if env.info.opPos(2) > env.info.myPos(2) % square 2
            angleOp = pi - angleOp;
        
        else % square 4
            angleOp = pi + angleOp;
        end
    end
    
    % Set coordinates towards the opponent
    x = env.basic.lmax*cos(angleOp);
    y = env.basic.lmax*sin(angleOp);
    
    move = [x, y];
    return
end

minimumFuelDistance = 9999;
indexFuel = 1;
% Find closest fuel by distance and save its index
for i = 1:env.fuels.nFuel
    distance = ptdist(env.info.myPos, env.fuels.fPos(i, :));
    
    % Check if the fuel exists and has a lower distance
    if minimumFuelDistance > distance && env.fuels.fExist(i)
        indexFuel = i;
        minimumFuelDistance = distance;
    end    
end

% Get the angle of the closest fuel
angleF = abs(atan((env.info.myPos(2) - env.fuels.fPos(indexFuel, 2))/...
                 (env.info.myPos(1) - env.fuels.fPos(indexFuel, 1))));

if env.fuels.fPos(indexFuel, 1) > env.info.myPos(1)
    if env.fuels.fPos(indexFuel, 2) < env.info.myPos(2) % square 3
        angleF = -angleF;
    end
else
    if env.fuels.fPos(indexFuel, 2) > env.info.myPos(2) % square 2
        angleF = pi - angleF;
        
    else % square 4
        angleF = pi + angleF;
    end
end

% Set coordinates towards the fuel
x = env.basic.lmax*cos(angleF);
y = env.basic.lmax*sin(angleF);

minimumMineDistance = 9999;
indexMine = 1;

% Get next turns position according to the movement towards the fuel
newPos(1) = env.info.myPos(1) + x;
newPos(2) = env.info.myPos(2) + y;

% Get closest mine
for i = 1:env.mines.nMine
    distance = ptdist(newPos, env.mines.mPos(i, :));
    
    % Check if the mine exists and has a lower distance
    if minimumMineDistance > distance && env.mines.mExist(i)
        indexMine = i;
        minimumMineDistance = distance;
    end    
end 

% Dodge mine if it is in a certain range.
if minimumMineDistance <=  env.basic.rRbt + env.basic.rMF - 0.2
    % Get the angle of the mine
     angleM = abs(atan((newPos(2) - env.mines.mPos(indexMine, 2))/...
                       (newPos(1) - env.mines.mPos(indexMine, 1))));
     
    if env.mines.mPos(indexMine, 1) > newPos(1)
        if env.mines.mPos(indexMine, 2) < newPos(2) % square 3
            angleM = -angleM;
        end
    else
        if env.mines.mPos(indexMine, 2) > newPos(2) % square 2
            angleM = pi - angleM; 
        else % square 4
            angleM = pi + angleM;
        end
    end
    
    if angleF < 0
        angleF = angleF + 2*pi;
    end
    
    if angleM < 0
        angleM = angleM + 2*pi;
    end
    
    % Set coordinates to go above or under mine according to the angle of the fuel
    if angleF <= angleM
        x = env.basic.lmax*cos(angleF - pi/3);
        y = env.basic.lmax*sin(angleF - pi/3);
    else
        x = env.basic.lmax*cos(angleF + pi/3);
        y = env.basic.lmax*sin(angleF + pi/3);
    end
    
    move = [x, y];
    return
end

% Dodge opponent incase they have more points and are close
if ptdist(env.info.myPos, env.info.opPos) < 1 &&...
        env.info.fuel < env.info.fuel_op
    % Get the angle of the opponent
    angleOp = abs(atan((env.info.myPos(2) - env.info.opPos(2))/...
                 (env.info.myPos(1) - env.info.opPos(1))));
             
    if env.info.opPos(1) > env.info.myPos(1)
        if env.info.opPos(2) < env.info.myPos(2) % square 3
            angleOp = -angleOp;
        end
    else
        if env.info.opPos(2) > env.info.myPos(2) % square 2
            angleOp = pi - angleOp;
        
        else % square 4
            angleOp = pi + angleOp;
        end
    end
    
    % Set coordinates to the opposite direction of the opponent
    x = env.basic.lmax*cos(angleOp + pi);
    y = env.basic.lmax*sin(angleOp + pi);
end

% Incase the opponent is very close and our robot has any advantage in
% points go towards the opponent
if ptdist(env.info.myPos, env.info.opPos) < 1 &&...
        env.info.fuel > env.info.fuel_op
    % Get the angle of the opponent
    angleOp = abs(atan((env.info.myPos(2) - env.info.opPos(2))/...
                 (env.info.myPos(1) - env.info.opPos(1))));
             
    if env.info.opPos(1) > env.info.myPos(1)
        if env.info.opPos(2) < env.info.myPos(2) % square 3
            angleOp = -angleOp;
        end
    else
        if env.info.opPos(2) > env.info.myPos(2) % square 2
            angleOp = pi - angleOp;
        
        else % square 4
            angleOp = pi + angleOp;
        end
    end
    
    % Set coordinates towards the opponent
    x = env.basic.lmax*cos(angleOp);
    y = env.basic.lmax*sin(angleOp);
end
    
move = [x, y];

function d = ptdist(A,B)
        AB = B-A;
        d = sqrt(AB*AB');
end

end
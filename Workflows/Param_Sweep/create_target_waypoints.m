n = 10;
[x, y, z] = sphere(n);
sp_x = x;
sp_y = y;
sp_z = z+5;
target_waypoint = zeros(3,n^2);
for i=1:n+1
    for j=1:n+1
        target_waypoint(1,(n)*i + k) = sp_x(i,j);
        target_waypoint(2,(n)*i + k) = sp_y(i,j);
        target_waypoint(3,(n)*i + k) = sp_z(i,j);
    end
end
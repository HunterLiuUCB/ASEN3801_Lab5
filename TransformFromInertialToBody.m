%% TransformFromInertialToBody.m
function wind_body = TransformFromInertialToBody(wind_inertial, aircraft_eulerangles)
phi = aircraft_eulerangles(1);
theta = aircraft_eulerangles(2);
psi = aircraft_eulerangles(3);
R_EB = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
     cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
     -sin(theta),          sin(phi)*cos(theta),                             cos(phi)*cos(theta)]';
wind_body = R_EB * wind_inertial;
end
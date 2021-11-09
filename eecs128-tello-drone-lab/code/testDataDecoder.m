function data = testDataDecoder(file)

% index                        1
% time                         2
% reference input              3
% control left/right           4
% control forward/backward     5
% control up/down              6
% control yaw                  7
% pitch                        8
% roll                         9
% yaw                         10
% vgx                         11
% vgy                         12
% vgz                         13
% templ                       14
% temph                       15
% tof                         16
% h                           17
% bat                         18
% baro                        19
% time                        20
% agx                         21
% agy                         22
% agz                         23

values = csvread(file,3,0);

data.time                = values(:,2);
data.reference           = values(:,3);
data.conrtol_leftRight   = values(:,4);
data.control_forwardBack = values(:,5);
data.control_upDown      = values(:,6);
data.control_yaw         = values(:,7);
data.pitch               = values(:,8);
data.roll                = values(:,9);
data.yaw                 = values(:,10);
data.velocity_x          = values(:,11);
data.velocity_y          = values(:,12);
data.velocity_z          = values(:,13);
data.temperature_low     = values(:,14);
data.temperature_high    = values(:,15);
data.timeOfFlight        = values(:,16);
data.height              = values(:,17);
data.battery             = values(:,18);
data.barometer           = values(:,19);
data.acceleration_x      = values(:,21);
data.acceleration_y      = values(:,22);
data.acceleration_z      = values(:,23);

end
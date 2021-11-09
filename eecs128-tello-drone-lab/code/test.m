data1 = testDataDecoder("data/test_1.0.txt");

time1 = data1.time;
ip1 = data1.control_yaw;
yaw1 = data1.yaw;
fix1 = zeros(size(time1));
fix1(find(yaw1==
close all;
clear;
clc;

tb = TestClass2;
rtt = TestClass1(tb);
%% 
close all;
clear;
clc;

id = 2; % Second controller connected to device - (PS3 controller is 1 so emulated XBox controller is 2)
joy = vrjoystick(id);

tb = TestClass2;
rtt = TestClass1(tb);

avgIndex = 1;
sum = 0;
average = 0;
iteration = 5;
while (1)
    [axes, buttons, povs] = read(joy);
    
   
    if (avgIndex >= iteration)
        average = sum / 5;
        avgIndex = 0;
        sum = 0;
    else
        sum = sum + axes(1);
        avgIndex = avgIndex + 1;
    end
    
    tb.OnStateChange(average);
end
% clear all;
% clc;
% nikpi = raspi;
% configurePin(nikpi, 18, 'PWM');
% writePWMDutyCycle(nikpi, 18, 1);
% writePWMFrequency(nikpi, 18, 2000);

configurePin(nikpi, 18, 'PWM');
writePWMVoltage(nikpi, 18, 3);
% writePWMFrequency(nikpi, 18, 2000);
% for i=10:0.05:0.9
%     i = i*5;
    writePWMFrequency(nikpi, 18, 4000);
%     writePWMDutyCycle(nikpi, 18, .5);

    pause(1);
% end

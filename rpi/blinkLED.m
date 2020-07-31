% function blinkLED()
% nikrasp = raspi;
% configurePin(nikpi, 18, 'PWM');
% writePWMFrequency(nikpi, 18, 500);
% % for i=0:100
%     writePWMDutyCycle(nikpi, 18, 5*i);
for i=1:100
    disp(i);
    writeLED(nikpi,"LED0",1);
    pause(0.01);
        writeLED(nikpi,"LED0",0);
    pause(0.5);
%     pause(0.05);


% end

end
% end


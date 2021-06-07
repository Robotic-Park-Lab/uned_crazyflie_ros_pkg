function PIDx = PID_c2d(PIDs,T)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here
PIDx = [ PIDs(1)+PIDs(2)*T/2+PIDs(3)/T;
        -PIDs(1)+PIDs(2)*T/2-2*PIDs(3)/T;
         PIDs(3)/T];
end


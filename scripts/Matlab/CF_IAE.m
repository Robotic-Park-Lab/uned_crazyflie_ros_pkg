function [IAE, ITAE] = CF_IAE(data, time)

IAE = 0;
ITAE = 0;
for i=2:length(data)
   aux = abs((data(i)-data(i-1))*(time(i)-time(i-1))/2);
   IAE = IAE + aux;
   ITAE = ITAE + aux*time(i);
end

end


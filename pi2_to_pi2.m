function angle = pi2_to_pi2(angle)
if(abs(angle)>pi/2)
    if(angle>0)
        angle=angle-pi;
    else
        angle=angle+pi;
    end
end

function CD = drag(Drag, interp_type, time, altitude, speed)

if contains(interp_type,'time')
    ft_drag = polyfit(Drag(:,1),Drag(:,4),6);
    CD = polyval(ft_drag,time);
elseif contains(interp_type,'altitude')
    ft_drag = polyfit(Drag(:,2),Drag(:,4),6);
    CD = polyval(ft_drag,altitude);
elseif contains(interp_type,'speed')
    ft_drag = polyfit(Drag(:,3),Drag(:,4),6);
    CD = polyval(ft_drag,speed);
end

end
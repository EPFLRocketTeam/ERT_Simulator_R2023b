function diameter_ch(diameters_v,stage_z_v,rocket_name)
    % %INPUT: array of diameters
    %         array of distances for these diameters
    %This method changes the rocket definition file for diameter changes values
    %and distance values given as input
    input=readlines(rocket_name,"EmptyLineRule","skip");
    replace1=['diameters ',' '];
    for i=1:size(diameters_v,2)
        replace1= [replace1 , num2str(diameters_v(i)) ,' ' ];
    end
    replace2=['stage_z ',' '];
    for i=1:size(stage_z_v,2)
        replace2= [replace2 , num2str(stage_z_v(i)) ,' ' ];
    end
    
    input(2,1)=replace1;
    input(3,1)= replace2;
    fid= fopen(rocket_name, 'w+');
    fwrite(fid, strjoin(input, '\n'));
    fclose(fid);

end

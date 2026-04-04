function [azimuth_BS,range_tgt] = get_target_position(time_s,struc_trajectory)
   
       % the target position at given time instance "time"
       target_pos = struc_trajectory.radar2_pos + (struc_trajectory.relative_velocity_vector*time_s);
       % Range of the target
       range_tgt = norm(target_pos-struc_trajectory.radar1_pos);  
       % cos of angle between BS and MU
       cos_thetat = target_pos(1,2)/range_tgt;
       % The angle between BS and MU
       azimuth_BS = acosd(cos_thetat); 
  
end
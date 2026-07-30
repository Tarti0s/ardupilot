#include "Copter.h"

bool ModeStep::init(bool ignore_checks)
{
    can_receive_cmd_xy = false;
    can_receive_cmd_z = false; 
    received_cmd_xy = false;
    received_cmd_z = false;

    move_x = 0.0f;
    move_y = 0.0f;
    move_z = 0.0f;

    xy = 0;
    z = 0;

    move_start_ms = 0;

    //initialise la distance du pas et le temps d'attente avant une nouvelle commande
    step_m = g2.user_parameters.get_step_dist();
    waiting_time_ms = g2.user_parameters.get_waiting_time();

    // initialise les positions 
    current_loc_vec = pos_control->get_pos_estimate_NED_m();
    target_loc_vec.zero();
    stop_loc_vec = pos_control->get_pos_estimate_NED_m();

    // initialise horizontal speed, acceleration
    pos_control->NE_set_max_speed_accel_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());
    pos_control->NE_set_correction_speed_accel_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());

    // initialize vertical speeds and acceleration
    pos_control->D_set_max_speed_accel_cm(wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_D_cmss());
    pos_control->D_set_correction_speed_accel_cm(wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_D_cmss());

    // initialise velocity controller
    pos_control->D_init_controller();
    pos_control->NE_init_controller();

    // initialise yaw
    auto_yaw.set_mode_to_default(false);

    return true;
}

void ModeStep::run()
{
    
    current_loc_vec = pos_control->get_pos_estimate_NED_m();
    update_simple_mode();

    //vérifie si les valeurs n'ont pas été changé par l'utilisateur
    step_m = g2.user_parameters.get_step_dist();
    waiting_time_ms = g2.user_parameters.get_waiting_time();
    
    //set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    //Appelle différentes fonctions selon le mode du drone
    switch(Step_state)
    {
        case SubMode::Waiting : {waiting();break;}
        case SubMode::Moving_xy : {moving_xy();break;}
        case SubMode::Moving_z : {moving_z();break;}
    }
    //Change le mode du drone
    if(received_cmd_xy){Step_state = SubMode::Moving_xy;}
    else if(received_cmd_z){Step_state = SubMode::Moving_z;}
    else{Step_state = SubMode::Waiting;}
}

float ModeStep::throttle_norm_input_dz() const //centre le joystick de gauche (par défaut reste en position basse)
{
    //récupère les informations de la RC
    const int16_t radio_min = channel_throttle->get_radio_min();
    const int16_t radio_max = channel_throttle->get_radio_max();
    const int16_t radio_in = channel_throttle->get_radio_in();
    const int16_t dead_zone = channel_throttle->get_dead_zone();
    const int16_t mid = (radio_min + radio_max) / 2;
    const int16_t dz_min = mid - dead_zone;
    const int16_t dz_max = mid + dead_zone;
    const int16_t reverse_mul = channel_throttle->get_reverse() ? -1 : 1;

    float throttle_norm;
    //Calcule et regarde l'état du joystick de gauche par rapport à son centre et retourne une valeur entre -1 et 1
    if (radio_in < dz_min && dz_min > radio_min) {
        throttle_norm = reverse_mul * (float)(radio_in - dz_min) / (float)(dz_min - radio_min);
    } else if (radio_in > dz_max && radio_max > dz_max) {
        throttle_norm = reverse_mul * (float)(radio_in - dz_max) / (float)(radio_max - dz_max);
    } else {
        throttle_norm = 0;
    }
    return constrain_float(throttle_norm, -1.0f, 1.0f);
}

void ModeStep::waiting()
{
    //enregistre l'état des joystics de la RC
    float pilot_roll = channel_roll->norm_input_dz(); //joystick gauche/droite
    float pilot_pitch = channel_pitch->norm_input_dz(); //joystick avant/arrière
    float pilot_throttle = throttle_norm_input_dz(); //joystick haut/bas

    moving();

    if(can_receive_cmd_xy)//si le drone est prêt à recevoir une commande de déplacement sur le plan horizontal
    {
        if(fabsf(pilot_roll) >= 0.5f || fabsf(pilot_pitch) >= 0.5f)//on vérifie le joystick
        {
            float forward = 0.0f;
            float right = 0.0f;
            float yaw = ahrs.get_yaw();

            if(pilot_roll > 0.25){right = step_m;} //droite
            else if(pilot_roll < -0.25){right = -step_m;} //gauche

            if(pilot_pitch > 0.25){forward = -step_m;} //avant
            else if(pilot_pitch < -0.25){forward = +step_m;} //arrière

            //Calcule la direction dans laquelle le drone doit aller selon son orientation
            move_x = forward * cosf(yaw) - right * sinf(yaw); 
            move_y = forward * sinf(yaw) + right * cosf(yaw);
            
            //fait passer le drone en sous-mode "moving_xy" et empêche une nouvelle commande
            received_cmd_xy = true;
            can_receive_cmd_xy = false;
            can_receive_cmd_z = false;
            start_loc_vec = current_loc_vec;//enregistre la position au moment ou on reçoit la commande de déplacement
        }
    }
    else if(fabsf(pilot_roll) < 0.15f && fabsf(pilot_pitch) < 0.15f){can_receive_cmd_xy = true;}//autorise le nouvel envoi d'une commande uniquement si le joystick est recentré

    if(can_receive_cmd_z)//si le drone est prêt à recevoir une commande de déplacement sur l'axe vertical
    {
        if(fabsf(pilot_throttle) >= 0.5f)//on vérifie le joystick
        {
            if(pilot_throttle > 0.0f){ move_z = -step_m;} //Monte, axe z inversé
            else{move_z = step_m;} //Descend

            //fait passer le drone en sous-mode "moving_z" et empêche une nouvelle commande
            received_cmd_z = true;
            can_receive_cmd_z = false;
            start_loc_vec = current_loc_vec;//enregistre la position au moment ou on reçoit la commande de déplacement
        }
    }
    else if(fabsf(pilot_throttle) < 0.15f){can_receive_cmd_z = true;}//autorise le nouvel envoi d'une commande uniquement si le joystick est recentré
}

void ModeStep::moving_xy()
{
    if (xy == 0) //Calcule une fois la cible à atteindre et lance le timer
    {
        target_loc_vec.x = start_loc_vec.x + move_x;
        target_loc_vec.y = start_loc_vec.y + move_y;
        target_loc_vec.z = start_loc_vec.z;
        move_x = 0.0f;
        move_y = 0.0f;
        xy++;
        move_start_ms = AP_HAL::millis();
    }
    
    moving();

    //Si la position est atteinte ou si le drone met trop de temps à l'atteindre
    if((fabsf(current_loc_vec.x - target_loc_vec.x) < 0.02 && fabsf(current_loc_vec.y - target_loc_vec.y) < 0.02) || (AP_HAL::millis() - move_start_ms >= waiting_time_ms))
    {
        received_cmd_xy = false; //retourne dans le sous-mode "waiting"
        stop_loc_vec = pos_control->get_pos_estimate_NED_m(); //enregistre la position d'arrêt de la manœuvre
        xy = 0;
        move_start_ms = 0;
    }
}

void ModeStep::moving_z()
{
    if (z == 0) //Calcule une fois la cible à atteindre et lance le timer
    {
        target_loc_vec.x = start_loc_vec.x;
        target_loc_vec.y = start_loc_vec.y;
        target_loc_vec.z = start_loc_vec.z + move_z;
        move_z = 0.0f;
        move_start_ms = AP_HAL::millis();
        z++;
    }

    moving();

    //Si la position est atteinte ou si le drone met trop de temps à l'atteindre
    if((fabsf(current_loc_vec.z - target_loc_vec.z) < 0.02) || (AP_HAL::millis() - move_start_ms >= waiting_time_ms))
    {
        received_cmd_z = false; //retourne dans le sous-mode "waiting"
        stop_loc_vec = pos_control->get_pos_estimate_NED_m(); //enregistre la position d'arrêt de la manœuvre
        z = 0;
        move_start_ms = 0;
    }
}

void ModeStep::moving()
{
    //calcul du décalage entre la position du drone et celle voulu
    if (Step_state == SubMode::Waiting)
    {pos_control->input_pos_NED_m(stop_loc_vec,0.0f,copter.wp_nav->get_terrain_margin_m());}//Objectif : sur-place
    else 
    {pos_control->input_pos_NED_m(target_loc_vec,0.0f,copter.wp_nav->get_terrain_margin_m());}//Objectif : se déplacer vers la position voulu
    
    //calcul de comment aller à la position voulu
    pos_control->NE_update_controller();
    pos_control->D_update_controller();
    //correction automatique pour retourner à la position voulu
    attitude_control->input_thrust_vector_rate_heading_rads(pos_control->get_thrust_vector(),get_pilot_desired_yaw_rate_rads());
}
#pragma once

#include <AP_Param/AP_Param.h>

class UserParameters {

public:
    UserParameters();
    static const struct AP_Param::GroupInfo var_info[];

    // Put accessors to your parameter variables here
    // UserCode usage example: g2.user_parameters.get_int8Param()
    AP_Int8 get_int8Param() const { return _int8; }
    AP_Int16 get_int16Param() const { return _int16; }
    AP_Float get_floatParam() const { return _float; }

    //Mode step
    //Renvoie dans le code les valeurs
    AP_Float get_step_dist() const {return _step_dist;}
    AP_Int32 get_waiting_time() const {return _waiting_time;}
    //récupère les valeurs demandées sur la RC
    void set_step_dist(float dist) { _step_dist.set_and_save(dist); }
    void set_waiting_time(uint32_t time) { _waiting_time.set_and_save(time); }
    
private:
    // Put your parameter variable definitions here
    AP_Int8 _int8;
    AP_Int16 _int16;
    AP_Float _float;
    //Mode step
    AP_Float _step_dist;
    AP_Int32 _waiting_time;
};

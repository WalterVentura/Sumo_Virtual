var line_state = 'feinting';
var sight_state;
var feint = Math.random() < 0.67 ? 'reversion+spin' : 'spin';
var counter = 0;
var reversion_counter;
var spin_counter = -1;
var turning_direction = 1;
var motor_left = 0;
var motor_right = 0;

const error_vector_size = 15;
var blind_flag = error_vector_size;
var error;
var derivative = 0;
var error_vector = [];
for (var i=0;i< error_vector_size;i++) 
{
    error_vector.push(0);
}
var error_index = 0;

const line_omega = 10.0;
const spin_omega = 40.0;
const search_omega = 40.0;
const lost_omega = 40.0;

var victory = 0;

const both_sensors = 0;
const one_sensor = 1;

function control(front_left, front_right, back_left, back_right, distance_left, distance_right) 
{
    const line_thres = 0.35;
    const left = -1;
    const right = 1;
    
    // Correction
    distance_left += distance_right;
    distance_right = distance_left - distance_right;
    distance_left -= distance_right;
    
    front_left += front_right;
    front_right = front_left - front_right;
    front_left -= front_right;
    
    counter++;
    var action = get_PID_action(distance_left, distance_right);
    switch(line_state)
    {
        case 'feinting':
            if(blind_flag == 0)
            {
                line_state = 'checking_for_line';
                sight_state = 'following';
            }
            else if((feint == 'reversion+spin' && counter == 27) ||
                    (feint == 'spin'           && counter == 15))
            {
                line_state = 'checking_for_line';
                sight_state = 'lost';
            }            
        break;

        case 'checking_for_line':
            if(front_left > line_thres)
            {
                if(front_right > line_thres)
                {
                    if(blind_flag == 0)
                    {
                        line_state = 'advancing_on_the_line';
                        counter = 0;
                    }
                    else
                    {
                        line_state = 'turning';
                        counter = 0;
                    }
                    spin_counter = 14;
                }
                else
                {
                    turning_direction = right;
                    line_state = 'advancing_on_the_line';
                    counter = 0;
                }
            }
            else
            {
                if(front_right > line_thres)
                {
                    turning_direction = left;
                    line_state = 'advancing_on_the_line';
                    counter = 0;
                }
                else
                {
                    if(blind_flag == 0)
                    {
                        sight_state = 'following';
                    }
                    else
                    {
                        switch(sight_state)
                        {
                            case 'searching':
                                if(counter > 45)
                                {
                                    sight_state = 'lost';
                                }
                            break;

                            case 'following':
                                if(blind_flag == error_vector_size)
                                {
                                    sight_state = 'lost';
                                } 
                            break;
                        }
                    }
                }
            }       
        break;

        case 'advancing_on_the_line':
            if(turning_direction == left)
            {
                if(front_right < 1 - line_thres)
                {
                    get_spin_counter(one_sensor);
                    
                    if(distance_right < 134)
                    {
                        victory = 1;
                    }
                    
                    line_state = 'reversing';
                    reversion_counter = counter;
                    counter = 0;

                }
                else if(front_left > line_thres)
                {
                    get_spin_counter(both_sensors);
             
                    if(blind_flag > 0)
                    {
                        line_state = 'turning';
                        counter = 0;
                    }                       
                }
            }
            else
            {
                if(front_left < 1 - line_thres)
                {
                    get_spin_counter(one_sensor);
                    
                    if(distance_left < 134)
                    {
                        victory = 1;
                    }
                    
                    line_state = 'reversing';
                    reversion_counter = counter;
                    counter = 0;
                }
                else if(front_right > line_thres)
                {
                    get_spin_counter(both_sensors);
                    
                    if(blind_flag > 0)
                    {
                        line_state = 'turning';
                        counter = 0;
                    }
                }
            }        
        break;

        case 'reversing':
            if(counter == reversion_counter)
            {
                line_state = 'turning';
                counter = 0;
            }
        break;

        case 'turning':
            if(victory == 0)
            {
                if(blind_flag == 0)
                {
                    line_state = 'checking_for_line';
                    sight_state = 'following';
                    spin_counter = -1;
                }
                else if(counter == spin_counter)
                {
                    line_state = 'checking_for_line';
                    sight_state = 'searching';
                    counter = 0;
                    spin_counter = -1;
                }
            }
            else
            {
                if(counter == spin_counter)
                {
                    line_state = 'checking_for_line';
                    sight_state = 'celebrating';
                    counter = 0;
                    spin_counter = -1;
                }
            }
        break;      
    }

    switch(line_state)
    {

        case 'feinting':
            if(feint == 'spin' || (feint == 'reversion+spin' && counter > 21))
            {
                motors(40,-40);
            }
            else
            {
                motors(-40,-40);
            }
        break;

        case 'checking_for_line':
            switch(sight_state)
            {
                case 'searching':
                    if(counter < 20)
                    {
                        motors(40,40);
                    }
                    else
                    {
                        motors(spin_omega*turning_direction, -spin_omega*turning_direction); 
                    }
                break;

                case 'following':
                    motors(search_omega + action, search_omega - action);  
                break;

                case 'lost':
                    motors(lost_omega,lost_omega);
                break;

                case 'celebrating':
                    motors(0,0);
                break;
            }       
        break;

        case 'advancing_on_the_line':
            motors(line_omega,line_omega);
        break;

        case 'reversing':
            motors(-line_omega, -line_omega);
        break;

        case 'turning':
            motors(spin_omega*turning_direction, -spin_omega*turning_direction);
        break;      
    }    
       
    return {
        leftSpeed: motor_left,
        rightSpeed: motor_right,
    };
}

function get_PID_action(distance_left, distance_right)
{
    const l = 200.0;
    const r_wheel = 40;
    
    const KP = 80.0;
    const KD = 500.0;

    var d = 0.57*l;
    var ds_L;
    var ds_R;
    
    const damping_angle = 0.25*Math.PI;
    var bot_omega;
    var damping_duration;
        
    if(distance_left > 134 && distance_right > 134) // Not seeing
    {
        if(blind_flag < error_vector_size)
        {
            if(blind_flag == 0)
            {
                error = error_vector[(error_index - 1 + error_vector_size)% error_vector_size] + (KD/KP)*derivative;
            }
            blind_flag++;                
        }
    }
    else
    {
        if(blind_flag == error_vector_size)
        {
            bot_omega = (motor_left - motor_right)*(r_wheel/l);
            damping_duration = Math.round(60*(damping_angle/bot_omega));
            damping_duration = constrain(damping_duration, 0 , error_vector_size);

            for(var i = 0; i < error_vector_size; i++)
            {
                if(i < damping_duration)
                {
                    error_vector[(error_index + i)%error_vector_size] = (bot_omega/60)*(damping_duration - i);
                }
                else
                {
                    error_vector[(error_index + i)%error_vector_size] = 0;
                }
            }       
        }
        blind_flag = 0;
        
        if(distance_left > 134) // Seeing Only with right sensor
        {
            ds_L = 1340;
            ds_R = 10*distance_right;
            
            error = Math.atan((l*(1+Math.sqrt(2)))/(4*(l - r_wheel + ds_R)));
        }
        else if(distance_right > 134) // Seeing only with left sensor
        {
            ds_L = 10*distance_left;
            ds_R = 1340;
            
            error = -Math.atan((l*(1+Math.sqrt(2)))/(4*(l - r_wheel + ds_L))); 
        }
        else // Seeing with both sensors
        {
            ds_L = 10*distance_left;
            ds_R = 10*distance_right;
            
            error = l*(ds_L - ds_R);
            error /= (ds_L +ds_R +2*l - 2*r_wheel)*Math.sqrt(d*d + (ds_L - ds_R)*(ds_L - ds_R));
            error = Math.atan(error); 
        }
    }
    
    derivative = (error - error_vector[error_index])/error_vector_size;
    error_vector[error_index] = error;
    error_index = (error_index + 1) % error_vector_size;    

    return (KP*error + KD*derivative);
}

function get_spin_counter(spin_type)
{ 
    if(spin_counter == -1)
    {
        const l = 200.0;
        const r_out = 770.0;
        const r_in = 720.0;
        const r_wheel = 40.0;
        var v = line_omega*r_wheel;
        var delta_t = (1.0/60)*counter;

        if(spin_type == both_sensors)
        {
            var x = l/r_in;
            var y = (v*delta_t)/r_in;
            var z = (x*x) + (y*y);

            var sin_theta = (-y/2) + x*Math.sqrt((1.0/z) - 0.25);
            sin_theta = constrain(sin_theta, 0.801, 0.990);         
        }
        else
        {
            var sin_theta = (r_out*r_out - r_in*r_in - (v*delta_t)*(v*delta_t))/(2*r_in*v*delta_t);
            sin_theta = constrain(sin_theta, 0.138, 0.801);      
        }
        var cos_theta = Math.sqrt(1 - (sin_theta*sin_theta));
        var angle = 0.5*Math.PI + Math.atan((r_in*sin_theta - l + r_wheel)/(r_in*cos_theta - 0.5*l));
        spin_counter = Math.round(60*angle*(l/(2*spin_omega*r_wheel)));
    }
}

function motors(expected_speed_left, expected_speed_right)
{
    const step_size = 13.33;
    
    var expected_rotation_speed;  //clockwise
    var expected_translation_speed; //foward
    var current_translation_speed; // foward
    
    expected_speed_left = constrain(expected_speed_left, -40, 40);
    expected_speed_right = constrain(expected_speed_right, -40, 40);
 
    current_translation_speed = (motor_left + motor_right)/2;
    expected_translation_speed = (expected_speed_left + expected_speed_right)/2;
    expected_rotation_speed = expected_speed_left - expected_translation_speed;
    
    if(expected_translation_speed > current_translation_speed + step_size)
    {
         current_translation_speed += step_size;
    }
    else
    {
        current_translation_speed = expected_translation_speed;     
    }
    
    motor_left = current_translation_speed + expected_rotation_speed;
    motor_right = current_translation_speed - expected_rotation_speed;
}

function constrain(value, min, max)
{
    return value < min ? min : (value > max ? max : value);
}    
var state = Math.random() < 0.67 ? 'dibre_reh' : 'dibre_giro';
var counter = 0;
var spin_counter;
var turning_direction;
var motor_left = 0
var motor_right = 0;

const KP = 80.0;
const KD = 500.0;

const error_vector_size = 15;
var blind_flag = error_vector_size;
var error;
var error_index = 0;

var error_vector = [];
for (var i=0;i< error_vector_size;i++) {
    error_vector.push(0);
}
var derivative = 0;

const line_omega = 10.0;
const spin_omega = 40.0;
const search_omega = 40.0;
const lost_omega = 40.0;

function control(front_left, front_right, back_left, back_right, distance_left, distance_right) 
{
    const line_thres = 0.50;
    const left = -1;
    const right = 1;
    
    // Correction
    distance_left += distance_right;
    distance_right = distance_left - distance_right;
    distance_left -= distance_right;
    
    switch(state)
    {
 
        case 'dibre_reh':
            
            counter++;
            get_PID_error(distance_left, distance_right);
            if(blind_flag == 0)
            {
                state ='searching';
            }
            else
            {
               if(counter == 26)
                {
                    state = 'lost';
                }
            }
            
        break;

        case 'dibre_giro':
            
            counter++;
            get_PID_error(distance_left, distance_right);
            if(blind_flag == 0)
            {
                state ='searching';
            }
            else
            {
               if(counter == 15)
                {
                    state = 'lost';
                }
            }
            
        break;            
            
        case 'searching':
           
            if(front_left > line_thres)
            {
                if(front_right > line_thres)
                {
                    turning_direction = Math.random() % 2;
                    if(turning_direction != right)
                    {
                        turning_direction = left;
                    }
                    state = 'turning';
                    spin_counter = 14;
                }
                else
                {
                    turning_direction = right;
                    state = 'advancing_on_the_line';
                    counter = 0;
                }
            }
            else
            {
                if(front_right > line_thres)
                {
                    turning_direction = left;
                    state = 'advancing_on_the_line';
                    counter = 0;
                }
                else
                {
                    get_PID_error(distance_left, distance_right);
                    if(blind_flag == error_vector_size)
                    {
                        state = 'lost';
                    }  
                }
            }
           
        break;
            
        case 'lost':
            
            if(front_left > line_thres)
            {
                if(front_right > line_thres)
                {
                    turning_direction = Math.random() % 2;
                    if(turning_direction != right)
                    {
                        turning_direction = left;
                    }
                    state = 'turning';
                    spin_counter = 14;
                }
                else
                {
                    turning_direction = right;
                    state = 'advancing_on_the_line';
                    counter = 0;
                }
            }
            else
            {
                if(front_right > line_thres)
                {
                    turning_direction = left;
                    state = 'advancing_on_the_line';
                    counter = 0;
                }
                else
                {
                    get_PID_error(distance_left, distance_right);
                    if(blind_flag == 0)
                    {
                        state = 'searching';
                    }  
                }
            }
            
        break;
                        
        case 'advancing_on_the_line':
            counter++;
            if(turning_direction == left)
            {
                if(front_left > line_thres)
                {
                    state = 'turning';
                    get_spin_counter();
                    
                }
                else 
                {
                    if(front_right < line_thres)
                    {
                        state = 'reversing';
                        get_spin_counter2();
                    }
                }
            }
            else
            {
                if(front_right > line_thres)
                {
                    state = 'turning';
                    get_spin_counter();
 
                }
                else
                {
                    if(front_left < line_thres)
                    {
                        state = 'reversing';
                        get_spin_counter2();
                    }
                }
            }
            
        break;
         
        case 'reversing':
            counter--;
            if(counter <= 0)
            {
                state = 'turning';
            }
        break;    
            
        case 'turning':
            get_PID_error(distance_left, distance_right);
            spin_counter--;
            if(spin_counter <= 0  || blind_flag == 0)
            {
                state = 'searching';
                counter = 0;
            }
        break;
              
    }
    
  
    switch (state) 
    {
        case 'dibre_reh':
            
            if(counter < 22)
            {
                motors(-40,-40);
            }
            else if(counter < 26)
            {
                motors(40,-40);
            }
            else
            {
                motors(0,0);
            }
               
        break;
   
        case 'dibre_giro':
            if(counter < 15)
            {
                motors(40,-40);
            }
            else
            {
                motors(0,0);
            }
                   
        break;            
            
        case 'searching':            
            motors(search_omega + KP*error + KD*derivative,
                   search_omega - KP*error - KD*derivative);        
        break;
            
        case 'lost':
            motors(lost_omega,lost_omega);
        break;
                               
        case 'advancing_on_the_line':
            motors(line_omega,line_omega);
        break;        
            
        case 'turning':
            motors(-spin_omega*turning_direction, spin_omega*turning_direction);
        break;
            
        case 'reversing':
            motors(-line_omega, -line_omega);
        break;                  
    }

    return {
        leftSpeed: motor_left,
        rightSpeed: motor_right,
        
        //log: [
              //{ name: 'Distance Left', value: distance_left, min: -300, max: 300 },
             // { name: 'Distance Right', value: distance_right, min: -300, max: 300 },
              //{ name: 'Erro', value: KP*error, min: -40, max: 40 },
              //{ name: 'Derivativo', value: KD*derivative, min: -40, max: 40 }
            //{ name: 'Tipo de Reversão', value: abelha, min: 0, max: 2 }
            
        //]
    };
}

function get_PID_error(distance_left, distance_right)
{
    const l = 200.0;
    const r_wheel = 40;
    
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
                error = error_vector[(error_index - 1 + error_vector_size)% error_vector_size] + (KD/80)*derivative;
            }
            blind_flag++;                
        }
    }
    else
    {
        if(blind_flag == error_vector_size)
        {
            bot_omega = (motor_left - motor_right)*(r_wheel/l);
            if(bot_omega == 0)
            {
                damping_duration = error_vector_size;
            }
            else
            {
                damping_duration = Math.round(60*(damping_angle/bot_omega));
                if(damping_duration > error_vector_size)
                {
                    damping_duration = error_vector_size;
                }
            }
            
         

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
}

function get_spin_counter()
{
    const l = 200.0;
    const r_in = 720.0;
    const r_wheel = 40.0;
    
    var x = l/r_in;
    var v = line_omega*r_wheel;
    var delta_t = (1.0/60)*counter;
    var y = (v*delta_t)/r_in;
    var z = (x*x) + (y*y);
    
    var cos_B = (-x/2) + y*Math.sqrt((1.0/z) - 0.25);
    if(cos_B > 0.120)
    {
        cos_B = 0.120;
    }
    else if(cos_B < -0.139)
    {
        cos_B = -0.139;
    }
    var sin_B = Math.sqrt(1 - (cos_B*cos_B));
    
    var angle = 0.5*Math.PI + Math.atan((r_in*sin_B - l + r_wheel)/(r_in*cos_B + 0.5*l));
    
    v = spin_omega*r_wheel;
    var bot_omega = (2*v)/l;
    var spin_time = angle/bot_omega;
    spin_counter = Math.round(1.0*60*spin_time);
}

function get_spin_counter2()
{
    const l = 200.0;
    const r_out = 770.0;
    const r_in = 720.0;
    const r_wheel = 40.0;
    var v = line_omega*r_wheel;
    var delta_t = (1.10/60)*counter;
    
    var sin_teta = (r_out*r_out - r_in*r_in - (v*delta_t)*(v*delta_t))/(2*r_in*v*delta_t);
    if(sin_teta > 0.918)
    {
        sin_teta = 0.918;
    }
    else if(sin_teta < 0.138)
    {
        sin_teta = 0.138;
    }
    var cos_teta = Math.sqrt(1 - (sin_teta*sin_teta));
    
    var angle = 0.5*Math.PI + Math.atan((r_in*sin_teta - l + r_wheel)/(r_in*cos_teta - 0.5*l));
    
    v = spin_omega*r_wheel;
    var bot_omega = (2*v)/l;
    var spin_time = angle/bot_omega;
    spin_counter = Math.round(1.0*60*spin_time);  
}    

function motors(expected_speed_left, expected_speed_right)
{
    const step_size = 80;
    
    if(expected_speed_left > 40)
    {
        expected_speed_left = 40;
    }
    else if(expected_speed_left < -40)
    {
        expected_speed_left = -40;
    }
    
    if(expected_speed_right > 40)
    {
        expected_speed_right = 40;
    }
    else if(expected_speed_right < -40)
    {
        expected_speed_right = -40;
    }
       
    
    if(expected_speed_left > motor_left && expected_speed_right > motor_right)
    {
        if(expected_speed_left - motor_left < step_size)
        {
            motor_left = expected_speed_left;
        }
        else
        {
            motor_left += step_size;
        }
        
        if(expected_speed_right - motor_right < step_size)
        {
            motor_right = expected_speed_right;
        }
        else
        {
            motor_right += step_size;
        }
    }
    else
    {
        motor_left = expected_speed_left;
        motor_right = expected_speed_right;
    }
}




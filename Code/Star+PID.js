
var state = 'searching';
var counter = 6000;
var spin_counter;
var turning_direction;
var motor_left, motor_right;

var error;


const line_omega = 10.0;
const spin_omega = 40.0;

function control(front_left, front_right, back_left, back_right, distance_left, distance_right) 
{
    const line_thres = 0.50;
    const left = -1;
    const right = 1;
    
    const KP = 80;
    
    var constrain;
    
    switch(state)
    {
        case 'searching':
            get_PID_error(distance_left, distance_right);
            counter++;
            if(counter > 6000)
            {
                state = 'lost';
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
            spin_counter--;
            if(spin_counter <= 0 )
            {
                state = 'searching';
                counter = 0;
            }
        break;
              
    }
    
  
    switch (state) 
    {
        case 'searching':
            constrain = KP*error;
            if(constrain > 40)
            {
                constrain = 40;
            }
            else 
            {
                if(constrain < -40)
                {
                    constrain = -40;
                }
            }
            
            motor_left = constrain;
            motor_right = -constrain; 
        break;
            
        case 'lost':
            motor_left = 40;
            motor_right = 40;
        break;
            
        case 'advancing_on_the_line':
            motor_left = line_omega;
            motor_right = line_omega;
        break;        
            
        case 'turning':
            motor_left = -spin_omega*turning_direction;
            motor_right = spin_omega*turning_direction;
        break;
            
         case 'reversing':
            motor_left = -line_omega;
            motor_right = -line_omega;
        break;      
            
    }

    return {
        leftSpeed: motor_left,
        rightSpeed: motor_right,
    };
}

function get_PID_error(distance_left, distance_right)
{
    const l = 200.0;
    const r_wheel = 40;
    var d = 0.57*l;
    
    var ds_L;
    var ds_R;
    
    var error_1;
    var error_2;
    
    var x_R;
    var x_L;
    var y_R;
    var y_L;
    var y_v;
    
    var sin_A;
    var cos_A;
    var tan_A;
    
    var x;
    var y;
    var invert_flag;
    
    if(distance_left > 134)
    {
        if(distance_right > 134)
        {
            error = 0;
        }
        else
        {
            ds_L = 1340;
            ds_R = 10*distance_right;
            error = -Math.atan((l*(1+Math.sqrt(2)))/(4*(l - r_wheel + ds_R)));
        }
    }
    else
    {
        ds_L = 10*distance_left;
        if(distance_right > 134)
        {
            ds_R = 1340;
            error = Math.atan((l*(1+Math.sqrt(2)))/(4*(l - r_wheel + ds_L)));  
        }
        else
        {
            ds_R = 10*distance_right;
            
            error_1 = l*(ds_L - ds_R);
            error_1 /= (ds_L +ds_R +2*l - 2*r_wheel)*Math.sqrt(d*d + (ds_L - ds_R)*(ds_L - ds_R));
            error_1 = -Math.atan(error_1);
            
           //Inverttion
           if(ds_L < ds_R)
            {
                ds_L += ds_R;
                ds_R = ds_L - ds_R;
                ds_L -= ds_R;
                invert_flag = 1;
            }
            else
            {
                invert_flag = 0;
            }
            
            x_R = d/2.0;
            x_L = -x_R;
            y_R = ds_R + l - r_wheel;
            y_L = ds_L + l - r_wheel;
            y_v = 0.5*(y_R + y_L - Math.sqrt((y_R + y_L)*(y_R + y_L) - 4*(x_L*x_R + y_L*y_R)));
            
            error_2 = Math.sqrt((y_R - y_v)*(y_R - y_v) + (x_R)*(x_R));
            sin_A = (y_R - y_v)/error_2;
            cos_A = x_R/error_2;
            tan_A = sin_A/cos_A;
            
            x = x_L*cos_A*cos_A - x_R*sin_A*sin_A + (y_L -y_R)*sin_A*cos_A + 0.5*l*(cos_A - sin_A);
            y = tan_A*(x - x_R) + y_R + 0.5*(l/cos_A);
            
            error_2 = -Math.atan(x/y);
            
            //Invertion
            if(invert_flag == 1)
            {
                error_2 *= -1;
            }
            
            error = 0.5*(error_1 + error_2);
         
        }
    }
 
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
    var sin_B = 1 - Math.sqrt(1 - (cos_B*cos_B));
    
    var angle = 0.5*Math.PI + Math.atan((r_in*sin_B - l + r_wheel)/(r_in*sin_B + 0.5*l));
    
    v = spin_omega*r_wheel;
    var bot_omega = (2*v)/l;
    var spin_time = angle/bot_omega;
    spin_counter = Math.round(5.7*60*spin_time);
}

function get_spin_counter2()
{
    const l = 200.0;
    const r_out = 770.0;
    const r_in = 720.0;
    const r_wheel = 40.0;
    var v = line_omega*r_wheel;
    var delta_t = (1.0/60)*counter;
    
    var sin_teta = (r_out*r_out - r_in*r_in - (v*delta_t)*(v*delta_t))/(2*r_in*v*delta_t);
    var cos_teta = 1 - Math.sqrt(1 - (sin_teta*sin_teta));
    
    var angle = 0.5*Math.PI + Math.atan((r_in*sin_teta - l + r_wheel)/(r_in*sin_teta - 0.5*l));
    
    v = spin_omega*r_wheel;
    var bot_omega = (2*v)/l;
    var spin_time = angle/bot_omega;
    spin_counter = Math.round(1.0*60*spin_time);    
}    


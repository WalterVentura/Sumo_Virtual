
var state = 'searching';
var counter;
var spin_counter;
var turning_direction;
var motor_left, motor_right;

const line_omega = 10.0;
const spin_omega = 40.0;

function control(front_left, front_right, back_left, back_right, distance_left, distance_right) 
{

    const line_thres = 0.50;
    const left = -1;
    const right = 1;
    
  
    switch(state)
    {
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
            }
        break;
              
    }
    
  
    switch (state) 
    {
        case 'searching':
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


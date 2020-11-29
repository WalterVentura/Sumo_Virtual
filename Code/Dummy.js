
    var state = Math.random() > 0.5 ? 'tl' : 'tr';
var counter = Math.round(15 * Math.random()) + 10;

function control(front_right, front_left, back_right, back_left, dist) {
    var left, right;
    switch (state) {
        case 's':
            if (front_left < 0.25 && front_right < 0.25) {
                state = 's';
            } else {
                if (front_left < 0.25) {
                    state = 'tr';
                } else {
                    state = 'tl';
                }
                counter = Math.round(10 * Math.random()) + 2;
            }
            break;
        case 'tl':
        case 'tr':
            if (counter > 0) {
                counter--;
            } else {
                state = 's';
            }
            break;
    }

    var speed = 40;

    switch (state) {
        case 's':
            left = speed;
            right = speed;
            break;
        case 'tr':
            left = -speed;
            right = speed;
            break;
        case 'tl':
            left = speed;
            right = -speed;
            break;
    }


    return {
        leftSpeed: left,
        rightSpeed: right
    };
}

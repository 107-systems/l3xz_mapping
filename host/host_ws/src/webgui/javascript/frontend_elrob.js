function button_up_onclick() {
    joydrive.up();
}

function button_down_onclick() {
    joydrive.down();
}

function button_left_onclick() {
    joydrive.left();
}

function button_right_onclick() {
    joydrive.right();
}

function button_emstop_onclick() {
    emergencyStop.press();
}

function button_emstop_change(pressed) {
    if(pressed)
    {
        joydrive.stop();
    }
    var btn = document.getElementById("button_emstop");
    if (btn == null) {
        return;
    }

    if (pressed) {
        btn.style.backgroundColor = 'yellow';
    }
    else {
        btn.style.backgroundColor = 'red';
    }
}

function button_download_onclick()
{
    vis1.download();
    vis2.download();
    vis3.download();
    vis4.download();
    vis5.download();
}



var fps = 10;
var refresh_rate = 1000 / fps;

var head_cam_view = document.getElementById("head_cam_view");
var hand_cam_view = document.getElementById("hand_cam_view");

function update_camera_view()
{
    head_cam_view.src = "/get_head_cam_view?" + (new Date).getTime();
    hand_cam_view.src = "/get_hand_cam_view?" + (new Date).getTime();
}

setInterval(update_camera_view, refresh_rate);
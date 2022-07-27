function setIdValue(id, val) {
  var view = document.getElementById(id)
  view.value = val.value;
};

// function makeNavigateOptions(n) {
//     var select = document.getElementsByName('navigateToSelect')[0]
//     console.log(select.name)

//     var option = document.createElement("option")
//     option.text = "Select Keyframe id..."

//     select.appendChild(option);

//     for(let i=0; i < n; ++i) {
//       var opt = document.createElement("option")
//       opt.text = "Keyframe" + String(i)
//       opt.value = String(i)
//       select.appendChild(opt)
//     }
// }

// $(document).on('show.bs.modal', '#navigateToModal', function (e) {
  
//   var select = document.getElementsByName('navigateToSelect')[0]
  
//   console.log(select.options.length)

//   if(select.options.length < 5)
//     makeNavigateOptions(5)
// });

var ros = new ROSLIB.Ros({
  url : 'ws://192.168.10.10:9090'
});

ros.on('connection', function() {
  document.getElementById("status").innerHTML = "Connected";
  var img = document.createElement('img'); 
    img.src = 'images/connect.png'; 
	// document.getElementById('status').appendChild(img);
});

ros.on('error', function(error) {
  document.getElementById("status").innerHTML = "Error";
});

ros.on('close', function() {
  document.getElementById("status").innerHTML = "Closed";
});

var txt_listener = new ROSLIB.Topic({
  ros : ros,
  name : '/txt_msg',
  messageType : 'std_msgs/String'
});

txt_listener.subscribe(function(m) {
  document.getElementById("msg").innerHTML = m.data + " move(1,0)";
  move(1,0)
});

var triggerClient = new ROSLIB.Service({
  ros : ros,
  name : '/ui_trigger_service',
  serviceType : 'std_srvs/Trigger'
});

var cmdClient = new ROSLIB.Service({
  ros : ros,
  name : '/ui_cmd_service',
  serviceType : 'spot_kinova_msgs/UiCmd'
});

var request = new ROSLIB.ServiceRequest({

});

function simple_cmd(cmd){
  console.log('called simple_cmd('+cmd+')')
  var req = new ROSLIB.ServiceRequest({command:cmd, arg:'0'});

  cmdClient.callService(req, function(result) {
    console.log('Result for service call on '
      + cmdClient.name
      + ': '
      + result.success
      + ', '
      + result.message);
  });
}

function simple_cmd_with_val(cmd, val){

  console.log('called simple_cmd_with_val('+cmd+','+val+')')
  var req = new ROSLIB.ServiceRequest({command:cmd, arg: val});

  cmdClient.callService(req, function(result) {
     console.log('Result for service call on '
       + cmdClient.name
       + ': '
       + result.success
       + ', '
       + result.message);
  });
  
}

function bodypose_sim(){
  console.log('called body()')

  const bodyRadio = document.getElementsByName('bodySimRadio');
  
  // '0' : flat body, '1' : tilt body
  if(bodyRadio[0].checked==true)
    arg_val = '1'
  
  if(bodyRadio[1].checked==true)
    arg_val = '0'

  console.log(arg_val)
  var cmd = new ROSLIB.ServiceRequest({command:'body_sim', arg: arg_val});

  cmdClient.callService(cmd, function(result) {
    console.log('Result for service call on '
      + cmdClient.name
      + ': '
      + result.success
      + ', '
      + result.message);
  });
}

cmd_vel_listener = new ROSLIB.Topic({
  ros : ros,
  name : "/cmd_vel",
  messageType : 'geometry_msgs/Twist'
});

move = function (linear, angular) {
  var twist = new ROSLIB.Message({
    linear: {
      x: linear,
      y: 0,
      z: 0
    },
    angular: {
      x: 0,
      y: 0,
      z: angular
    }
    });
  cmd_vel_listener.publish(twist);
}

createJoystick = function () {
  var options = {
    zone: document.getElementById('zone_joystick'),
    threshold: 0.1,
    position: { left: '50%', top: '20%'},
    mode: 'static',
    size: 150,
    color: '#000000',
  };

  manager = nipplejs.create(options);

  linear_speed = 0;
  angular_speed = 0;

  self.manager.on('start', function (event, nipple) {
    console.log("Movement start");
    timer = setInterval(function () {
        move(linear_speed, angular_speed);
    }, 25);
    console.log(linear_speed)
  });

  self.manager.on('move', function (event, nipple) {
    console.log("Moving");
    max_linear = 5.0; // m/s
    max_angular = 2.0; // rad/s
    max_distance = 75.0; // pixels;
    linear_speed = Math.sin(nipple.angle.radian) * max_linear * nipple.distance/max_distance;
    angular_speed = -Math.cos(nipple.angle.radian) * max_angular * nipple.distance/max_distance;
    console.log(linear_speed)
  });

  self.manager.on('end', function () {
    console.log("Movement end");
    if (timer) {
      clearInterval(timer);
    }
    self.move(0, 0);
    console.log(linear_speed)
  });
}

window.onload = function () {
  createJoystick();
}

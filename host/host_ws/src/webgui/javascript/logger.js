class Logger
{
  constructor(odom_topic)
  {
    this.odom = null;
    this.odomSub = new ROSLIB.Topic({
      ros : ros,
      name : odom_topic,
      messageType : '/nav_msgs/Odometry'
    });
    this.subscribe_odom();
  }

  subscribe_odom()
  {
    function handle(message)
    {
      logger.odom = message;
    }
    this.odomSub.subscribe(function(message)  { handle(message);  });
  }
  
  set_waypoint(topic, infotag)
  {
    if(this.odom)
    {
      var client = new ROSLIB.Service({
        ros : ros,
        name : topic,
        serviceType : 'l3xz_mapping/SetWaypoint'
      });

      var request = new ROSLIB.ServiceRequest({
      waypoint : {
          header: {
            seq : 0,
            frame_id : 'waypoint'
          },
          position : {
            x : logger.odom.pose.pose.position.x,
            y : logger.odom.pose.pose.position.y,
            z : logger.odom.pose.pose.position.z
          },
          tag : infotag
        }
      });

      client.callService(request, function(result) { console.log(result);});
    }
  }

  set_startpoint(topic, lat, lon)
  {
    console.log(this.odom)
    if(this.odom)
    {
      var client = new ROSLIB.Service({
        ros : ros,
        name : topic,
        serviceType : 'l3xz_mapping/SetStartpoint'
      });

      var request = new ROSLIB.ServiceRequest({
      startpoint : {
          position : {
            x : logger.odom.pose.pose.position.x,
            y : logger.odom.pose.pose.position.y,
            z : logger.odom.pose.pose.position.z
          },
          latitude : parseFloat(lat),
	  longitude: parseFloat(lon)
        }
      });

      client.callService(request, function(result) { console.log(result);});
    }
  }
  
}

var logger = new Logger('/odom_slam');

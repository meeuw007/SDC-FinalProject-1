from pid import PID
from lowpass import *
from yaw_controller import YawController
from math import *

##
#    Throttle PID
##
Kp_v = 0.05      # 0.08      # 0.08
Kd_v = 0.02      # 0.02      # 0.002
Ki_v = 0.001     # 0.005     # 0.001

##
#    Steering PID
##
Kp_s = 0.05      
Kd_s = 0.02      
Ki_s = 0.001    


##
#    Vehicle throttle control setting
## 
GAS_DENSITY = 2.858
ONE_MPH = 0.44704
MIN_THROTTLE = 0.0
MAX_THROTTLE = 1.0






# mynote: TODO: implement a feedback controller from pid,low pass (both for acceleration), yaw controller (steering)

class Controller(object):
    def __init__(self, *args, **kwargs):
        # TODO: Implement
        self.target_v = 0.0
        self.target_w = 0.0
        self.current_v = 0.0
        self.dbw_status = False
        self.vehicle_cfg = args[0]
        self.throttle = PID(Kp_v, Ki_v, Kd_v, mn=MIN_THROTTLE, mx=MAX_THROTTLE)
        self.error_v  = 0.0
        wheel_base    = self.vehicle_cfg['wheel_base']
        steer_ratio   = self.vehicle_cfg['steer_ratio']
        min_speed     = 0.0
        max_lat_accel = self.vehicle_cfg['max_lat_accel']
        max_steer_angle = self.vehicle_cfg['max_steer_angle']
        self.steering = YawController(wheel_base, steer_ratio, min_speed, max_lat_accel, max_steer_angle)
        

    def control(self, *args, **kwargs):
        # TODO: Change the arg, kwarg list to suit your needs (args: target_v,target_w,current_v,dbw_status,dt)
        #       Feedback controls: pid/lowpass (throttle) and yaw controller (steering)
        #       Throttle := [0,1], Brake := N*m, Steering := Radian
        
        # velocity args
        target_v   = args[0]#current_velocity.twist.linear
        target_w   = args[1]#current_velocity.twist.angular
        current_v  = args[2]#twist_cmd.twist.linear
        dbw_status = args[3]#dbw
        dt         = args[4]#sample time dt
        # linear speed error in x,y
        #v_current = sqrt(current_v.x**2 + current_v.y**2)  
        #v_target  = sqrt(target_v.x**2 + target_v.y**2)
        #v_error   = v_target - v_current
        v_current = abs(current_v.x)
        v_target  = abs(target_v.x)
        v_error   = abs(v_target - v_current)

        # Throttle control
        throttle_cmd = self.throttle.step(v_error, dt)
        
        # Steering angle- steering output proportional to (v_current/v_target)
        w_target = target_w.z
        steering_cmd = self.steering.get_steering(v_target, w_target, v_current)
        steering_cmd = (steering_cmd+pi)%(2*pi) - pi
        
        print("target v.x: ", target_v.x)
        print("target v.y: ", target_v.y)
        print("target v.z: ", target_v.z)
        print("target w.x: ", target_w.x)
        print("target w.y: ", target_w.y)
        print("target w.z: ", target_w.z)
        
        print("current v.x: ", current_v.x)
        print("current v.y: ", current_v.y)
        print("current v.z: ", current_v.z)
        
        print("throttle: ", throttle_cmd)
        print("steering: ", steering_cmd)
        
        
        # TODO:Brake control
        # if v_error > 0 go over the speed limit; time to brake
        # else                                  ; add pid throttle
        
        
        # Return throttle, brake, steer in this order
        return throttle_cmd, 0., steering_cmd

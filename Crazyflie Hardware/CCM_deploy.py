###################################
# IMPORTS

# Imports for crazyflie (the drone)
import logging
import time
import json
import numpy as np
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
# from cflib.utils.reset_estimator import reset_estimator
from multiprocessing import SimpleQueue

# Imports for qualisys (the motion capture system)
import asyncio
import xml.etree.cElementTree as ET
from threading import Thread
import qtm_rt as qtm
from scipy.spatial.transform import Rotation

# CCM modules
from np2pth import get_system_wrapper, get_controller_wrapper
import importlib
import sys
sys.path.append('../systems')
sys.path.append('../configs')
sys.path.append('../models')

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


###################################
# PARAMETERS

# Specify the uri of the drone to which you want to connect (if your radio
# channel is X, the uri should be 'radio://0/X/2M/E7E7E7E7E7')
uri = 'radio://0/24/2M/E7E7E7E7E7' # <-- FIXME

# Specify the variables you want to log at 100 Hz from the drone

variables = [
    'stateEstimate.x',
    'stateEstimate.y',
    'stateEstimate.z',
    'stateEstimate.roll',
    'stateEstimate.pitch',
    'stateEstimate.yaw',
    'stateEstimate.vx',
    'stateEstimate.vy',
    'stateEstimate.vz',
    # 'pm.vbat',
    # 'pwm.m1_pwm',
    # 'pwm.m2_pwm',
    # 'pwm.m3_pwm',
    # 'pwm.m4_pwm',
    # 'motor.m1',
    # 'motor.m2',
    # 'motor.m3',
    # 'motor.m4',
    'acc.z',
    'controller.cmd_thrust',
    'controller.roll',
    'controller.pitch',
    # 'ctrltarget.roll'
    # 'ctrltarget.pitch'    
]
# Specify the IP address of the motion capture system
ip_address = '128.174.245.64' 
# Specify the name of the rigid body that corresponds to your active marker
# deck in the motion capture system. If your marker deck number is X, this name
# should be 'marker_deck_X'.
marker_deck_name =  'marker_deck_20' 
# Specify the marker IDs that correspond to your active marker deck in the
# motion capture system. If your marker deck number is X, these IDs should be
# [X + 1, X + 2, X + 3, X + 4]. They are listed in clockwise order (viewed
# top-down), starting from the front.
marker_deck_ids =  [21, 22, 23, 24]


###################################
# CLIENT FOR CRAZYFLIE

class CrazyflieClient:
    def __init__(self, uri, use_controller=False, use_observer=False, marker_deck_ids=None):
        self.use_controller = use_controller
        self.use_observer = use_observer
        self.marker_deck_ids = marker_deck_ids
        self.cf = Crazyflie(rw_cache='./cache')
        self.cf.connected.add_callback(self._connected)
        self.cf.fully_connected.add_callback(self._fully_connected)
        self.cf.connection_failed.add_callback(self._connection_failed)
        self.cf.connection_lost.add_callback(self._connection_lost)
        self.cf.disconnected.add_callback(self._disconnected)
        print(f'CrazyflieClient: Connecting to {uri}')
        self.cf.open_link(uri)
        self.is_fully_connected = False
        self.data = {}

    def _connected(self, uri):
        print(f'CrazyflieClient: Connected to {uri}')

    def _fully_connected(self, uri):
        if self.marker_deck_ids is not None:
            print(f'CrazyflieClient: Using active marker deck with IDs {marker_deck_ids}')

            # Set the marker mode (3: qualisys)
            self.cf.param.set_value('activeMarker.mode', 3)

            # Set the marker IDs
            self.cf.param.set_value('activeMarker.front', marker_deck_ids[0])
            self.cf.param.set_value('activeMarker.right', marker_deck_ids[1])
            self.cf.param.set_value('activeMarker.back', marker_deck_ids[2])
            self.cf.param.set_value('activeMarker.left', marker_deck_ids[3])

        # Set Kalman estimator
        self.cf.param.set_value('stabilizer.estimator', 2)

        # Reset the default observer

        self.cf.param.set_value('kalman.resetEstimation', 1)
        time.sleep(0.1)
        self.cf.param.set_value('kalman.resetEstimation', 0)

        # Enable rate control for roll/pitch/yaw   1 =angle | 0 =rate
        self.cf.param.set_value('flightmode.stabModeRoll', 1)
        self.cf.param.set_value('flightmode.stabModePitch', 1)
        self.cf.param.set_value('flightmode.stabModeYaw', 0)

        if self.use_controller:
            self.cf.param.set_value('stabilizer.controller', 6)
        else:
            self.cf.param.set_value('stabilizer.controller', 1)  #1

        # Start logging
        self.logconfs = []
        self.logconfs.append(LogConfig(name=f'LogConf0', period_in_ms=10))
        num_variables = 0
        for v in variables:
            num_variables += 1
            if num_variables > 5: # <-- could increase if you paid attention to types / sizes (max 30 bytes per packet)
                num_variables = 0
                self.logconfs.append(LogConfig(name=f'LogConf{len(self.logconfs)}', period_in_ms=10))
            self.data[v] = {'time': [], 'data': []}
            self.logconfs[-1].add_variable(v)
        for logconf in self.logconfs:
            try:
                self.cf.log.add_config(logconf)
                logconf.data_received_cb.add_callback(self._log_data)
                logconf.error_cb.add_callback(self._log_error)
                logconf.start()
            except KeyError as e:
                print(f'CrazyflieClient: Could not start {logconf.name} because {e}')
                for v in logconf.variables:
                    print(f' - {v.name}')
            except AttributeError:
                print(f'CrazyflieClient: Could not start {logconf.name} because of bad configuration')
                for v in logconf.variables:
                    print(f' - {v.name}')

        print(f'CrazyflieClient: Fully connected to {uri}')
        self.is_fully_connected = True

    def _connection_failed(self, uri, msg):
        print(f'CrazyflieClient: Connection to {uri} failed: {msg}')

    def _connection_lost(self, uri, msg):
        print(f'CrazyflieClient: Connection to {uri} lost: {msg}')

    def _disconnected(self, uri):
        print(f'CrazyflieClient: Disconnected from {uri}')
        self.is_fully_connected = False

    def _log_data(self, timestamp, data, logconf):
        for v in logconf.variables:
            self.data[v.name]['time'].append(timestamp / 1e3)
            self.data[v.name]['data'].append(data[v.name])

    def _log_error(self, logconf, msg):
        print(f'CrazyflieClient: Error when logging {logconf}: {msg}')

    def move(self, x, y, z, yaw, dt):
        print(f'CrazyflieClient: Move to {x}, {y}, {z} with yaw {yaw} degrees for {dt} seconds')
        start_time = time.time()
        while time.time() - start_time < dt:
            self.cf.commander.send_position_setpoint(x, y, z, yaw)
            time.sleep(0.1)

    def pwm_to_thrust(self,pwm, vbat):
        p_hat = pwm / 65536.0
        v_hat = vbat / 4.2
        thrust_gf = 11.09 - 39.08 * p_hat - 9.53 * v_hat + 20.57 * p_hat**2 + 38.43 * p_hat * v_hat
        return thrust_gf * 9.81 / 1000.0  # gf → N

    def f_to_pwm(self,f_hat, v):
        v_hat = v / 4.2
        p_hat = 0.5 + 0.12 * f_hat - 0.41 * v_hat - 0.002 * f_hat**2 - 0.043 * f_hat * v_hat
        # pwm = np.clip(np.round(10001+ p_hat * (60000-10001)), 10001, 60000).astype(int)
        pwm =   np.clip(np.round(p_hat*60000), 10001, 60000).astype(int)
        return pwm

    def CCM_track(self, xstar, ustar, controller, dt=0.01, m=0.031, g=9.81):
        print("STARTING TRACKING")
        num_steps = len(xstar[0,:])
         # Match trajectory timestep to controller update rate
        self.track_log = {
        'xcurr': [],
        'u': [],
        'f_cmd': [],
        'timestamp': [],
        'target_attitude':[]
        }
        throttle_prev= self.data['controller.cmd_thrust']['data'][-1]
        # print("throttle_prev :", throttle_prev)

        for i in range(num_steps):
            t_start = time.time()

            # Get current state from client
            try:
                pos = np.array([
                    self.data['stateEstimate.y']['data'][-1],
                    self.data['stateEstimate.x']['data'][-1],
                    self.data['stateEstimate.z']['data'][-1],  #due to negative z direction of model used
                ])
                vel_body = np.array([
                    self.data['stateEstimate.vy']['data'][-1],
                    self.data['stateEstimate.vx']['data'][-1],
                    self.data['stateEstimate.vz']['data'][-1],
                ])
                roll = self.data['stateEstimate.roll']['data'][-1] * 3.14/180
                pitch = -self.data['stateEstimate.pitch']['data'][-1] *3.14/180
                # yaw = self.data['stateEstimate.yaw']['data'][-1]
                # pwm_vals = [
                #     self.data['motor.m1']['data'][-1],
                #     self.data['motor.m2']['data'][-1],
                #     self.data['motor.m3']['data'][-1],
                #     self.data['motor.m4']['data'][-1],
                # ]
                # vbat = self.data['pm.vbat']['data'][-1]

                # f_curr = sum(self.pwm_to_thrust(pwm, vbat) for pwm in pwm_vals)  / (m * g) 
                f_curr = self.data['acc.z']['data'][-1]

                #Obtain current state for CCM controller
                xcurr = np.concatenate([pos, vel_body, [f_curr*g], [roll], [pitch]])
            

                u= np.ravel(np.array(controller(xcurr,xcurr-xstar[:,i], ustar[:,i])))


                roll_cmd= roll* (180/3.14)+u[1]*dt*(180/3.14)   
                pitch_cmd = pitch*(180/3.14) + u[2]*dt *(180/3.14)     


                f_cmd = f_curr  + (u[0]/g)*dt   #f_curr + (u[0]/g)*T_loop #due to negative z direction of model used

                # Proportional controller for throttle to RPM mapping
                throttle =  np.clip(np.round(throttle_prev+ 400*u[0]*dt), 10001, 60000).astype(int)  #(throttle_prev+ 400*u[0]*dt).astype(int)
                throttle_prev=throttle


                self.track_log['xcurr'].append(xcurr.copy())
                self.track_log['u'].append(u.copy())
                self.track_log['f_cmd'].append(f_cmd)
                self.track_log['timestamp'].append(i)
                att = np.array([self.data['controller.roll']['data'][-1],self.data['controller.pitch']['data'][-1] ])  
                self.track_log['target_attitude'].append(att.copy())


                # Send to Crazyflie. No Yaw control here.
                self.cf.commander.send_setpoint(roll_cmd,pitch_cmd, 0.0, throttle)  #due to negative z direction of model used

 


            except Exception as e:
                print(f"⚠️ Control step {i} failed: {e}")

            # Check command publishing latency
            # print(max(0, dt - time.time() +t_start))
            time.sleep(max(0, dt - time.time() +t_start))

    def stop(self, dt):
        print(f'CrazyflieClient: Stop for {dt} seconds')
        self.cf.commander.send_stop_setpoint()
        self.cf.commander.send_notify_setpoint_stop()
        start_time = time.time()
        while time.time() - start_time < dt:
            time.sleep(0.1)

    def disconnect(self):
        self.cf.close_link()




#########################################
class MocapTransform:
    def __init__(self, d1=0.0136, d2=0.0109):
        # Marker-to-body and marker-to-world offsets
        self.R_inA_ofB = np.eye(3)
        self.p_inA_ofB = np.array([0., 0., -d1])
        self.R_inA_ofW = np.eye(3)
        self.p_inA_ofW = np.array([0, 0., -d1 - d2])

        # These will be computed from the first mocap frame
        self.initialized = False
        self.R_inW_ofQ = None
        self.p_inW_ofQ = None

    def update_and_transform(self, mocap_position, mocap_euler):
        """
        mocap_position: [x, y, z]
        mocap_euler: [yaw, pitch, roll] in radians (ZYX order)
        Returns: (position, quaternion) in Crazyflie world frame
        """
        if not np.all(np.isfinite(mocap_position)) or not np.all(np.isfinite(mocap_euler)):
            # print("⚠️ Skipping transformation due to NaN in input:", mocap_position, mocap_euler)
            return np.full(3, np.nan), np.full(4, np.nan)

        # Mocap frame (Q): pose of active marker in mocap world
        R_inQ_ofA = Rotation.from_euler('ZYX', mocap_euler).as_matrix()
        p_inQ_ofA = np.array(mocap_position)

        if not self.initialized:
            # Pose of drone world in mocap frame
            R_inQ_ofW = R_inQ_ofA @ self.R_inA_ofW
            p_inQ_ofW = p_inQ_ofA + R_inQ_ofA @ self.p_inA_ofW
            # Invert: mocap world in drone world
            self.R_inW_ofQ = R_inQ_ofW.T
            self.p_inW_ofQ = -R_inQ_ofW.T @ p_inQ_ofW
            self.initialized = True

        # Transform: active marker in mocap → body in Crazyflie world
        R_inW_ofB = Rotation.from_matrix(self.R_inW_ofQ @ R_inQ_ofA @ self.R_inA_ofB)
        p_inW_ofB = self.p_inW_ofQ + self.R_inW_ofQ @ (p_inQ_ofA + R_inQ_ofA @ self.p_inA_ofB)
        q_cf = R_inW_ofB.as_quat()  # [x, y, z, w]

        return p_inW_ofB, q_cf  # position [x, y, z], quaternion [x, y, z, w]

###################################
# CLIENT FOR QUALISYS

class QualisysClient(Thread):
    def __init__(self, ip_address, marker_deck_name, pose_queue):
        Thread.__init__(self)
        self.ip_address = ip_address
        self.marker_deck_name = marker_deck_name
        self.connection = None
        self.qtm_6DoF_labels = []
        self._stay_open = True
        self.pose_streaming = False
        self.data = {
            'time': [],
            'x': [],
            'y': [],
            'z': [],
            'yaw': [],
            'pitch': [],
            'roll': [],
        }
        self.data_transformed = {
            'time': [],
            'x': [],
            'y': [],
            'z': [],
            'yaw': [],
            'pitch': [],
            'roll': [],
        }
        self.mocap_transform= MocapTransform()
        self.pose_queue = pose_queue
        self.start()

    def close(self):
        self.pose_queue.put('END')
        self._stay_open = False
        self.join()

    def run(self):
        asyncio.run(self._life_cycle())

    async def _life_cycle(self):
        await self._connect()
        while (self._stay_open):
            await asyncio.sleep(1)
        await self._close()

    async def _connect(self):
        print('QualisysClient: Connect to motion capture system')
        self.connection = await qtm.connect(self.ip_address, version='1.24')
        params = await self.connection.get_parameters(parameters=['6d'])
        xml = ET.fromstring(params)
        self.qtm_6DoF_labels = [label.text.strip() for index, label in enumerate(xml.findall('*/Body/Name'))]
        await self.connection.stream_frames(
            components=['6d'],
            on_packet=self._on_packet,
        )

    def _on_packet(self, packet):
        header, bodies = packet.get_6d()

        if bodies is None:
            print(f'QualisysClient: No rigid bodies found')
            return

        if self.marker_deck_name not in self.qtm_6DoF_labels:
            print(f'QualisysClient: Marker deck {self.marker_deck_name} not found')
            return

        index = self.qtm_6DoF_labels.index(self.marker_deck_name)
        position, orientation = bodies[index]

        # Get time in seconds, with respect to the qualisys clock
        t = packet.timestamp / 1e6

        # Get position of marker deck (x, y, z in meters)
        x, y, z = np.array(position) / 1e3

        # Get orientation of marker deck (yaw, pitch, roll in radians)
        Rot = Rotation.from_matrix(np.reshape(orientation.matrix, (3, -1), order='F'))
        yaw, pitch, roll = Rot.as_euler('ZYX', degrees=False)
        # print("raw :", [x,y,z,yaw,pitch,roll])
        # Store time, position, and orientation
        self.data['time'].append(t)
        self.data['x'].append(x)
        self.data['y'].append(y)
        self.data['z'].append(z)
        self.data['yaw'].append(yaw)
        self.data['pitch'].append(pitch)
        self.data['roll'].append(roll)

        # Transform to world frame of drone
        pos_cf, quat_cf = self.mocap_transform.update_and_transform([x, y, z],[yaw, pitch, roll])

        # Check if the measurements are valid
        if np.isfinite(x):
            self.pose_streaming=True
            qx, qy, qz, qw = Rot.as_quat()
            if self.pose_queue.empty():
                #send transformed pose
                self.pose_queue.put((pos_cf[0], pos_cf[1], pos_cf[2], quat_cf[0], quat_cf[1], quat_cf[2], quat_cf[3]))
        else:
            self.pose_streaming=False


    async def _close(self):
        await self.connection.stream_frames_stop()
        self.connection.disconnect()

def send_poses(client, queue):
    print('Start sending poses')
    while True:
        pose = queue.get()
        if pose == 'END':
            print('Stop sending poses')
            break
        x, y, z, qx, qy, qz, qw = pose

        client.cf.extpos.send_extpose(x, y, z, qx, qy, qz, qw)


###################################
# FLIGHT CODE

if __name__ == '__main__':
    g=9.81
    system = importlib.import_module('system_QUADROTOR_9D')
    controller = get_controller_wrapper('../log_QUADROTORM_R100_0.5_25_0.8/controller_best.pth.tar') 

    data =   np.load('quadN_high_speed2_T3.npz')     #np.load('quadN_MOF_T7hlf.npz')
    dt=1/100
    Xstar = data['X']
    Ustar = data['U']

    

    # Specify whether or not to use the motion capture system
    use_mocap = True

    # Initialize radio
    cflib.crtp.init_drivers()

    # Create and start the client that will connect to the drone
    drone_client = CrazyflieClient(
        uri,
        use_controller=False,
        use_observer=False,
        marker_deck_ids=marker_deck_ids if use_mocap else None,
    )

    # Wait until the client is fully connected to the drone
    while not drone_client.is_fully_connected:
        time.sleep(0.1)

    # Create and start the client that will connect to the motion capture system
    if use_mocap:
        pose_queue = SimpleQueue()
        Thread(target=send_poses, args=(drone_client, pose_queue)).start()
        mocap_client = QualisysClient(ip_address, marker_deck_name, pose_queue)




    drone_client.stop(3.0)

    if use_mocap:
        while not mocap_client.pose_streaming:
            print('WAITING FOR MOCAP STREAM ')
            time.sleep(0.5)
    try:
        # Takeoff

        drone_client.move(0.0, 0.0, 0.5, 0.0, 3.0)

        ###################### START TRACKING #######################
        drone_client.cf.commander.send_setpoint(0, 0, 0, 0)
        drone_client.CCM_track(Xstar, Ustar, controller, dt=dt)
        ############################################################

        # Initiate HOLD-and-LAND sequence
        drone_client.move(
            drone_client.data['stateEstimate.x']['data'][-1],
            drone_client.data['stateEstimate.y']['data'][-1],
            drone_client.data['stateEstimate.z']['data'][-1], 0.0, 3.0
        )

        drone_client.move(
            drone_client.data['stateEstimate.x']['data'][-1],
            drone_client.data['stateEstimate.y']['data'][-1],
            0.1, 0.0, 1.0
        )
        drone_client.stop(1.0)

    except KeyboardInterrupt:
        print("\n[INFO] KeyboardInterrupt caught! Stopping the drone...")
        drone_client.stop(1.0)

    finally:
        # Always disconnect cleanly
        print("[INFO] Disconnecting from Crazyflie...")
        drone_client.disconnect()


    # Disconnect from the motion capture system
    if use_mocap:
        mocap_client.close()

    # Assemble flight data from both clients
    controller_log = {
    'xcurr': [x.tolist() for x in drone_client.track_log['xcurr']],
    'u': [u.tolist() for u in drone_client.track_log['u']],
    'f_cmd': drone_client.track_log['f_cmd'],
    'timestamp': drone_client.track_log['timestamp'],
    'target_attitude': [att.tolist() for att in drone_client.track_log['target_attitude']]
    }

    data = {}
    data['drone'] = drone_client.data
    data['mocap'] = mocap_client.data if use_mocap else {}
    data['controller']= controller_log
    # data['mocap_transformed'] = mocap_client.data_transformed if use_mocap else {}

    # Write flight data to a file
    with open('CCM_high_speed_T3.json', 'w') as outfile:
        json.dump(data, outfile, sort_keys=False)

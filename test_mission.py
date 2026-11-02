"""ROSなしの単体・模擬制御試験。実ROS/Gazebo試験の代用ではない。"""
import importlib.util
import math
import sys
import types
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import yaml
from approach_logic import bearing_deg, estimate_distance, planned_travel, progress, select_box
from mission_logic import CommandHeading, classify_regions


def stub(name, **attrs):
    module = types.ModuleType(name)
    module.__dict__.update(attrs)
    sys.modules[name] = module


class Twist:
    def __init__(self):
        self.linear, self.angular = NS(x=0.,y=0.,z=0.), NS(x=0.,y=0.,z=0.)


class Goal:
    def __init__(self): self.time_allowance = NS(sec=0,nanosec=0)


for name in ('cv2','cv_bridge','rclpy','action_msgs','geometry_msgs','nav2_msgs','nav_msgs',
             'sensor_msgs','lifecycle_msgs','rcl_interfaces','std_srvs','std_msgs'):
    stub(name)
stub('cv_bridge', CvBridge=object)
stub('rclpy.node', Node=object)
stub('rclpy.action', ActionClient=object)
stub('rclpy.clock', Clock=object, ClockType=NS(STEADY_TIME=1))
stub('rclpy.qos', qos_profile_sensor_data=object())
stub('rclpy.parameter', Parameter=object)
stub('rclpy.signals', SignalHandlerOptions=NS(NO=0))
stub('action_msgs.msg', GoalStatus=NS(STATUS_SUCCEEDED=4))
stub('geometry_msgs.msg', PoseStamped=object, Twist=Twist)
stub('nav_msgs.msg', Odometry=object)
stub('sensor_msgs.msg', Image=object, CameraInfo=object, LaserScan=object)
stub('lifecycle_msgs.srv', GetState=NS(Request=object))
stub('nav2_msgs.action', **{n:NS(Goal=Goal) for n in ('NavigateToPose','FollowWaypoints','Spin')})
stub('rcl_interfaces.msg', Parameter=object, ParameterValue=object, ParameterType=NS(PARAMETER_BOOL=1))
stub('rcl_interfaces.srv', GetParameters=object, SetParameters=object)
stub('std_srvs.srv', Trigger=NS(Request=object), SetBool=NS(Request=object))
stub('std_msgs.msg', String=lambda **kw: NS(**kw))
import mission
from visual_approach import VisualApproach

ROOT=Path(__file__).parent
CONFIG=yaml.safe_load((ROOT/'config.yaml').read_text())


class FakeMission(mission.Mission):
    def __init__(self):
        self.cfg=CONFIG
        self.ap=dict(CONFIG['approach'])
        self.events=[]; self.heading=CommandHeading(); self.images=[]
        self.frame='map'; self.run_dir=Path('test_images'); self.direct_owned=False
        self.follow='follow'; self.navigate='navigate'; self.spin_client='spin'
        self.state='STARTING'; self.confirmed=False; self.ocr_process=None
        self.original_layers={'/local':True,'/global':False}
        self.hooks=NS(choose_box_image=lambda records,ctx: records[0],
                      box_distance=lambda record,ctx: 4., box_visible=lambda path,ctx: True)
        self.next_client=NS(call_async=lambda req: NS(success=True,message='accepted'))
        self.total_travel=0.
    def get_logger(self): return Mock()
    def preflight(self): return Path('routes'), {'x':0.,'y':1.8}
    def context(self): return {}
    def pose(self, p): return p
    def set_state(self,s): self.state=s; self.events.append(('state',s))
    def start_action(self,c,g,feedback=None): self.events.append(('action',c))
    def finish_action(self): return NS(missed_waypoints=[])
    def wait_future(self,f,timeout=None): return f
    def load_route(self,p): self.events.append(('route',p.name)); return []
    def rotate(self,deg): self.events.append(('spin',deg)); self.heading.commit(deg)
    def capture(self,label):
        r={'path':label,'command_deg':self.heading.degrees}; self.images.append(r)
        self.events.append(('photo',label,self.heading.degrees)); return r
    def align_box(self,r): self.face_command(r['command_deg']); self.rotate(2.); return self.capture('aligned')
    def ocr(self,r): return 'c'
    def layers(self,on): self.events.append(('layers',on))
    def approach_halfway(self,d): self.events.append(('half',d)); return 'halfway'
    def drive_straight(self,d,final=False): self.events.append(('drive',d,final)); return 'blocked'
    def wait_arrival(self): self.events.append(('confirm',))
    def stop_direct(self): self.events.append(('zero',))
    def cancel_action(self): self.events.append(('cancel',))
    def front_stop(self,on): self.events.append(('front_stop',on))
    def layer_call(self,n,v): self.events.append(('restore',n,v))


class Tests(unittest.TestCase):
    def test_bearing_sign(self):
        k={'fx':100.,'cx':100.}
        self.assertAlmostEqual(bearing_deg([40,0,60,10],k),26.565051177)
        self.assertAlmostEqual(bearing_deg([140,0,160,10],k),-26.565051177)
    def test_distance_and_halfway_geometry(self):
        k={'width':1280,'height':720,'fy':1000.}
        cfg=dict(CONFIG['approach'])
        self.assertAlmostEqual(estimate_distance([200,100,400,255],k,cfg),2.)
        self.assertAlmostEqual(planned_travel(2.,cfg),1.5)
    def test_clipped_box_rejected(self):
        for b in ([0,100,50,200],[100,0,200,100],[100,100,1280,200],[100,100,200,720]):
            with self.assertRaises(ValueError): estimate_distance(b,{'width':1280,'height':720,'fy':1000.},CONFIG['approach'])
    def test_unrealistic_distance_and_close_rejected(self):
        with self.assertRaises(ValueError): estimate_distance([10,10,20,11],{'width':1280,'height':720,'fy':1000.},CONFIG['approach'])
        with self.assertRaises(ValueError): planned_travel(0.2,CONFIG['approach'])
    def test_largest_valid_box(self):
        p={'image_size':[100,100], 'regions':[{'bbox':[10,10,20,30]}, {'bbox':[20,20,70,80]}]}
        self.assertEqual(select_box(p)['bbox'],[20.,20.,70.,80.])
        self.assertIsNone(select_box(dict(p,regions=[])))
    def test_invalid_bbox_rejected(self):
        for box in ([0,0,101,10],[10,0,0,10],[0,0,float('nan'),20]):
            with self.assertRaises(ValueError): select_box({'image_size':[100,100],'regions':[{'bbox':box}]})
    def test_progress_in_robot_heading(self):
        d,c,a=progress((1.,2.,math.pi/2),(1.,4.,math.pi/2))
        self.assertAlmostEqual(d,2.); self.assertAlmostEqual(c,0.); self.assertAlmostEqual(a,0.)
    def test_ocr_rules_unchanged(self):
        self.assertEqual(classify_regions({'regions':[{'text':'A'},{'text':' C '}]}),'c')
        with self.assertRaises(ValueError): classify_regions({'regions':[{'text':'abc'}]})
    def test_full_flow_midpoint_once_and_inverse(self):
        n=FakeMission(); n.run()
        self.assertIn(('route','c.yaml'),n.events)
        photos=[e[2] for e in n.events if e[0]=='photo' and e[1].startswith('search_')]
        self.assertEqual(photos,[0,15,30,45,60,75])
        self.assertEqual(len([e for e in n.events if e[0]=='half']),1)
        self.assertEqual(len([e for e in n.events if e[0]=='drive' and e[2]]),1)
        self.assertEqual(len([e for e in n.events if e[0]=='action' and e[1]=='navigate']),1)
        self.assertEqual(n.heading.degrees,0.)
        self.assertLess(n.events.index(('confirm',)),n.events.index(('state','RETURN_HEADING')))
        self.assertEqual(n.events[-1],('state','DONE'))
    def test_early_stop_skips_midpoint(self):
        n=FakeMission(); n.approach_halfway=lambda d:'blocked'; n.run()
        self.assertFalse(any(e[:2]==('state','MIDWAY_CAPTURE') for e in n.events))
        self.assertIn(('confirm',),n.events)
    def test_no_box_no_drive(self):
        n=FakeMission(); n.hooks.choose_box_image=lambda r,c:None
        with self.assertRaises(RuntimeError): n.run()
        self.assertFalse(any(e[0] in ('half','drive') for e in n.events))
        self.assertNotIn(('state','NEXT_FILE'),n.events)
    def test_failed_waypoint_no_search(self):
        n=FakeMission(); n.finish_action=lambda:NS(missed_waypoints=[0])
        with self.assertRaises(RuntimeError): n.run()
        self.assertFalse(any(e[0]=='photo' and e[1].startswith('search_') for e in n.events))
    def test_midway_right_search(self):
        n=FakeMission(); n.heading.commit(30.)
        n.hooks.box_visible=lambda p,c:False
        n.hooks.choose_box_image=lambda r,c:r[1]
        VisualApproach.midway_check(n)
        self.assertEqual([r['command_deg'] for r in n.images[:3]],[30,40,20])
        self.assertEqual(n.heading.degrees,22.)
    def test_confirmation_only_when_stopped(self):
        n=FakeMission(); r=NS(success=False,message='')
        n.state='APPROACH_FINAL'; n.confirm_callback(None,r); self.assertFalse(r.success)
        n.state='WAIT_CONFIRM'; n.confirm_callback(None,r); self.assertTrue(r.success)
    def test_failure_zero_cancel_restore(self):
        n=FakeMission(); self.assertEqual(n.cleanup_failure(),[])
        self.assertIn(('zero',),n.events); self.assertIn(('cancel',),n.events)
        self.assertIn(('restore','/local',True),n.events); self.assertIn(('restore','/global',False),n.events)
        self.assertNotIn(('state','NEXT_FILE'),n.events)
    def test_spin_failure_restores_front_zone(self):
        n=FakeMission(); n._rotate_action=Mock(side_effect=RuntimeError('abort'))
        with self.assertRaises(RuntimeError): mission.Mission.rotate(n,10.)
        self.assertEqual(n.heading.degrees,0.)
        self.assertEqual(n.events[-2:],[('front_stop',False),('front_stop',True)])


class MotionTests(unittest.TestCase):
    def run_motion(self, mode, final=False):
        n=FakeMission(); now=[10.]; n.vel_pub=Mock()
        n.odom=(0.,0.,0.); n.odom_received=10.; n.output=Twist(); n.output.linear.x=0.15
        n.output_received=10.; n.gate_received=10.; n.gate_status={'owner':'mission','scan_ok':True}
        n.ap.update(output_grace_seconds=0.1, stop_hold_seconds=0.1, stall_seconds=0.5,max_drive_seconds=2.)
        n.set_direct=lambda enabled:None
        n.wait_condition=lambda *a:None
        def pump():
            now[0]+=0.05
            if mode!='stale_odom': n.odom_received=now[0]
            if mode!='stale_output': n.output_received=now[0]
            n.gate_received=now[0]
            if mode=='scan_loss': n.gate_status['scan_ok']=False
            if mode in ('move','stale_output','stale_odom'): n.odom=(n.odom[0]+0.01,0.,0.)
            if mode=='blocked': n.output.linear.x=0.
            if mode=='cross': n.odom=(0.,0.3,0.)
            if mode=='jump': n.odom=(2.,0.,0.)
        n.pump=pump
        with patch('visual_approach.time.monotonic',side_effect=lambda:now[0]):
            try:
                result=VisualApproach.drive_straight(n,0.2 if mode=='move' else 5.,final=final)
                return result,n
            except Exception:
                self.assertIn(('zero',),n.events)
                raise
    def test_zero_command_waits_for_stationary_odom(self):
        n=FakeMission(); n.direct_owned=True; n.vel_pub=Mock()
        n.odom_speed=(0.1,0.); now=[10.]; n.odom_received=10.
        def pump():
            now[0]+=0.1; n.odom_received=now[0]
            if now[0]>10.4: n.odom_speed=(0.,0.)
        n.pump=pump
        with patch('visual_approach.time.monotonic',side_effect=lambda:now[0]):
            VisualApproach.stop_direct(n)
        self.assertGreaterEqual(now[0],10.6)
        self.assertTrue(all(c.args[0].linear.x==0. for c in n.vel_pub.publish.call_args_list))

    def test_midpoint_stops(self): self.assertEqual(self.run_motion('move')[0],'halfway')
    def test_monitor_stop_latches(self): self.assertEqual(self.run_motion('blocked')[0],'blocked')
    def test_final_distance_limit_is_failure(self):
        with self.assertRaises(RuntimeError): self.run_motion('move',final=True)
    def test_sensor_or_motion_faults_stop(self):
        for mode in ('stale_odom','stale_output','scan_loss','cross','jump','stalled'):
            with self.subTest(mode=mode), self.assertRaises((RuntimeError,TimeoutError)):
                self.run_motion(mode)


class GateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path=ROOT.parent/'field_world/field_world/command_gate.py'
        spec=importlib.util.spec_from_file_location('gate_under_test',path)
        cls.mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(cls.mod)
    def gate(self):
        g=object.__new__(self.mod.CommandGate)
        g.owner=True; g.timeout=0.4; g.scan_timeout=0.7;g.pub=Mock();g.status=Mock()
        g.scan_ok=lambda:True
        moving=Twist();moving.linear.x=0.15
        g.commands={'nav':(moving,10.),'mission':(Twist(),0.)}
        return g
    def test_mission_timeout_never_falls_back_to_nav(self):
        g=self.gate()
        with patch.object(self.mod.time,'monotonic',return_value=10.):g.tick()
        self.assertEqual(g.pub.publish.call_args.args[0].linear.x,0.)
    def test_handoff_discards_old_commands(self):
        g=self.gate();g.switch(NS(data=False),NS())
        self.assertFalse(g.owner)
        self.assertEqual(g.commands['nav'][1],0.)
    def test_scan_loss_forces_zero(self):
        g=self.gate();g.owner=False;g.scan_ok=lambda:False
        with patch.object(self.mod.time,'monotonic',return_value=10.):g.tick()
        self.assertEqual(g.pub.publish.call_args.args[0].linear.x,0.)
    def test_scan_stale_by_stamp(self):
        g=self.gate();g.scan_time=10.;g.scan_stamp=1.;g.get_clock=lambda:NS(now=lambda:NS(nanoseconds=10*10**9))
        with patch.object(self.mod.time,'monotonic',return_value=10.):
            self.assertFalse(self.mod.CommandGate.scan_ok(g))


if __name__=='__main__': unittest.main(verbosity=2)
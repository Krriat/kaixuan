"""Native servo wire format and period checks; never imports ROS transport."""
from pathlib import Path
import sys,unittest
import numpy as np
PACKAGE=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PACKAGE/'scripts'))
from cst_lqr_native import PHASES,config_hash,decode_trace,encode_setpoint
class NativeProtocolTests(unittest.TestCase):
    def test_configuration_hash_is_order_independent_and_detects_changes(self):
        self.assertEqual(config_hash({'a':1,'b':2}),config_hash({'b':2,'a':1}))
        self.assertNotEqual(config_hash({'a':1}),config_hash({'a':2}))
    def test_zero_frame_and_feedback_frame_match_fixed_cpp_offsets(self):
        class Input:state=np.zeros(4)
        class Model:
            def inverse_dynamics(self,q,v,a):return np.arange(7,dtype=float)
        class Feedback:
            input_model=Input();model=Model();friction=np.ones(7);friction_speed=.1
            gain=np.arange(175,dtype=float).reshape(7,25)
            inverse_integral_gain=np.eye(7)
            def select(self,*args):pass
        q,v,a=np.arange(7)*.1,np.ones(7)*.01,np.zeros(7)
        f=Feedback();data=np.array(encode_setpoint('line',123.,2.,.5,(q,v,a),f))
        self.assertEqual(len(data),258)
        np.testing.assert_array_equal(data[:6],[1,2,PHASES.index('line'),123.,.5,2.])
        np.testing.assert_allclose(data[6:13],q);np.testing.assert_allclose(data[13:20],v)
        np.testing.assert_allclose(data[34:209].reshape(7,25),f.gain)
        np.testing.assert_allclose(data[209:].reshape(7,7),np.eye(7))
        zero=np.array(encode_setpoint('enable_zero',123.,0.,1.,(q,a,a),f,True))
        self.assertEqual(zero[1],1);np.testing.assert_array_equal(zero[20:],0.)
        with self.assertRaises(ValueError):encode_setpoint('line',123.,0.,float('nan'),(q,v,a),f)
    def test_invalid_trace_rejected_and_true_servo_rows_are_decodable(self):
        trace=np.zeros(98);trace[1]=PHASES.index('return_line');trace[33:40]=np.arange(7)
        values,phase=decode_trace(trace);self.assertEqual(phase,'return_line')
        np.testing.assert_array_equal(values[33:40],np.arange(7))
        for bad in [trace[:-1],np.full(98,np.nan),np.r_[0.,99.,np.zeros(96)]]:
            with self.assertRaises(ValueError):decode_trace(bad)
if __name__=='__main__':unittest.main()

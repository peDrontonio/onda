#!/usr/bin/env python3
"""
Stability Test Script for Manipulator
Tests if PID gains are properly tuned by checking:
1. Initial stability (no movement when at rest)
2. Response to step commands
3. Settling time
4. Overshoot
"""

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
import time
import sys

class StabilityTester:
    def __init__(self):
        rospy.init_node('stability_tester')
        
        # Publishers for commands
        self.publishers = {}
        for i in range(1, 7):
            topic = f'/manipulator/joint_{i}_position_controller/command'
            self.publishers[i] = rospy.Publisher(topic, Float64, queue_size=10)
        
        # Subscriber for joint states
        self.joint_positions = [0.0] * 6
        self.joint_velocities = [0.0] * 6
        rospy.Subscriber('/manipulator/joint_states', JointState, self.joint_state_callback)
        
        rospy.sleep(2.0)  # Wait for connections
        
    def joint_state_callback(self, msg):
        """Update current joint positions"""
        self.joint_positions = list(msg.position)
        self.joint_velocities = list(msg.velocity)
    
    def test_initial_stability(self):
        """Test 1: Check if manipulator stays still at home position"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("TEST 1: Initial Stability (5 seconds)")
        rospy.loginfo("=" * 60)
        rospy.loginfo("Manipulator should stay still without commands...")
        
        initial_positions = self.joint_positions[:]
        rospy.sleep(5.0)
        final_positions = self.joint_positions[:]
        
        max_drift = max(abs(f - i) for f, i in zip(final_positions, initial_positions))
        
        if max_drift < 0.01:  # Less than 0.01 rad drift
            rospy.loginfo(f"✓ PASS: Max drift = {max_drift:.4f} rad (< 0.01)")
            return True
        else:
            rospy.logwarn(f"✗ FAIL: Max drift = {max_drift:.4f} rad (>= 0.01)")
            rospy.logwarn("  → PID gains may be too high (oscillation)")
            rospy.logwarn("  → Try reducing P gains by 50%")
            return False
    
    def test_step_response(self, joint_idx=1, target=0.5):
        """Test 2: Step response - move joint and check behavior"""
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"TEST 2: Step Response (Joint {joint_idx} → {target} rad)")
        rospy.loginfo("=" * 60)
        
        # Record initial position
        initial_pos = self.joint_positions[joint_idx - 1]
        rospy.loginfo(f"Initial position: {initial_pos:.3f} rad")
        
        # Send command
        self.publishers[joint_idx].publish(Float64(target))
        rospy.loginfo(f"Command sent: {target} rad")
        
        # Monitor for 5 seconds
        start_time = rospy.Time.now()
        positions = []
        times = []
        
        while (rospy.Time.now() - start_time).to_sec() < 5.0:
            positions.append(self.joint_positions[joint_idx - 1])
            times.append((rospy.Time.now() - start_time).to_sec())
            rospy.sleep(0.05)
        
        # Analysis
        final_pos = positions[-1]
        error = abs(final_pos - target)
        max_pos = max(positions)
        overshoot = max(0, max_pos - target)
        
        # Find settling time (within 2% of target)
        settling_threshold = 0.02 * abs(target - initial_pos)
        settled = False
        settling_time = 5.0
        
        for t, pos in zip(times, positions):
            if abs(pos - target) < settling_threshold:
                if not settled:
                    settling_time = t
                    settled = True
        
        # Check for oscillations
        oscillations = 0
        for i in range(1, len(positions) - 1):
            if (positions[i] > positions[i-1] and positions[i] > positions[i+1]) or \
               (positions[i] < positions[i-1] and positions[i] < positions[i+1]):
                oscillations += 1
        
        # Results
        rospy.loginfo(f"Final position: {final_pos:.3f} rad")
        rospy.loginfo(f"Error: {error:.4f} rad")
        rospy.loginfo(f"Overshoot: {overshoot:.4f} rad ({overshoot/abs(target-initial_pos)*100:.1f}%)")
        rospy.loginfo(f"Settling time: {settling_time:.2f}s")
        rospy.loginfo(f"Oscillations detected: {oscillations}")
        
        # Evaluate
        passed = True
        
        if error > 0.05:
            rospy.logwarn("✗ High steady-state error")
            rospy.logwarn("  → Consider adding integral gain (I)")
            passed = False
        else:
            rospy.loginfo("✓ Low steady-state error")
        
        if overshoot > 0.1:
            rospy.logwarn("✗ High overshoot")
            rospy.logwarn("  → P gain may be too high")
            rospy.logwarn("  → Increase D gain for damping")
            passed = False
        else:
            rospy.loginfo("✓ Acceptable overshoot")
        
        if oscillations > 10:
            rospy.logwarn("✗ Excessive oscillations")
            rospy.logwarn("  → P gain is too high")
            rospy.logwarn("  → Increase D gain or reduce P")
            passed = False
        else:
            rospy.loginfo("✓ Stable response")
        
        if settling_time < 2.0:
            rospy.loginfo("✓ Fast settling time")
        elif settling_time < 4.0:
            rospy.loginfo("○ Acceptable settling time")
        else:
            rospy.logwarn("✗ Slow settling time")
            rospy.logwarn("  → P gain may be too low")
            passed = False
        
        return passed
    
    def test_velocity_limits(self):
        """Test 3: Check if velocities are reasonable"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("TEST 3: Velocity Check")
        rospy.loginfo("=" * 60)
        
        max_velocity = max(abs(v) for v in self.joint_velocities)
        rospy.loginfo(f"Max velocity: {max_velocity:.3f} rad/s")
        
        if max_velocity > 5.0:
            rospy.logwarn("✗ Excessive velocities detected")
            rospy.logwarn("  → PID gains are too aggressive")
            rospy.logwarn("  → Reduce P and D gains")
            return False
        else:
            rospy.loginfo("✓ Velocities within reasonable limits")
            return True
    
    def run_all_tests(self):
        """Run complete test suite"""
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("MANIPULATOR STABILITY TEST SUITE")
        rospy.loginfo("=" * 60 + "\n")
        
        results = []
        
        # Test 1: Initial stability
        results.append(("Initial Stability", self.test_initial_stability()))
        rospy.sleep(2.0)
        
        # Test 2: Step response
        results.append(("Step Response", self.test_step_response(joint_idx=1, target=0.5)))
        rospy.sleep(2.0)
        
        # Return to home
        rospy.loginfo("Returning to home position...")
        for i in range(1, 7):
            self.publishers[i].publish(Float64(0.0))
        rospy.sleep(3.0)
        
        # Test 3: Velocity check
        results.append(("Velocity Check", self.test_velocity_limits()))
        
        # Summary
        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("TEST SUMMARY")
        rospy.loginfo("=" * 60)
        
        passed = 0
        for test_name, result in results:
            status = "✓ PASS" if result else "✗ FAIL"
            rospy.loginfo(f"{test_name:30} {status}")
            if result:
                passed += 1
        
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"Overall: {passed}/{len(results)} tests passed")
        rospy.loginfo("=" * 60 + "\n")
        
        if passed == len(results):
            rospy.loginfo("🎉 All tests passed! PID tuning looks good.")
            return 0
        else:
            rospy.logwarn("⚠️  Some tests failed. Review PID_TUNING_GUIDE.md for help.")
            return 1

if __name__ == '__main__':
    try:
        tester = StabilityTester()
        exit_code = tester.run_all_tests()
        sys.exit(exit_code)
    except rospy.ROSInterruptException:
        pass

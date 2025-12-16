#!/usr/bin/env python3
"""
Advanced disturbance examples - Programmatic usage

This script demonstrates various ways to use the disturbance service
in automated tests, experiments, and research scenarios.

Author: Manipulator Test Team
"""

import rospy
import numpy as np
from manipulator_msgs.srv import ApplyDisturbance, ApplyDisturbanceRequest
from geometry_msgs.msg import Vector3
import time


class DisturbanceController:
    """Helper class for applying disturbances programmatically"""
    
    def __init__(self, service_name='/apply_disturbance'):
        rospy.init_node('disturbance_controller', anonymous=True)
        rospy.wait_for_service(service_name, timeout=10.0)
        self.service = rospy.ServiceProxy(service_name, ApplyDisturbance)
        self.is_active = False
        
    def apply_force(self, fx=0.0, fy=0.0, fz=0.0):
        """Apply a disturbance force"""
        req = ApplyDisturbanceRequest()
        req.force.x = fx
        req.force.y = fy
        req.force.z = fz
        
        try:
            response = self.service(req)
            if response.success:
                self.is_active = not self.is_active
                rospy.loginfo(f"Disturbance {'activated' if self.is_active else 'deactivated'}")
                return True
            else:
                rospy.logerr(f"Failed: {response.message}")
                return False
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False
    
    def toggle_off(self):
        """Ensure disturbance is off"""
        if self.is_active:
            self.apply_force(0, 0, 0)


def example_1_simple_push():
    """Example 1: Simple push in one direction"""
    print("\n=== Example 1: Simple Push ===")
    controller = DisturbanceController()
    
    # Apply 50N force in +X direction
    controller.apply_force(fx=50.0)
    rospy.sleep(5.0)  # Wait 5 seconds
    
    # Remove force
    controller.apply_force(fx=50.0)
    rospy.sleep(2.0)
    

def example_2_ramp_force():
    """Example 2: Gradually increase force (ramp)"""
    print("\n=== Example 2: Ramp Force ===")
    controller = DisturbanceController()
    
    for force in range(10, 101, 10):
        print(f"Applying {force}N...")
        controller.apply_force(fx=float(force))
        rospy.sleep(2.0)
        controller.toggle_off()
        rospy.sleep(1.0)


def example_3_sinusoidal_force():
    """Example 3: Sinusoidal varying force"""
    print("\n=== Example 3: Sinusoidal Force ===")
    controller = DisturbanceController()
    
    amplitude = 50.0  # Newtons
    frequency = 0.5   # Hz
    duration = 20.0   # seconds
    
    start_time = time.time()
    
    while (time.time() - start_time) < duration:
        t = time.time() - start_time
        force = amplitude * np.sin(2 * np.pi * frequency * t)
        
        # Toggle to update force
        if controller.is_active:
            controller.toggle_off()
        controller.apply_force(fx=force)
        
        rospy.sleep(0.1)  # Update at 10Hz
    
    controller.toggle_off()


def example_4_random_disturbances():
    """Example 4: Random disturbances (wind simulation)"""
    print("\n=== Example 4: Random Disturbances ===")
    controller = DisturbanceController()
    
    for i in range(10):
        fx = np.random.uniform(-30, 30)
        fy = np.random.uniform(-30, 30)
        fz = np.random.uniform(-10, 10)
        
        print(f"Random force {i+1}: [{fx:.1f}, {fy:.1f}, {fz:.1f}] N")
        controller.apply_force(fx, fy, fz)
        rospy.sleep(3.0)
        controller.toggle_off()
        rospy.sleep(1.0)


def example_5_directional_sweep():
    """Example 5: Apply force from different directions"""
    print("\n=== Example 5: Directional Sweep ===")
    controller = DisturbanceController()
    
    force_magnitude = 60.0
    angles = np.linspace(0, 2*np.pi, 8, endpoint=False)
    
    for i, angle in enumerate(angles):
        fx = force_magnitude * np.cos(angle)
        fy = force_magnitude * np.sin(angle)
        
        print(f"Direction {i+1}: {np.degrees(angle):.0f}° - [{fx:.1f}, {fy:.1f}] N")
        controller.apply_force(fx, fy, 0.0)
        rospy.sleep(3.0)
        controller.toggle_off()
        rospy.sleep(1.0)


def example_6_impulse_test():
    """Example 6: Short impulse (collision simulation)"""
    print("\n=== Example 6: Impulse Test ===")
    controller = DisturbanceController()
    
    # Apply high force for short duration
    print("Applying 150N impulse...")
    controller.apply_force(fx=150.0)
    rospy.sleep(0.5)  # Only 500ms
    controller.toggle_off()
    
    rospy.sleep(3.0)  # Wait for settling
    

def example_7_stress_test():
    """Example 7: Stress test with increasing force"""
    print("\n=== Example 7: Stress Test ===")
    controller = DisturbanceController()
    
    forces = [20, 40, 60, 80, 100, 120, 150, 180, 200]
    
    for force in forces:
        print(f"\nStress level: {force}N")
        controller.apply_force(fx=float(force))
        rospy.sleep(5.0)
        
        user_input = input("Continue? (y/n): ")
        controller.toggle_off()
        
        if user_input.lower() != 'y':
            break
        
        rospy.sleep(2.0)


def example_8_coupled_forces():
    """Example 8: Multiple directions simultaneously"""
    print("\n=== Example 8: Coupled Forces ===")
    controller = DisturbanceController()
    
    scenarios = [
        ("Pure X", 50.0, 0.0, 0.0),
        ("Pure Y", 0.0, 50.0, 0.0),
        ("Pure Z", 0.0, 0.0, 50.0),
        ("X+Y diagonal", 40.0, 40.0, 0.0),
        ("X+Y+Z", 30.0, 30.0, 30.0),
    ]
    
    for name, fx, fy, fz in scenarios:
        print(f"\nScenario: {name}")
        controller.apply_force(fx, fy, fz)
        rospy.sleep(4.0)
        controller.toggle_off()
        rospy.sleep(2.0)


def example_9_adaptive_disturbance():
    """Example 9: Adaptive disturbance based on robot state"""
    print("\n=== Example 9: Adaptive Disturbance ===")
    controller = DisturbanceController()
    
    # This would typically subscribe to robot state
    # Here we simulate with time-based logic
    
    print("Simulating adaptive disturbance...")
    for i in range(5):
        # Simulate "detecting" robot position
        phase = i % 3
        
        if phase == 0:
            print("Phase 1: Low disturbance")
            controller.apply_force(fx=20.0)
        elif phase == 1:
            print("Phase 2: Medium disturbance")
            controller.apply_force(fx=50.0)
        else:
            print("Phase 3: High disturbance")
            controller.apply_force(fx=80.0)
        
        rospy.sleep(4.0)
        controller.toggle_off()
        rospy.sleep(2.0)


def example_10_data_collection():
    """Example 10: Collect data for analysis"""
    print("\n=== Example 10: Data Collection ===")
    controller = DisturbanceController()
    
    import csv
    
    # Create CSV file
    with open('/tmp/disturbance_log.csv', 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(['timestamp', 'fx', 'fy', 'fz', 'magnitude'])
        
        forces = np.linspace(10, 100, 10)
        
        for force in forces:
            timestamp = rospy.Time.now().to_sec()
            fx, fy, fz = float(force), 0.0, 0.0
            magnitude = np.sqrt(fx**2 + fy**2 + fz**2)
            
            writer.writerow([timestamp, fx, fy, fz, magnitude])
            
            controller.apply_force(fx, fy, fz)
            rospy.sleep(3.0)
            controller.toggle_off()
            rospy.sleep(1.0)
    
    print("Data saved to /tmp/disturbance_log.csv")


def main():
    """Main function - run selected example"""
    
    examples = {
        '1': ("Simple Push", example_1_simple_push),
        '2': ("Ramp Force", example_2_ramp_force),
        '3': ("Sinusoidal Force", example_3_sinusoidal_force),
        '4': ("Random Disturbances", example_4_random_disturbances),
        '5': ("Directional Sweep", example_5_directional_sweep),
        '6': ("Impulse Test", example_6_impulse_test),
        '7': ("Stress Test", example_7_stress_test),
        '8': ("Coupled Forces", example_8_coupled_forces),
        '9': ("Adaptive Disturbance", example_9_adaptive_disturbance),
        '10': ("Data Collection", example_10_data_collection),
    }
    
    print("\n" + "="*60)
    print("MANIPULATOR DISTURBANCE - ADVANCED EXAMPLES")
    print("="*60)
    print("\nAvailable examples:")
    for key, (name, _) in examples.items():
        print(f"  {key}. {name}")
    print("  0. Run all examples")
    print("  q. Quit")
    
    choice = input("\nSelect example: ").strip()
    
    if choice == 'q':
        return
    elif choice == '0':
        for _, (name, func) in examples.items():
            print(f"\n{'='*60}")
            print(f"Running: {name}")
            print('='*60)
            try:
                func()
            except Exception as e:
                print(f"Error: {e}")
            rospy.sleep(2.0)
    elif choice in examples:
        _, func = examples[choice]
        func()
    else:
        print("Invalid choice!")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nInterrupted by user")

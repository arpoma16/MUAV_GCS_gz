#!/usr/bin/env python3
"""
PX4 Simulation Diagnostic Tool
Monitors PX4 topics and identifies why drones cannot takeoff or enter landing mode
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (
    VehicleStatus,
    VehicleControlMode,
    EstimatorStatus,
    SensorGps,
    VehicleLocalPosition,
    TimesyncStatus,
    FailsafeFlags
)
import sys

class PX4Diagnostics(Node):
    def __init__(self, namespace='px4_3'):
        super().__init__('px4_diagnostics')
        self.namespace = namespace

        # Storage for latest messages
        self.vehicle_status = None
        self.estimator_status = None
        self.gps_status = None
        self.local_position = None
        self.timesync = None
        self.failsafe = None

        # QoS profile compatible with PX4
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscribers
        self.create_subscription(
            VehicleStatus,
            f'/{namespace}/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            qos_profile
        )

        self.create_subscription(
            EstimatorStatus,
            f'/{namespace}/fmu/out/estimator_status',
            self.estimator_status_callback,
            qos_profile
        )

        self.create_subscription(
            SensorGps,
            f'/{namespace}/fmu/out/vehicle_gps_position',
            self.gps_callback,
            qos_profile
        )

        self.create_subscription(
            VehicleLocalPosition,
            f'/{namespace}/fmu/out/vehicle_local_position',
            self.local_position_callback,
            qos_profile
        )

        self.create_subscription(
            TimesyncStatus,
            f'/{namespace}/fmu/out/timesync_status',
            self.timesync_callback,
            qos_profile
        )

        self.create_subscription(
            FailsafeFlags,
            f'/{namespace}/fmu/out/failsafe_flags',
            self.failsafe_callback,
            qos_profile
        )

        # Create timer for periodic diagnostics
        self.create_timer(2.0, self.print_diagnostics)

        self.get_logger().info(f'PX4 Diagnostics started for {namespace}')
        self.get_logger().info('Monitoring drone status...\n')

    def vehicle_status_callback(self, msg):
        self.vehicle_status = msg

    def estimator_status_callback(self, msg):
        self.estimator_status = msg

    def gps_callback(self, msg):
        self.gps_status = msg

    def local_position_callback(self, msg):
        self.local_position = msg

    def timesync_callback(self, msg):
        self.timesync = msg

    def failsafe_callback(self, msg):
        self.failsafe = msg

    def print_diagnostics(self):
        """Print comprehensive diagnostics"""
        print("\n" + "="*80)
        print(f"PX4 DIAGNOSTICS REPORT - {self.namespace}")
        print("="*80)

        # Vehicle Status
        if self.vehicle_status:
            print(f"\n📊 VEHICLE STATUS:")
            print(f"   Arming State: {self.get_arming_state(self.vehicle_status.arming_state)}")
            print(f"   Nav State: {self.get_nav_state(self.vehicle_status.nav_state)}")
            print(f"   Failsafe: {self.vehicle_status.failsafe}")
            print(f"   Pre-flight checks passed: {not self.vehicle_status.pre_flight_checks_pass}")
        else:
            print("\n⚠️  VEHICLE STATUS: No data received")

        # Estimator Status
        if self.estimator_status:
            print(f"\n🧭 ESTIMATOR STATUS:")
            print(f"   Position Horiz Valid: {bool(self.estimator_status.pos_horiz_accuracy < 1.0)}")
            print(f"   Position Vert Valid: {bool(self.estimator_status.pos_vert_accuracy < 1.0)}")
            print(f"   Yaw angle valid: {bool(self.estimator_status.heading_test_ratio < 1.0)}")
            print(f"   Innovation test ratios:")
            print(f"      - Heading: {self.estimator_status.heading_test_ratio:.3f}")
            print(f"      - Velocity horiz: {self.estimator_status.vel_test_ratio:.3f}")
            print(f"      - Velocity vert: {self.estimator_status.vel_test_ratio:.3f}")

            if self.estimator_status.heading_test_ratio > 1.0:
                print(f"   ❌ YAW ESTIMATE ERROR DETECTED!")
        else:
            print("\n⚠️  ESTIMATOR STATUS: No data received")

        # GPS Status
        if self.gps_status:
            print(f"\n🛰️  GPS STATUS:")
            print(f"   Fix type: {self.gps_status.fix_type} (3=3D fix required)")
            print(f"   Satellites: {self.gps_status.satellites_used}")
            print(f"   EPH (Horiz accuracy): {self.gps_status.eph:.2f}m")
            print(f"   EPV (Vert accuracy): {self.gps_status.epv:.2f}m")

            if self.gps_status.fix_type < 3:
                print(f"   ❌ INSUFFICIENT GPS FIX!")
        else:
            print("\n⚠️  GPS STATUS: No data received")

        # Local Position
        if self.local_position:
            print(f"\n📍 LOCAL POSITION:")
            print(f"   XY Valid: {self.local_position.xy_valid}")
            print(f"   Z Valid: {self.local_position.z_valid}")
            print(f"   V_XY Valid: {self.local_position.v_xy_valid}")
            print(f"   V_Z Valid: {self.local_position.v_z_valid}")
            print(f"   Position: ({self.local_position.x:.2f}, {self.local_position.y:.2f}, {self.local_position.z:.2f})")
            print(f"   Heading valid: {self.local_position.heading_good_for_control}")

            if not self.local_position.xy_valid or not self.local_position.z_valid:
                print(f"   ❌ POSITION ESTIMATE INVALID!")
        else:
            print("\n⚠️  LOCAL POSITION: No data received")

        # Timesync Status
        if self.timesync:
            print(f"\n⏰ TIME SYNCHRONIZATION:")
            print(f"   Observed offset: {self.timesync.observed_offset} ns")
            print(f"   Round trip time: {self.timesync.round_trip_time} ns")

            # Check if time offset is too large (>1ms is problematic)
            if abs(self.timesync.observed_offset) > 1000000:  # 1ms in nanoseconds
                print(f"   ⚠️  WARNING: Large time offset detected!")
        else:
            print("\n⚠️  TIMESYNC: No data received")

        # Failsafe Flags
        if self.failsafe:
            print(f"\n🚨 FAILSAFE FLAGS:")
            failsafes_active = []

            if hasattr(self.failsafe, 'mode_req_angular_velocity') and self.failsafe.mode_req_angular_velocity:
                failsafes_active.append("Angular velocity limit")
            if hasattr(self.failsafe, 'mode_req_attitude') and self.failsafe.mode_req_attitude:
                failsafes_active.append("Attitude control")
            if hasattr(self.failsafe, 'mode_req_local_position') and self.failsafe.mode_req_local_position:
                failsafes_active.append("Local position required")
            if hasattr(self.failsafe, 'mode_req_global_position') and self.failsafe.mode_req_global_position:
                failsafes_active.append("Global position required")
            if hasattr(self.failsafe, 'mode_req_home_position') and self.failsafe.mode_req_home_position:
                failsafes_active.append("Home position required")

            if failsafes_active:
                print(f"   Active failsafes: {', '.join(failsafes_active)}")
            else:
                print(f"   ✅ No critical failsafes active")

        # Summary and recommendations
        print(f"\n💡 DIAGNOSIS:")
        self.print_recommendations()

        print("="*80 + "\n")

    def print_recommendations(self):
        """Print diagnostic recommendations based on observed issues"""
        issues = []

        if self.estimator_status and self.estimator_status.heading_test_ratio > 1.0:
            issues.append("❌ YAW ESTIMATE ERROR - Magnetometer calibration or interference issue")

        if self.gps_status and self.gps_status.fix_type < 3:
            issues.append("❌ INSUFFICIENT GPS - Simulation may not have GPS plugin configured")

        if self.local_position and (not self.local_position.xy_valid or not self.local_position.z_valid):
            issues.append("❌ INVALID POSITION ESTIMATE - EKF2 not converged")

        if self.timesync and abs(self.timesync.observed_offset) > 1000000:
            issues.append("⚠️  TIME SYNCHRONIZATION DRIFT - Check PX4_SIM_SPEED_FACTOR")

        if self.vehicle_status and self.vehicle_status.failsafe:
            issues.append("🚨 FAILSAFE ACTIVE - Check failsafe flags above")

        if not issues:
            print("   ✅ No major issues detected")
        else:
            for issue in issues:
                print(f"   {issue}")

    def get_arming_state(self, state):
        states = {
            0: "INIT",
            1: "STANDBY",
            2: "ARMED",
            3: "STANDBY_ERROR"
        }
        return states.get(state, f"UNKNOWN({state})")

    def get_nav_state(self, state):
        states = {
            0: "MANUAL",
            1: "ALTCTL",
            2: "POSCTL",
            3: "AUTO_MISSION",
            4: "AUTO_LOITER",
            5: "AUTO_RTL",
            14: "OFFBOARD",
            17: "AUTO_TAKEOFF",
            18: "AUTO_LAND",
            19: "AUTO_FOLLOW_TARGET",
            20: "AUTO_PRECLAND"
        }
        return states.get(state, f"UNKNOWN({state})")


def main(args=None):
    rclpy.init(args=args)

    # Get namespace from command line or use default
    namespace = sys.argv[1] if len(sys.argv) > 1 else 'px4_3'

    diagnostics = PX4Diagnostics(namespace)

    try:
        rclpy.spin(diagnostics)
    except KeyboardInterrupt:
        pass
    finally:
        diagnostics.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

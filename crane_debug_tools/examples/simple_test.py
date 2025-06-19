#!/usr/bin/env python3
"""
Simple test script for crane debug tools
Demonstrates basic usage of the CLI interface
"""

import subprocess
import sys
import time

def run_crane_skill_command(args):
    """Run a crane_skill command and return the result"""
    try:
        cmd = ['ros2', 'run', 'crane_debug_tools', 'crane_skill'] + args
        print(f"Running: {' '.join(cmd)}")
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
        
        if result.returncode == 0:
            print("✓ Command succeeded")
            if result.stdout:
                print(f"Output: {result.stdout}")
        else:
            print("✗ Command failed")
            if result.stderr:
                print(f"Error: {result.stderr}")
        
        return result.returncode == 0
    except subprocess.TimeoutExpired:
        print("✗ Command timed out")
        return False
    except Exception as e:
        print(f"✗ Command failed with exception: {e}")
        return False

def main():
    print("=== Crane Debug Tools Test Script ===\n")
    
    # Test 1: List available skills
    print("Test 1: List available skills")
    if not run_crane_skill_command(['list']):
        print("Failed to list skills")
        return 1
    
    print("\n" + "="*50 + "\n")
    
    # Test 2: Execute simple skill
    print("Test 2: Execute Sleep skill")
    if not run_crane_skill_command(['run', 'Sleep', '0', 'duration:1.0']):
        print("Failed to execute Sleep skill")
        return 1
    
    print("\n" + "="*50 + "\n")
    
    # Test 3: Execute positioning skill
    print("Test 3: Execute EmplaceRobot skill")
    if not run_crane_skill_command(['run', 'EmplaceRobot', '0', 'target_x:1.0', 'target_y:2.0', 'target_theta:0.5']):
        print("Failed to execute EmplaceRobot skill")
        return 1
    
    print("\n" + "="*50 + "\n")
    
    # Test 4: Execute multi-robot command
    print("Test 4: Execute multi-robot Idle skill")
    if not run_crane_skill_command(['multi', 'Idle', '0,1,2']):
        print("Failed to execute multi-robot Idle skill")
        return 1
    
    print("\n" + "="*50 + "\n")
    
    # Test 5: Execute scenario (if file exists)
    print("Test 5: Execute scenario file")
    scenario_result = run_crane_skill_command(['scenario', 'scenarios/basic_skills_test.json'])
    if not scenario_result:
        print("Note: Scenario test failed (this is expected if crane_robot_skills is not running)")
    
    print("\n=== All CLI tests completed ===")
    return 0

if __name__ == '__main__':
    sys.exit(main())
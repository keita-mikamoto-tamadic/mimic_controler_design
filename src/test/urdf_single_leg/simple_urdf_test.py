#!/usr/bin/env python3
import pinocchio as pin
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
import os

def load_simple_urdf(urdf_path):
    # Try different loading methods
    print("Method 1: Standard loading")
    model1 = pin.buildModelFromUrdf(urdf_path)
    print(f"  nq: {model1.nq}, nv: {model1.nv}, njoints: {model1.njoints}")
    
    print("\nMethod 2: With root joint type")
    model2 = pin.buildModelFromUrdf(urdf_path, pin.JointModelFreeFlyer())
    print(f"  nq: {model2.nq}, nv: {model2.nv}, njoints: {model2.njoints}")
    
    print("\nMethod 3: Fixed base attempt")
    # Create a simple model - use the standard one but understand its structure
    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()
    
    print(f"\nDetailed model info:")
    print(f"  Model name: {model.name}")
    print(f"  nq (config DOF): {model.nq}")
    print(f"  nv (velocity DOF): {model.nv}")
    print(f"  njoints: {model.njoints}")
    
    for i in range(model.njoints):
        if i == 0:
            print(f"  Joint {i}: {model.names[i]} (universe)")
        else:
            joint = model.joints[i]
            print(f"  Joint {i}: {model.names[i]}, type: {joint.shortname()}")
    
    return model, data

def test_configurations(model, data):
    print(f"\nTesting configurations...")
    
    # Get neutral configuration
    q_neutral = pin.neutral(model)
    print(f"Neutral config size: {len(q_neutral)}")
    print(f"Neutral config: {q_neutral}")
    
    # Test with neutral
    pin.forwardKinematics(model, data, q_neutral)
    print("Forward kinematics with neutral config: OK")
    
    return q_neutral

def visualize_simple(model, data, q):
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    
    print(f"\nJoint positions:")
    positions = []
    for i in range(model.njoints):
        pos = data.oMi[i].translation
        print(f"  {model.names[i]}: ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f})")
        positions.append([pos[0], pos[2]])  # x, z for 2D
    
    # Simple 2D plot
    positions = np.array(positions)
    plt.figure(figsize=(10, 6))
    plt.plot(positions[:, 0], positions[:, 1], 'o-', markersize=8, linewidth=2)
    
    for i, name in enumerate(model.names):
        if i < len(positions):
            plt.annotate(name, positions[i], xytext=(5, 5), textcoords='offset points')
    
    plt.grid(True)
    plt.axis('equal')
    plt.xlabel('X [m]')
    plt.ylabel('Z [m]')
    plt.title('Robot Links')
    plt.show()

def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    urdf_path = os.path.join(base_dir, "../../../urdf/mimic_v1_single_leg.urdf")
    
    if not os.path.exists(urdf_path):
        print(f"Error: URDF file not found at {urdf_path}")
        return
    
    model, data = load_simple_urdf(urdf_path)
    q = test_configurations(model, data)
    visualize_simple(model, data, q)

if __name__ == "__main__":
    main()
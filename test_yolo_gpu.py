#!/usr/bin/env python3
"""
Test YOLO Model with GPU
========================
"""

import onnxruntime as ort
import numpy as np
import os

def test_yolo_gpu():
    """Test YOLO model loading with GPU"""
    
    # Find YOLO model
    model_paths = [
        "./src/neural_network_detector/third_party/YOLOs-CPP/models/yolo12n.onnx",
        "./src/neural_network_detector/third_party/models/yolo12n.onnx"
    ]
    
    model_path = None
    for path in model_paths:
        if os.path.exists(path):
            model_path = path
            break
    
    if not model_path:
        print("❌ YOLO model not found")
        return False
    
    print(f"🎯 Testing model: {model_path}")
    
    # Test different provider configurations
    provider_configs = [
        # GPU first
        {
            'name': 'GPU (CUDA + TensorRT)',
            'providers': ['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider']
        },
        # CUDA only
        {
            'name': 'GPU (CUDA only)',
            'providers': ['CUDAExecutionProvider', 'CPUExecutionProvider']
        },
        # CPU only
        {
            'name': 'CPU only',
            'providers': ['CPUExecutionProvider']
        }
    ]
    
    for config in provider_configs:
        print(f"\n🧪 Testing: {config['name']}")
        print("-" * 40)
        
        try:
            # Create session
            session = ort.InferenceSession(
                model_path,
                providers=config['providers']
            )
            
            # Get actual providers used
            actual_providers = session.get_providers()
            print(f"✅ Session created successfully")
            print(f"   Requested: {config['providers']}")
            print(f"   Actual: {actual_providers}")
            
            # Get model info
            inputs = session.get_inputs()
            outputs = session.get_outputs()
            print(f"   Inputs: {len(inputs)}, Outputs: {len(outputs)}")
            
            if inputs:
                input_shape = inputs[0].shape
                input_type = inputs[0].type
                print(f"   Input shape: {input_shape}, Type: {input_type}")
                
                # Test inference with dummy data
                if all(isinstance(dim, int) and dim > 0 for dim in input_shape):
                    dummy_input = np.random.rand(*input_shape).astype(np.float32)
                    
                    # Time the inference
                    import time
                    start_time = time.time()
                    
                    try:
                        output = session.run(None, {inputs[0].name: dummy_input})
                        inference_time = (time.time() - start_time) * 1000
                        print(f"   ✅ Inference successful: {inference_time:.1f}ms")
                        
                        if 'CUDAExecutionProvider' in actual_providers:
                            print(f"   🎮 Using GPU acceleration!")
                        else:
                            print(f"   🖥️  Using CPU")
                            
                    except Exception as e:
                        print(f"   ❌ Inference failed: {e}")
                else:
                    print(f"   ⚠️  Dynamic input shape, skipping inference test")
            
        except Exception as e:
            print(f"❌ Session creation failed: {e}")
    
    return True

if __name__ == "__main__":
    print("🎯 YOLO GPU Test")
    print("================")
    test_yolo_gpu()
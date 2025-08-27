#!/usr/bin/env python3
"""
GPU Debug Script for YOLO
=========================
"""

import subprocess
import sys

def check_nvidia_gpu():
    """Check NVIDIA GPU availability"""
    print("🎮 NVIDIA GPU Check")
    print("=" * 20)
    
    try:
        result = subprocess.run(['nvidia-smi'], capture_output=True, text=True)
        if result.returncode == 0:
            print("✅ NVIDIA GPU detected")
            print(result.stdout)
            return True
        else:
            print("❌ nvidia-smi failed")
            return False
    except FileNotFoundError:
        print("❌ nvidia-smi not found")
        return False

def check_cuda():
    """Check CUDA installation"""
    print("\n🔧 CUDA Check")
    print("=" * 15)
    
    try:
        result = subprocess.run(['nvcc', '--version'], capture_output=True, text=True)
        if result.returncode == 0:
            print("✅ CUDA compiler found")
            print(result.stdout)
            return True
        else:
            print("❌ nvcc not found")
            return False
    except FileNotFoundError:
        print("❌ CUDA not installed")
        return False

def check_onnxruntime():
    """Check ONNX Runtime providers"""
    print("\n🧠 ONNX Runtime Check")
    print("=" * 22)
    
    try:
        import onnxruntime as ort
        providers = ort.get_available_providers()
        print(f"✅ ONNX Runtime version: {ort.__version__}")
        print(f"Available providers: {providers}")
        
        if 'CUDAExecutionProvider' in providers:
            print("✅ CUDA provider available")
            return True
        else:
            print("❌ CUDA provider not available")
            return False
            
    except ImportError:
        print("❌ ONNX Runtime not installed")
        return False

def check_pytorch_cuda():
    """Check PyTorch CUDA support"""
    print("\n🔥 PyTorch CUDA Check")
    print("=" * 21)
    
    try:
        import torch
        print(f"✅ PyTorch version: {torch.__version__}")
        print(f"CUDA available: {torch.cuda.is_available()}")
        if torch.cuda.is_available():
            print(f"CUDA version: {torch.version.cuda}")
            print(f"GPU count: {torch.cuda.device_count()}")
            for i in range(torch.cuda.device_count()):
                print(f"GPU {i}: {torch.cuda.get_device_name(i)}")
        return torch.cuda.is_available()
    except ImportError:
        print("❌ PyTorch not installed")
        return False

def provide_gpu_solutions():
    """Provide solutions for GPU issues"""
    print("\n💡 GPU Solutions")
    print("=" * 16)
    
    print("🔧 For ONNX Runtime GPU support:")
    print("1. Uninstall current version:")
    print("   pip uninstall onnxruntime onnxruntime-gpu")
    print("2. Install GPU version:")
    print("   pip install onnxruntime-gpu")
    print("3. Check CUDA compatibility:")
    print("   https://onnxruntime.ai/docs/execution-providers/CUDA-ExecutionProvider.html")
    print()
    
    print("🔧 For CUDA installation:")
    print("1. Install CUDA toolkit:")
    print("   sudo apt install nvidia-cuda-toolkit")
    print("2. Or download from NVIDIA:")
    print("   https://developer.nvidia.com/cuda-downloads")
    print()
    
    print("🔧 Alternative: Use CPU with optimization:")
    print("1. Lower YOLO input resolution (320x320)")
    print("2. Reduce processing frequency (0.5 Hz)")
    print("3. Use smaller model (yolo8n instead of yolo12n)")

def main():
    print("🔍 GPU Debug for YOLO")
    print("=" * 25)
    
    gpu_ok = check_nvidia_gpu()
    cuda_ok = check_cuda()
    onnx_gpu_ok = check_onnxruntime()
    torch_cuda_ok = check_pytorch_cuda()
    
    print("\n📋 Summary")
    print("=" * 10)
    print(f"NVIDIA GPU:     {'✅' if gpu_ok else '❌'}")
    print(f"CUDA:           {'✅' if cuda_ok else '❌'}")
    print(f"ONNX GPU:       {'✅' if onnx_gpu_ok else '❌'}")
    print(f"PyTorch CUDA:   {'✅' if torch_cuda_ok else '❌'}")
    
    if onnx_gpu_ok:
        print("\n🎉 GPU support ready for YOLO!")
        print("Set use_gpu:=true in YOLO parameters")
    else:
        print("\n⚠️  GPU support not available")
        provide_gpu_solutions()

if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""
System Monitor for YOLO Debugging
=================================
"""

import psutil
import time
import subprocess

def monitor_yolo_process():
    """Monitor YOLO process resources"""
    
    print("🔍 System Monitor")
    print("=================")
    
    while True:
        try:
            # Find YOLO process
            yolo_procs = []
            for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
                try:
                    if 'yolo12_detector_node' in ' '.join(proc.info['cmdline'] or []):
                        yolo_procs.append(proc)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    continue
            
            if yolo_procs:
                for proc in yolo_procs:
                    try:
                        # Get process info
                        cpu_percent = proc.cpu_percent()
                        memory_mb = proc.memory_info().rss / 1024 / 1024
                        status = proc.status()
                        
                        print(f"\n🎯 YOLO Process (PID {proc.pid}):")
                        print(f"   Status: {status}")
                        print(f"   CPU: {cpu_percent:.1f}%")
                        print(f"   Memory: {memory_mb:.1f} MB")
                        
                        # Check if process is stuck
                        if status == 'sleeping' and cpu_percent < 0.1:
                            print("   ⚠️  Process appears to be waiting/sleeping")
                            
                            # Get thread info
                            threads = proc.threads()
                            print(f"   Threads: {len(threads)}")
                            
                        elif status == 'running' and cpu_percent > 50:
                            print("   🔥 Process is actively running (high CPU)")
                            
                    except (psutil.NoSuchProcess, psutil.AccessDenied):
                        print("   ❌ Cannot access process info")
            else:
                print("\n❌ No YOLO process found")
            
            # System info
            cpu_total = psutil.cpu_percent()
            memory = psutil.virtual_memory()
            
            print(f"\n💻 System:")
            print(f"   CPU: {cpu_total:.1f}%")
            print(f"   Memory: {memory.percent:.1f}% ({memory.used/1024/1024/1024:.1f}GB used)")
            
            time.sleep(3)
            
        except KeyboardInterrupt:
            print("\n🛑 Monitoring stopped")
            break
        except Exception as e:
            print(f"\n❌ Monitor error: {e}")
            time.sleep(3)

if __name__ == "__main__":
    monitor_yolo_process()

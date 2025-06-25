#!/usr/bin/env python3
"""
PPCom Sequential Launcher Script

This script launches ppcom_router_new.py first, waits for the initialization
message "PPCom Router initialized successfully!", then starts ppcom_call_new.py.
"""

import subprocess
import sys
import os
import signal
import threading
import time
from pathlib import Path

def main():
    """Main launcher function"""
    print("=" * 60)
    print("PPCom Sequential Launcher")
    print("=" * 60)
    print("This launcher will:")
    print("1. Start ppcom_router_new.py")
    print("2. Wait for 'PPCom Router initialized successfully!' message")
    print("3. Start ppcom_call_new.py")
    print("-" * 60)
    
    # Get the directory where this script is located
    script_dir = Path(__file__).parent.absolute()
    
    router_script = script_dir / "ppcom_router.py"
    call_script = script_dir / "ppcom_call.py"
    
    if not router_script.exists():
        print(f"ERROR: Router script not found at {router_script}")
        return 1
    
    if not call_script.exists():
        print(f"ERROR: Call script not found at {call_script}")
        return 1
    
    router_process = None
    call_process = None
    
    try:
        # Step 1: Launch router and monitor its output
        print(f"[LAUNCHER] Starting PPCom Router: {router_script}")
        
        router_process = subprocess.Popen(
            [sys.executable, str(router_script)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            universal_newlines=True,
            bufsize=1
        )
        
        # Monitor router output for initialization message
        router_initialized = False
        
        def monitor_router():
            nonlocal router_initialized, call_process
            for line in iter(router_process.stdout.readline, ''):
                if line:
                    print(f"[ROUTER] {line.rstrip()}")
                    
                    # Check for initialization message
                    if not router_initialized and "PPCom Router initialized successfully!" in line:
                        print("[LAUNCHER] ✓ PPCom Router initialization detected!")
                        router_initialized = True
                        
                        # Start the call script
                        print(f"[LAUNCHER] Starting PPCom Call: {call_script}")
                        call_process = subprocess.Popen(
                            [sys.executable, str(call_script)],
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            universal_newlines=True,
                            bufsize=1
                        )
                        
                        # Start monitoring call process output
                        def monitor_call():
                            if call_process:
                                for call_line in iter(call_process.stdout.readline, ''):
                                    if call_line:
                                        print(f"[CALL] {call_line.rstrip()}")
                        
                        call_thread = threading.Thread(target=monitor_call, daemon=True)
                        call_thread.start()
        
        # Start router monitoring in a thread
        router_thread = threading.Thread(target=monitor_router, daemon=True)
        router_thread.start()
        
        print("[LAUNCHER] ✓ Both processes started successfully!")
        print("[LAUNCHER] Monitoring processes... (Press Ctrl+C to stop)")
        
        # Wait for processes to complete or be interrupted
        while True:
            # Check if router process has terminated
            router_status = router_process.poll()
            if router_status is not None:
                print(f"[LAUNCHER] Router process terminated with code {router_status}")
                break
            
            # Check if call process has terminated (if it was started)
            if call_process:
                call_status = call_process.poll()
                if call_status is not None:
                    print(f"[LAUNCHER] Call process terminated with code {call_status}")
                    # Call process finished, but keep router running
                    print("[LAUNCHER] Call process completed, router continues running...")
                    call_process = None  # Clear reference
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\n[LAUNCHER] Keyboard interrupt received. Shutting down...")
    except Exception as e:
        print(f"[LAUNCHER] Unexpected error: {e}")
    finally:
        # Clean up processes
        print("[LAUNCHER] Cleaning up processes...")
        
        if call_process and call_process.poll() is None:
            print("[LAUNCHER] Terminating call process...")
            call_process.terminate()
            try:
                call_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print("[LAUNCHER] Force killing call process...")
                call_process.kill()
        
        if router_process and router_process.poll() is None:
            print("[LAUNCHER] Terminating router process...")
            router_process.terminate()
            try:
                router_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                print("[LAUNCHER] Force killing router process...")
                router_process.kill()
        
        print("[LAUNCHER] Cleanup complete.")
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
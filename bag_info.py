#!/usr/bin/env python3
import sys
from pathlib import Path
from rosbags.highlevel import AnyReader

def print_bag_info(bag_path):
    path = Path(bag_path)
    if not path.exists():
        print(f"Error: File '{bag_path}' does not exist.")
        sys.exit(1)

    print(f"Reading info for: {bag_path} ...\n")
    
    try:
        with AnyReader([path]) as reader:
            duration = reader.duration / 1e9
            print(f"path:        {path.resolve()}")
            print(f"duration:    {duration:.2f} s")
            print(f"start:       {reader.start_time / 1e9:.6f}")
            print(f"end:         {reader.end_time / 1e9:.6f}")
            print(f"messages:    {reader.message_count}")
            print("topics:")
            
            # Afficher les topics proprement
            for topic, connection in reader.topics.items():
                print(f"  - {topic: <30} {connection.msgcount: >8} msgs    : {connection.msgtype}")
                
    except Exception as e:
        print(f"Error reading bag: {e}")

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python3 bag_info.py <path_to_bag>")
        sys.exit(1)
        
    print_bag_info(sys.argv[1])

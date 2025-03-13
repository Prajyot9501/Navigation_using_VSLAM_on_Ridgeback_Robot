#!/usr/bin/env python3
import sqlite3
import os
import argparse

def print_db_contents(db_path):
    """Print all objects in the semantic map database"""
    if not os.path.exists(db_path):
        print(f"Database file not found: {db_path}")
        return
        
    conn = sqlite3.connect(db_path)
    c = conn.cursor()
    
    # Get total count
    c.execute("SELECT COUNT(*) FROM objects")
    count = c.fetchone()[0]
    print(f"Total objects in database: {count}")
    
    # Get class distribution
    c.execute("SELECT class_name, COUNT(*) FROM objects GROUP BY class_name")
    classes = c.fetchall()
    
    print("\nObjects by class:")
    for class_name, count in classes:
        print(f"  {class_name}: {count}")
    
    # Get all objects
    c.execute("SELECT id, class_name, instance_id, x, y, z, confidence, detection_count FROM objects")
    objects = c.fetchall()
    
    print("\nAll objects:")
    for obj in objects:
        obj_id, class_name, instance_id, x, y, z, confidence, detection_count = obj
        print(f"ID: {obj_id}, Class: {class_name}, Instance: {instance_id}")
        print(f"  Position: ({x:.2f}, {y:.2f}, {z:.2f})")
        print(f"  Confidence: {confidence:.2f}, Detections: {detection_count}")
    
    conn.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Check semantic map database contents')
    parser.add_argument('--db', type=str, default='~/semantic_map.db', help='Path to the database file')
    
    args = parser.parse_args()
    db_path = os.path.expanduser(args.db)
    
    print_db_contents(db_path)
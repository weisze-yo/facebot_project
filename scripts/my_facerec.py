#!/usr/bin/env python

import argparse
import pickle
import os
import cv2
import face_recognition
import rospy
import numpy as np
import pandas as pd
import csv

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from collections import Counter
from pathlib import Path
from datetime import datetime
from PIL import Image as PilImage, ImageDraw


DEFAULT_ENCODINGS_PATH = Path("/home/mustar/catkin_ws/src/facebot_package/data/output/encodings.pkl")
BOUNDING_BOX_COLOR = "blue"
TEXT_COLOR = "white"

# Create directories if they don't already exist
if not os.path.exists("/home/mustar/catkin_ws/src/facebot_package/data/training"):
    os.makedirs("/home/mustar/catkin_ws/src/facebot_package/data/training")
if not os.path.exists("/home/mustar/catkin_ws/src/facebot_package/data/output"):
    os.makedirs("/home/mustar/catkin_ws/src/facebot_package/data/output")
if not os.path.exists("/home/mustar/catkin_ws/src/facebot_package/data/validation"):
    os.makedirs("/home/mustar/catkin_ws/src/facebot_package/data/validation")

parser = argparse.ArgumentParser(description="Recognize faces in an image")
parser.add_argument("--train", action="store_true", help="Train on input data")
parser.add_argument("--validate", action="store_true", help="Validate trained model")
parser.add_argument("--test", action="store_true", help="Test the model with an unknown image")
parser.add_argument("-m", action="store", default="hog", choices=["hog", "cnn"], help="Which model to use for training: hog (CPU), cnn (GPU)")
parser.add_argument("-f", action="store", help="Path to an image with an unknown face")
args = parser.parse_args()

bridge = CvBridge()


def encode_known_faces(model="hog", encodings_location=DEFAULT_ENCODINGS_PATH):
    """
    Loads images in the training directory and builds a dictionary of their
    names and encodings.
    """
    names = []
    encodings = []

    training_dir = "/home/mustar/catkin_ws/src/facebot_package/data/training"
    for dirpath, dirnames, filenames in os.walk(training_dir):
        for filename in filenames:
            name = os.path.basename(dirpath)
            image_path = os.path.join(dirpath, filename)

            image = face_recognition.load_image_file(image_path)

            face_locations = face_recognition.face_locations(image, model=model)
            face_encodings = face_recognition.face_encodings(image, face_locations)

            for encoding in face_encodings:
                names.append(name)
                encodings.append(encoding)

    name_encodings = {"names": names, "encodings": encodings}
    with open(str(encodings_location), "wb") as f:
        pickle.dump(name_encodings, f)

def load_encodings(encodings_location):
    """
    Load encodings from the given file location.
    """
    with open(str(encodings_location), "rb") as f:
        loaded_encodings = pickle.load(f)
    return loaded_encodings

def validate(loaded_encodings, model="hog"):
    """
    Runs recognize_faces on a set of images with known faces to validate
    known encodings.
    """
    validation_dir = "/home/mustar/catkin_ws/src/facebot_package/data/validation"
    for dirpath, dirnames, filenames in os.walk(validation_dir):
        for filename in filenames:
            image_path = os.path.join(dirpath, filename)
            image = face_recognition.load_image_file(image_path)
            recognize_faces(image, loaded_encodings, model=model)


def recognize_faces(image, loaded_encodings, model="hog"):
    """
    Given an unknown image, get the locations and encodings of any faces and
    compares them against the known encodings to find potential matches.
    """
    face_names = []

    input_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    input_face_locations = face_recognition.face_locations(input_image, model=model)
    input_face_encodings = face_recognition.face_encodings(input_image, input_face_locations)

    pillow_image = PilImage.fromarray(input_image)
    draw = ImageDraw.Draw(pillow_image)

    for bounding_box, unknown_encoding in zip(input_face_locations, input_face_encodings):
        name = _recognize_face(unknown_encoding, loaded_encodings)
        if not name:
            name = "Unknown"
        face_names.append(name)  # Update the face_names list
        _display_face(draw, bounding_box, name)

    del draw
    img_with_overlay = bridge.cv2_to_imgmsg(np.array(pillow_image), encoding="passthrough")
    img_pub.publish(img_with_overlay)
    pillow_image.show()

    return face_names


def _recognize_face(unknown_encoding, loaded_encodings):
    """
    Given an unknown encoding and all known encodings, find the known
    encoding with the most matches.
    """
    boolean_matches = face_recognition.compare_faces(loaded_encodings["encodings"], unknown_encoding)
    votes = Counter(
        name
        for match, name in zip(boolean_matches, loaded_encodings["names"])
        if match
    )
    if votes:
        return votes.most_common(1)[0][0]

def _display_face(draw, bounding_box, name):
    """
    Draws bounding boxes around faces, a caption area, and text captions.
    """
    top, right, bottom, left = bounding_box
    draw.rectangle(((left, top), (right, bottom)), outline=BOUNDING_BOX_COLOR)
    text_width, text_height = draw.textsize(name)
    text_left = left
    text_top = bottom - text_height
    text_right = left + text_width
    text_bottom = bottom
    draw.rectangle(
        ((text_left, text_top), (text_right, text_bottom)),
        fill=BOUNDING_BOX_COLOR,
        outline=BOUNDING_BOX_COLOR,
    )
    draw.text(
        (text_left, text_top),
        name,
        fill=TEXT_COLOR,
    )

def camera_callback(loaded_encodings):
    # Create a VideoCapture object for the default camera
    capture = cv2.VideoCapture(2)  
    rospy.loginfo("Please stand in front of the camera...")
    
    while True:
        ret, frame = capture.read()  # Read a frame from the camera

        if not ret:
            rospy.loginfo("Failed to capture frame from camera.")
            break
            
        rospy.loginfo("Image captured. Face recognition and attendance login processing...")
        # Exit the loop after processing the image
        break
        
    face_names = recognize_faces(frame, loaded_encodings, model=args.m)
    record_attendance(face_names)
    capture.release()  # Release the camera



def record_attendance(face_names):
    """
    Records the attendance by appending the name and current time to the attendance data list.
    """
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    names = []
    attendance_data = []

    for face_name in face_names:
        if face_name != "Unknown" and face_name not in names:
            names.append(face_name)
            attendance_data.append({"Name": face_name, "Time": current_time})
            rospy.loginfo("Hello {}, your attendance was recorded.".format(face_name))
        else:
            rospy.loginfo("You are not our student.")

    attendance_path = "/home/mustar/catkin_ws/src/facebot_package/data/output/attendance.csv"

    if os.path.exists(attendance_path):
        with open(attendance_path, mode="a") as file:
            writer = csv.DictWriter(file, fieldnames=["Name", "Time"])
            for data in attendance_data:
                writer.writerow(data)
    else:
        with open(attendance_path, mode="w") as file:
            writer = csv.DictWriter(file, fieldnames=["Name", "Time"])
            writer.writeheader()
            for data in attendance_data:
                writer.writerow(data)


if __name__ == "__main__":
    rospy.init_node('face_recognition_attendance_node')
    img_pub = rospy.Publisher('/face_recognition_attendance_node/image', Image, queue_size=1)
    loaded_encodings = load_encodings(DEFAULT_ENCODINGS_PATH)  # Initialize loaded_encodings

    if args.train:
        encode_known_faces(model=args.m)
        rospy.loginfo("Training completed.")
        exit()  # Exit the script after training is completed

    elif args.validate:
        validate(loaded_encodings, model=args.m)
        rospy.loginfo("Validation completed.")
        exit()  # Exit the script after validation is completed

    elif args.test:
        camera_callback(loaded_encodings)
        #capture.release()  # Release the camera
        rospy.loginfo("Have a nice day!")
        exit()  # Exit the script after face recognition and attendance login is completed

    else:
        rospy.loginfo("No action specified.")
